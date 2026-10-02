"""Adapt Python reference output to non-commanding Phase 4 messages."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_interface.msg import (
    ControlCycleTiming,
    ControlShadowDecision,
    ControlShadowRequest,
    ControlStream,
)


@dataclass(frozen=True)
class WorkerSnapshot:
    """ROS-independent inputs used to qualify one shadow decision."""

    velocity: tuple[float, ...] | None
    plan_age_s: float | None
    timing_age_s: float | None
    plan_valid: bool
    plan_reason: str
    controller_state: str
    controller_reason: str
    estimator_health: str
    estimator_state_timestamp_ns: int


def qualify_snapshot(snapshot: WorkerSnapshot, maximum_age_s: float) -> str:
    """Return ``accepted`` or a stable worker-side rejection reason."""
    if snapshot.velocity is None:
        return "planned_control_unavailable"
    if len(snapshot.velocity) != 6:
        return "planned_control_dimension_invalid"
    if not all(math.isfinite(value) for value in snapshot.velocity):
        return "planned_control_nonfinite"
    if snapshot.plan_age_s is None or snapshot.plan_age_s > maximum_age_s:
        return "planned_control_stale"
    if snapshot.timing_age_s is None:
        return "planner_timing_unavailable"
    if snapshot.timing_age_s > maximum_age_s:
        return "planner_timing_stale"
    if not snapshot.plan_valid:
        detail = snapshot.plan_reason or "unknown"
        return f"planner_invalid:{detail}"
    return "accepted"


class ControlShadowWorker(Node):
    """Mirror the reference controller result into the shadow protocol."""

    def __init__(self) -> None:
        super().__init__("control_shadow_worker")
        self._maximum_age_s = float(self.declare_parameter(
            "maximum_input_age_s", 0.20).value)
        if self._maximum_age_s <= 0.0:
            raise ValueError("maximum_input_age_s must be positive")

        self._velocity: tuple[float, ...] | None = None
        self._plan_received_ns: int | None = None
        self._timing: ControlCycleTiming | None = None
        self._timing_received_ns: int | None = None
        self._controller_state = "UNKNOWN"
        self._controller_reason = "status_unavailable"
        self._estimator_health = "UNKNOWN"
        self._pending_request = None
        self._pending_request_ns = None
        self._request_count = 0
        self._timing_count = 0
        self._plan_count = 0
        self._decision_count = 0

        reliable_one = QoSProfile(depth=1)
        reliable_one.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._decision_pub = self.create_publisher(
            ControlShadowDecision,
            "/catheter_mppi/shadow_decision",
            reliable_one,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/catheter_mppi/shadow_worker_status", 10)
        self.create_timer(1.0, self._publish_status)
        self.create_subscription(
            ControlShadowRequest,
            "/catheter_mppi/shadow_request",
            self._request_callback,
            reliable_one,
        )
        self.create_subscription(
            ControlStream,
            "/catheter_mppi/planned_control",
            self._plan_callback,
            10,
        )
        self.create_subscription(
            ControlCycleTiming,
            "/catheter_mppi/control_cycle_timing",
            self._timing_callback,
            sensor_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            "/catheter_mppi/status",
            self._status_callback,
            10,
        )
        self.get_logger().info(
            "non-commanding Python shadow worker started; "
            "command publisher absent")

    def _plan_callback(self, message: ControlStream) -> None:
        self._plan_count += 1
        self._velocity = tuple(float(value) for value in message.joint_vel)
        self._plan_received_ns = time.monotonic_ns()
        if (self._pending_request is not None
                and self._pending_request_ns is not None
                and self._timing_received_ns is not None
                and self._timing_received_ns >= self._pending_request_ns):
            request = self._pending_request
            started_ns = self._pending_request_ns
            self._pending_request = None
            self._pending_request_ns = None
            self._publish_request_decision(request, started_ns)

    def _timing_callback(self, message: ControlCycleTiming) -> None:
        self._timing_count += 1
        self._timing = message
        self._timing_received_ns = time.monotonic_ns()

    def _status_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if "catheter_mppi" not in status.name:
                continue
            values = {item.key: item.value for item in status.values}
            self._controller_state = status.message or "UNKNOWN"
            self._controller_reason = values.get("reason", "unknown")
            self._estimator_health = values.get(
                "estimator_health", "UNKNOWN")
            break

    def _publish_status(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "control/shadow_worker"
        status.hardware_id = "non_actuating"
        status.level = DiagnosticStatus.OK
        status.message = "SHADOW_WORKER"
        status.values = [
            KeyValue(key="command_publisher_present", value="false"),
            KeyValue(key="requests", value=str(self._request_count)),
            KeyValue(key="timing_samples", value=str(self._timing_count)),
            KeyValue(key="plans", value=str(self._plan_count)),
            KeyValue(key="decisions", value=str(self._decision_count)),
        ]
        message.status = [status]
        self._status_pub.publish(message)

    def _snapshot(self, now_ns: int) -> WorkerSnapshot:
        timing = self._timing
        return WorkerSnapshot(
            velocity=self._velocity,
            plan_age_s=(None if self._plan_received_ns is None else
                        1.0e-9*(now_ns-self._plan_received_ns)),
            timing_age_s=(None if self._timing_received_ns is None else
                          1.0e-9*(now_ns-self._timing_received_ns)),
            plan_valid=bool(timing.plan_valid) if timing is not None else False,
            plan_reason=str(timing.plan_reason) if timing is not None else "",
            controller_state=self._controller_state,
            controller_reason=self._controller_reason,
            estimator_health=self._estimator_health,
            estimator_state_timestamp_ns=(
                int(timing.estimator_state_timestamp_ns)
                if timing is not None else 0),
        )

    def _request_callback(self, request: ControlShadowRequest) -> None:
        self._request_count += 1
        # Keep only the latest useful request. A result is emitted only after a
        # later Python planning cycle publishes both timing and control output,
        # so an old plan is never relabeled with a newer input watermark.
        self._pending_request = request
        self._pending_request_ns = time.monotonic_ns()

    def _publish_request_decision(
            self, request: ControlShadowRequest, started_ns: int) -> None:
        snapshot = self._snapshot(time.monotonic_ns())
        reason = qualify_snapshot(snapshot, self._maximum_age_s)

        decision = ControlShadowDecision()
        decision.header.stamp = self.get_clock().now().to_msg()
        decision.header.frame_id = "control_shadow_worker"
        decision.schema_version = int(request.schema_version)
        decision.shell_epoch = int(request.shell_epoch)
        decision.request_sequence = int(request.request_sequence)
        decision.device_sequence = int(request.device_sequence)
        decision.marker_sequence = int(request.marker_sequence)
        decision.manager_sequence = int(request.manager_sequence)
        decision.target_revision = int(request.target_revision)
        decision.estimator_state_timestamp_ns = (
            snapshot.estimator_state_timestamp_ns)
        decision.computation_start_steady_ns = started_ns
        decision.valid = reason == "accepted"
        decision.controller_state = snapshot.controller_state
        decision.reason = reason
        decision.logical_velocity = (
            list(snapshot.velocity) if snapshot.velocity is not None and
            len(snapshot.velocity) == 6 else [0.0]*6)
        decision.projection_applied = True
        decision.estimator_health = snapshot.estimator_health
        decision.planner_reason = snapshot.plan_reason
        decision.computation_end_steady_ns = time.monotonic_ns()
        self._decision_pub.publish(decision)
        self._decision_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlShadowWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
