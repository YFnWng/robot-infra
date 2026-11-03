"""Non-commanding dry-run controller: reduced filter plus CatheterMppi over its rollout backend.

Research composition selected by estimator_backend=reduced_filter_dry_run. It has no command
publisher, manager client, arm service or mode claim, and refuses command_output_enabled. One owner
callback holds the filter and plans in the same thread right after a filter commit, when the plan
period is due (plan_due): a plan never waits on the owner lock or overlaps a filter step. Plans and
status are published on private topics only.
"""
from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path
import signal
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
import torch

from control_interface.msg import DeviceStream

from .orchestration.estimator_owner import encoder_history_qos, marker_measurement, stamp_ns
from .orchestration.parameters import declare_parameters
from .orchestration.reduced_filter_owner import OrderedInbox, ReducedFilterOwner
from .orchestration.reduced_filter_settings import (
    PLAN_OVERDUE_PERIODS, load_reduced_filter, plan_due, reduced_filter_contract, reduced_planner_config,
    require_filter_owned_switches_off)
from .planning.mppi import CatheterMppi
from .safety.hardware_contract import load_hardware_contract

SOURCE_NAME = "autonomy"
BACKEND = "reduced_filter_dry_run"


class ReducedFilterDryRunNode(Node):
    """catheter_mppi in dry-run form: estimates and plans, never commands."""

    def __init__(self, owner_factory=None):
        super().__init__("catheter_mppi")
        declare_parameters(self, source_name=SOURCE_NAME)

        def value(name):
            return self.get_parameter(name).value
        if str(value("estimator_backend")) != BACKEND:
            raise ValueError(f"estimator_backend must be {BACKEND}")
        if bool(value("command_output_enabled")):
            raise ValueError("the reduced-filter dry run never commands; command_output_enabled must be false")
        require_filter_owned_switches_off(value)
        self.frame_id = str(value("frame_id"))
        self.plan_rate_hz = float(value("plan_rate_hz"))
        period_ms = float(value("reduced_filter_image_period_ms"))
        startup = int(value("reduced_filter_startup_pending_images"))
        if not (math.isfinite(self.plan_rate_hz) and self.plan_rate_hz > 0 and 0 <= period_ms < 1000 and startup >= 3):
            raise ValueError("positive plan rate, image period in [0,1000) ms and start-up queue of at least 3 required")
        torch.set_num_threads(int(value("torch_intraop_threads")))
        torch.set_num_interop_threads(int(value("torch_interop_threads")))
        self.contract = reduced_filter_contract(load_hardware_contract(str(value("limits_file")), str(value("catheter"))),
                                         value("controller_velocity_max"), float(value("reduced_filter_tendon_upper_mm")))
        config = reduced_planner_config(self)
        if config.planning_deadline_s >= 1.0/self.plan_rate_hz:
            raise ValueError("planning deadline must be shorter than the plan period")
        self.device = str(value("device"))
        if owner_factory is None:
            owner_factory = self._load_owner
        backend, stream = owner_factory(str(value("reduced_filter_artifact_root")), period_ms, startup)
        self.planner = CatheterMppi(backend, self.contract, config)
        self.owner = ReducedFilterOwner(stream, backend, candidate_counts=self.planner.candidate_counts(),
                                        step_s=config.point_rollout_step_s, clock=time.perf_counter)
        self.report_dir = str(value("reduced_filter_report_dir"))
        self.inbox = OrderedInbox()
        self.counts = Counter()
        self.filter_rows, self.plan_rows = [], []
        self.target = None
        self.target_lock = threading.Lock()
        self.warmed, self.warmup_ms = False, None
        self.last_plan_s = None          # wall clock of the last plan, for the overdue diagnostic only
        self.last_plan_image_ns = None   # committed image of the last plan; drives plan_due
        input_group, owner_group, io_group = (MutuallyExclusiveCallbackGroup() for _ in range(3))
        self.create_subscription(DeviceStream, "/device/state", self._device_cb, encoder_history_qos(),
                                 callback_group=input_group)
        self.create_subscription(PointCloud, str(value("marker_topic")), self._marker_cb, qos_profile_sensor_data,
                                 callback_group=input_group)
        self.create_subscription(PointStamped, "/catheter_mppi/target_tip", self._target_cb, 10,
                                 callback_group=io_group)
        self.status_pub = self.create_publisher(DiagnosticArray, "/catheter_mppi/status", 10)
        self.plan_pub = self.create_publisher(String, "/catheter_mppi/dry_run_plan", 10)
        self.create_timer(0.005, self._owner_tick, callback_group=owner_group)
        self.create_timer(1.0, self._status_tick, callback_group=io_group)
        self.get_logger().info(
            f"catheter_mppi reduced-filter DRY RUN: never commands; {config.samples} samples, "
            f"{self.plan_rate_hz:g} Hz, tendon upper {self.contract.position_upper[2]:g} mm")

    def _load_owner(self, artifact_root, period_ms, startup):
        return load_reduced_filter(artifact_root, period_ms, startup, self.device)

    def _device_cb(self, message):
        if message.predicate != DeviceStream.ENC:
            return
        if not self.inbox.put(("encoder", stamp_ns(message.header), list(message.data), time.monotonic_ns())):
            self.counts["inbox_overflow_encoder"] += 1

    def _marker_cb(self, message):
        arrival = time.monotonic_ns()
        try:
            stamp, points, _ = marker_measurement(message, self.frame_id)
        except ValueError as error:
            self.counts["invalid_marker:"+str(error)] += 1
            return
        if not self.inbox.put(("marker", stamp, points*1000, arrival)):
            self.counts["inbox_overflow_marker"] += 1

    def _target_cb(self, message):
        point = np.array([message.point.x, message.point.y, message.point.z], dtype=np.float64)
        if message.header.frame_id != self.frame_id or not np.all(np.isfinite(point)):
            self.counts["invalid_target"] += 1
            return
        with self.target_lock:
            changed = self.target is None or not np.array_equal(point, self.target)
            self.target = point
        if changed:
            self.planner.reset_capture_target()

    def _owner_tick(self):
        events, overflowed = self.inbox.drain()
        if overflowed:
            self.owner.mark_history_lost()
        for kind, stamp, values, _arrival in events:
            try:
                if kind == "encoder":
                    self.owner.add_encoder(stamp, values)
                    self.counts["encoders_received"] += 1
                else:
                    self.counts["images_received"] += 1
                    self.owner.add_image(stamp, values)
            except ValueError as error:
                self.counts[f"invalid_{kind}:{error}"] += 1
        started_ns = time.time_ns()   # wall clock, comparable with plan rows' wall_ns
        result = self.owner.process()
        if result is None:
            return
        self.counts["status_"+result.status] += 1
        self.filter_rows.append(dict(
            source_ns=result.source_ns, wall_ns=started_ns, status=result.status, reason=result.reason,
            step_ms=result.seconds*1e3,
            position_mm=None if result.position_mm is None else result.position_mm.tolist(),
            lambda_value=result.lambda_value, probability_engaged=result.probability_engaged,
            consecutive_rejections=result.consecutive_rejections))
        # A skipped plan (warm-up, no target, not ready) keeps the due time, so the next commit retries.
        if (result.source_ns is not None
                and plan_due(self.last_plan_image_ns, int(result.source_ns), 1.0/self.plan_rate_hz)
                and self._plan_tick()):
            self.last_plan_image_ns = int(result.source_ns)
            self.last_plan_s = time.perf_counter()

    def _warm_up(self):
        """One discarded plan at first readiness, so first-call costs never meet a deadline."""
        rooted = self.owner.plan_root()
        result = self.owner.last_result
        if rooted.root is None or result is None or result.position_mm is None:
            return
        started = time.perf_counter()
        with self.target_lock:
            target = None if self.target is None else self.target.copy()
        aim = target if target is not None else np.asarray(result.position_mm)/1000.0
        try:
            self.planner.plan(rooted.root, rooted.joint_position, aim, deadline_started_s=started)
        except ValueError as error:
            self.counts["plan_warmup_error:"+str(error)[:80]] += 1
        self.planner.reset()
        self.warmup_ms = (time.perf_counter()-started)*1e3
        self.warmed = True

    def _plan_tick(self):
        """One plan cycle; True when it ran (a plan row was produced), False when skipped."""
        started = time.perf_counter()
        health = self.owner.health()
        if health.ready and not self.warmed:
            self._warm_up()
            return False
        with self.target_lock:
            target = None if self.target is None else self.target.copy()
        if target is None or not health.ready:
            self.counts["plan_skipped:" + ("no_target" if target is None else health.estimator_health.lower())] += 1
            return False
        rooted = self.owner.plan_root()
        row = dict(wall_ns=time.time_ns(), root_ms=rooted.seconds*1e3, estimator_health=health.estimator_health)
        if rooted.root is None:
            row.update(valid=False, reason=rooted.reason)
        else:
            try:
                plan = self.planner.plan(rooted.root, rooted.joint_position, target, deadline_started_s=started)
            except ValueError as error:
                plan = None
                row.update(valid=False, reason=f"planner_error: {error}"[:200])
        if rooted.root is not None and plan is not None:
            elapsed = time.perf_counter()-started
            row.update(
                image_ns=rooted.root.image_ns, plan_ns=rooted.root.plan_ns,
                root_age_ms=(rooted.root.plan_ns-rooted.root.image_ns)*1e-6, valid=bool(plan.valid),
                reason=plan.reason, planner_ms=plan.elapsed_s*1e3, rollout_ms=plan.rollout_ms, tick_ms=elapsed*1e3,
                sample_projection_ms=plan.sample_projection_ms, cost_weighting_ms=plan.cost_weighting_ms,
                update_projection_ms=plan.update_projection_ms,
                deadline_missed=bool(elapsed > self.planner.config.planning_deadline_s),
                blocked_candidates=int(plan.blocked_candidate_count),
                command_logical_velocity=np.asarray(plan.command_logical_velocity).tolist(),
                best_tip_sequence_m=None if plan.best_tip_sequence_m is None
                else np.asarray(plan.best_tip_sequence_m).tolist(),
                target_m=target.tolist())
        self.counts["plan_"+("valid" if row["valid"] else "invalid")] += 1
        self.counts["plan_deadline_missed"] += int(row.get("deadline_missed", False))
        self.plan_rows.append(row)
        self.plan_pub.publish(String(data=json.dumps(row)))
        return True

    def summary(self):
        health = self.owner.health()
        aligner = self.owner.stream.aligner
        ticks = [row["tick_ms"] for row in self.plan_rows if "tick_ms" in row]
        return dict(runtime_family="reduced_filter_dry_run", command_capable=False, command_output_enabled=False,
                    estimator_health=health.estimator_health, ready=health.ready, reason=health.reason,
                    images_dropped=aligner.images_dropped, images_skipped=aligner.images_skipped,
                    plan_tick_ms_p99=float(np.percentile(ticks, 99)) if ticks else None,
                    warmup_plan_ms=self.warmup_ms,
                    kernel_preparations_ms={str(laws): seconds*1e3 for laws, seconds in self.owner.preparations},
                    counts=dict(self.counts))

    def _status_tick(self):
        last = self.last_plan_s
        if (last is not None and time.perf_counter()-last > PLAN_OVERDUE_PERIODS/self.plan_rate_hz
                and self.owner.health().ready):
            self.counts["plan_overdue"] += 1   # commit-triggered planning has stalled (for example, no images)
        summary = self.summary()
        status = DiagnosticStatus(name="catheter_mppi", hardware_id="reduced_filter_dry_run",
                                  message="dry run; never commands")
        status.level = DiagnosticStatus.OK if summary["ready"] else DiagnosticStatus.WARN
        status.values = [KeyValue(key=key, value=json.dumps(item) if isinstance(item, dict) else str(item))
                         for key, item in summary.items()]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(message)

    def write_report(self):
        if not self.report_dir:
            return
        path = Path(self.report_dir)/"reduced_filter_dry_run.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        partial = path.with_suffix(".json.partial")
        partial.write_text(json.dumps(dict(self.summary(), filter_rows=self.filter_rows, plan_rows=self.plan_rows,
                                           artifacts=self.owner.stream.artifacts.resources))+"\n")
        partial.replace(path)


def finish(executor, node):
    """Stop and write the report. A session stop delivers SIGINT twice (supervisor and launch); the
    second must not interrupt executor.shutdown() and lose the report, so SIGINT is ignored from here."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        executor.shutdown()
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ReducedFilterDryRunNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # SIGINT shuts the context down while a worker may be entering its wait set; that is a normal stop.
        if rclpy.ok():
            raise
    finally:
        finish(executor, node)


if __name__ == "__main__":
    main()
