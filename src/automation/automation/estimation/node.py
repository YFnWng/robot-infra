"""Live catheter and sheath shape estimation from named coil transforms."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import time
from typing import Any

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_igtl_bridge.msg import PointArray, String, Transform

from .protocol import (
    DeviceConfig,
    ShapeConfig,
    ShapeConfigError,
    coil_points_mm_to_positions,
    fit_straight_segment_mm,
    parse_shape_config,
    stream_is_stale,
)


CONFIG_DEVICE = "estimator/config"
STATUS_DEVICE = "estimator/status"


@dataclass
class DeviceRuntime:
    config: DeviceConfig
    estimator: Any = None
    coil_positions_mm: dict[str, tuple[float, float, float]] = field(
        default_factory=dict
    )
    updated_coils: set[str] = field(default_factory=set)
    last_frame_monotonic: float | None = None
    last_estimate_time: float | None = None
    stream_stale: bool = False
    has_published_frame: bool = False


class StateEstimationNode(Node):
    """Run an independently selected backend for each configured device."""

    def __init__(self) -> None:
        super().__init__("state_estimator")
        default_config = os.environ.get(
            "STATE_ESTIMATION_CONFIG",
            str(
                Path.home() / "state_estimation" / "configs"
                / "live_coil_estimation.yaml"
            ),
        )
        self.declare_parameter("config_path", default_config)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self._noise_config_path = str(
            self.get_parameter("config_path").value
        )
        self._noise = None
        self._stale_timeout = float(
            self.get_parameter("stale_timeout_sec").value
        )
        self._config: ShapeConfig | None = None
        self._runtimes: dict[str, DeviceRuntime] = {}
        self._transform_to_device: dict[str, str] = {}
        self._last_errors: dict[str, tuple[str, float]] = {}

        latest_frame_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        transform_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/IGTL_STRING_IN", self._config_cb, 10
        )
        self.create_subscription(
            Transform,
            "/IGTL_TRANSFORM_IN",
            self._coil_transform_cb,
            transform_qos,
        )
        self._shape_pub = self.create_publisher(
            PointArray, "/IGTL_POINT_OUT", latest_frame_qos
        )
        self._status_pub = self.create_publisher(
            String, "/IGTL_STRING_OUT", 10
        )
        self.create_timer(0.1, self._check_stale)
        self.get_logger().info(
            "Multi-device shape estimator ready; waiting for named transforms"
        )

    def _config_cb(self, msg: String) -> None:
        if msg.name != CONFIG_DEVICE:
            return
        try:
            config = parse_shape_config(msg.data)
            runtimes = {}
            transform_to_device = {}
            for device in config.devices:
                estimator = None
                if config.enabled and device.coils and not device.rigid:
                    estimator = self._make_continuum_estimator(device)
                runtimes[device.device_id] = DeviceRuntime(
                    config=device,
                    estimator=estimator,
                )
                for transform_name in device.transform_names:
                    transform_to_device[transform_name] = device.device_id
        except (
            ShapeConfigError, ValueError, RuntimeError, ImportError
        ) as exc:
            self._publish_status("error", str(exc))
            return

        self._config = config
        self._runtimes = runtimes
        self._transform_to_device = transform_to_device
        self._last_errors.clear()
        self._publish_status_payload(config.status_payload())
        action = "Enabled" if config.enabled else "Disabled"
        summary_parts = []
        for device in config.devices:
            if not device.coils:
                backend = "inactive"
            else:
                backend = "rigid_line" if device.rigid else "continuum"
            summary_parts.append(
                f"{device.device_id}={backend}({len(device.coils)} coils)"
            )
        summary = ", ".join(summary_parts)
        self.get_logger().info(f"{action} shape estimation: {summary}")

    def _make_continuum_estimator(self, device: DeviceConfig):
        # Rigid-only configurations do not import or initialize GTSAM.
        from cr_common.configs import NoiseConfig, RodConfig
        from state_estimation import QuasiStaticKinematicsEstimator

        if self._noise is None:
            self._noise = NoiseConfig.from_yaml(self._noise_config_path)
        rod = RodConfig(
            length=device.length_m,
            n_sections=device.n_sections,
            proximal_node_idx=device.proximal_node_idx,
        )
        return QuasiStaticKinematicsEstimator(
            rod,
            self._noise,
            solver="lm",
            known_base_pose=False,
            compute_marginals=False,
            max_iterations=20,
        )

    def _coil_transform_cb(self, msg: Transform) -> None:
        device_id = self._transform_to_device.get(msg.name)
        if device_id is None:
            return
        if self._config is None or not self._config.enabled:
            return
        runtime = self._runtimes[device_id]
        translation = msg.transform.translation
        runtime.coil_positions_mm[msg.name] = (
            translation.x, translation.y, translation.z
        )
        runtime.updated_coils.add(msg.name)
        expected = set(runtime.config.transform_names)
        if not expected.issubset(runtime.updated_coils):
            return
        points_mm = [
            runtime.coil_positions_mm[name]
            for name in runtime.config.transform_names
        ]
        runtime.updated_coils.difference_update(expected)
        self._process_device_frame(runtime, points_mm)

    def _process_device_frame(
        self, runtime: DeviceRuntime, points_mm
    ) -> None:
        now_monotonic = time.monotonic()
        now_ros = self.get_clock().now()
        timestamp = now_ros.nanoseconds * 1.0e-9
        dt = 0.05 if runtime.last_estimate_time is None else max(
            1.0e-4, min(1.0, timestamp - runtime.last_estimate_time)
        )
        try:
            if runtime.config.rigid:
                output_points_mm = fit_straight_segment_mm(
                    points_mm,
                    [
                        value * 1000.0
                        for value in runtime.config.coil_locations_m
                    ],
                    runtime.config.length_m * 1000.0,
                    runtime.config.n_sections,
                )
            else:
                output_points_mm = self._estimate_continuum(
                    runtime, points_mm, timestamp, dt
                )
        except Exception as exc:
            self._publish_error_throttled(
                runtime.config.device_id, f"estimation failed: {exc}"
            )
            return

        shape = PointArray()
        shape.name = runtime.config.output_device_name
        shape.pointdata = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in output_points_mm
        ]
        self._shape_pub.publish(shape)
        runtime.last_frame_monotonic = now_monotonic
        runtime.last_estimate_time = timestamp
        if (
            not runtime.has_published_frame
            or runtime.stream_stale
            or runtime.config.device_id in self._last_errors
        ):
            runtime.stream_stale = False
            self._last_errors.pop(runtime.config.device_id, None)
            self._publish_status(
                "streaming",
                "valid coil transforms received",
                runtime.config.device_id,
            )
        runtime.has_published_frame = True

    def _estimate_continuum(
        self,
        runtime: DeviceRuntime,
        points_mm,
        timestamp: float,
        dt: float,
    ):
        from cr_common.configs import MeasurementPacket

        positions = coil_points_mm_to_positions(
            points_mm, runtime.config.local_coil_indices
        )
        packet = MeasurementPacket(
            timestamp=timestamp,
            dt=dt,
            base_pose=None,
            positions=positions,
        )
        state = runtime.estimator.update(packet)
        return [
            [
                float(node.position[0] * 1000.0),
                float(node.position[1] * 1000.0),
                float(node.position[2] * 1000.0),
            ]
            for node in state.nodes
        ]

    def _check_stale(self) -> None:
        now = time.monotonic()
        for runtime in self._runtimes.values():
            if runtime.last_frame_monotonic is None or runtime.stream_stale:
                continue
            if stream_is_stale(
                runtime.last_frame_monotonic, now, self._stale_timeout
            ):
                runtime.stream_stale = True
                self._publish_status(
                    "stale",
                    "no fresh complete coil frame",
                    runtime.config.device_id,
                )

    def _publish_error_throttled(
        self, device_id: str, message: str
    ) -> None:
        now = time.monotonic()
        previous = self._last_errors.get(device_id)
        if (
            previous
            and previous[0] == message
            and now - previous[1] < 1.0
        ):
            return
        self._last_errors[device_id] = (message, now)
        self._publish_status("error", message, device_id)

    def _publish_status(
        self, state: str, message: str, device_id: str | None = None
    ) -> None:
        payload = {
            "schema_version": 2,
            "state": state,
            "message": message,
        }
        if device_id is not None:
            payload["device_id"] = device_id
        self._publish_status_payload(payload)

    def _publish_status_payload(self, payload: dict) -> None:
        msg = String()
        msg.name = STATUS_DEVICE
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateEstimationNode()
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
