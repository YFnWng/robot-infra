"""ROS 2 coordinator for live position-only catheter shape estimation."""
from __future__ import annotations

import json
import os
from pathlib import Path
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_igtl_bridge.msg import PointArray, String

from cr_common.configs import MeasurementPacket, NoiseConfig, RodConfig
from state_estimation import QuasiStaticKinematicsEstimator

from .protocol import (
    ShapeConfig,
    ShapeConfigError,
    coil_points_mm_to_positions,
    parse_shape_config,
    stream_is_stale,
)


CONFIG_DEVICE = "estimator/config"
STATUS_DEVICE = "estimator/status"
SHAPE_DEVICE = "estimator/shape"


class StateEstimationNode(Node):
    """Join Slicer configuration and tracker POINT frames in ROS 2."""

    def __init__(self) -> None:
        super().__init__("state_estimator")
        default_config = os.environ.get(
            "STATE_ESTIMATION_CONFIG",
            str(Path.home() / "state_estimation" / "configs" / "live_coil_estimation.yaml"),
        )
        self.declare_parameter("config_path", default_config)
        self.declare_parameter("stale_timeout_sec", 0.5)
        config_path = self.get_parameter("config_path").value
        self._noise = NoiseConfig.from_yaml(config_path)
        self._stale_timeout = float(self.get_parameter("stale_timeout_sec").value)

        self._config: ShapeConfig | None = None
        self._estimator: QuasiStaticKinematicsEstimator | None = None
        self._last_frame_monotonic: float | None = None
        self._last_estimate_time: float | None = None
        self._stream_stale = False
        self._has_published_frame = False
        self._last_error: tuple[str, float] | None = None

        latest_frame_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(String, "/IGTL_STRING_IN", self._config_cb, 10)
        self.create_subscription(
            PointArray,
            "/tracking/igtl/point_in",
            self._coil_cb,
            latest_frame_qos,
        )
        self._shape_pub = self.create_publisher(
            PointArray, "/IGTL_POINT_OUT", latest_frame_qos
        )
        self._status_pub = self.create_publisher(String, "/IGTL_STRING_OUT", 10)
        self.create_timer(0.1, self._check_stale)
        self.get_logger().info(f"Live shape estimator ready; noise config: {config_path}")

    def _config_cb(self, msg: String) -> None:
        if msg.name != CONFIG_DEVICE:
            return
        try:
            config = parse_shape_config(msg.data)
            rod = RodConfig(
                length=config.rod_length_m,
                n_sections=config.n_sections,
                proximal_node_idx=config.proximal_node_idx,
            )
            estimator = QuasiStaticKinematicsEstimator(
                rod,
                self._noise,
                solver="lm",
                known_base_pose=False,
                compute_marginals=False,
                max_iterations=20,
            )
        except (ShapeConfigError, ValueError, RuntimeError) as exc:
            self._publish_status("error", str(exc))
            return

        self._config = config
        self._estimator = estimator
        self._last_frame_monotonic = None
        self._last_estimate_time = None
        self._stream_stale = False
        self._has_published_frame = False
        self._last_error = None
        self._publish_status_payload(config.status_payload())
        self.get_logger().info(
            f"Configured {len(config.coil_node_indices)} coils on nodes "
            f"{list(config.coil_node_indices)}"
        )

    def _coil_cb(self, msg: PointArray) -> None:
        config = self._config
        estimator = self._estimator
        if config is None or estimator is None:
            self._publish_error_throttled("coil frame received before configuration")
            return
        now_monotonic = time.monotonic()
        now_ros = self.get_clock().now()
        timestamp = now_ros.nanoseconds * 1.0e-9
        dt = 0.05 if self._last_estimate_time is None else max(
            1.0e-4, min(1.0, timestamp - self._last_estimate_time)
        )
        try:
            positions = coil_points_mm_to_positions(
                [(point.x, point.y, point.z) for point in msg.pointdata],
                config.local_coil_indices,
            )
        except ShapeConfigError as exc:
            self._publish_error_throttled(str(exc))
            return
        packet = MeasurementPacket(
            timestamp=timestamp,
            dt=dt,
            base_pose=None,
            positions=positions,
        )
        try:
            state = estimator.update(packet)
        except Exception as exc:  # optimizer errors must not terminate the ROS node
            self._publish_error_throttled(f"estimation failed: {exc}")
            return

        shape = PointArray()
        shape.name = SHAPE_DEVICE
        shape.pointdata = [
            Point(
                x=float(node.position[0] * 1000.0),
                y=float(node.position[1] * 1000.0),
                z=float(node.position[2] * 1000.0),
            )
            for node in state.nodes
        ]
        self._shape_pub.publish(shape)
        self._last_frame_monotonic = now_monotonic
        self._last_estimate_time = timestamp
        if (
            not self._has_published_frame
            or self._stream_stale
            or self._last_error is not None
        ):
            self._stream_stale = False
            self._last_error = None
            self._publish_status("streaming", "valid coil frames received")
        self._has_published_frame = True

    def _check_stale(self) -> None:
        if self._last_frame_monotonic is None or self._stream_stale:
            return
        if stream_is_stale(
            self._last_frame_monotonic, time.monotonic(), self._stale_timeout
        ):
            self._stream_stale = True
            self._publish_status("stale", "no fresh coil frame")

    def _publish_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if self._last_error and self._last_error[0] == message and now - self._last_error[1] < 1.0:
            return
        self._last_error = (message, now)
        self._publish_status("error", message)

    def _publish_status(self, state: str, message: str) -> None:
        self._publish_status_payload({
            "schema_version": 1,
            "state": state,
            "message": message,
        })

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
