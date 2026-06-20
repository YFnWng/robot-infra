"""StateEstimationNode: ROS2 wrapper around QuasiStaticKinematicsEstimator.

Imports the state_estimation library (pure Python, no ROS2) and wraps it
in a ROS2 node that subscribes to sensor topics and publishes estimated poses.

Data flow:
    /device/state (DeviceStream) → joint positions → base pose + cable disp
    /em_tracker/poses (TBD)      → EM coil poses
    → QuasiStaticKinematicsEstimator.update()
    → /estimator/tip_pose  (PoseStamped)
    → /estimator/rod_state (PoseArray)
"""
from __future__ import annotations

import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

import numpy as np

# Allow state_estimation library to be found even if not installed via pip.
# Set STATE_ESTIMATION_PATH env var or rely on PYTHONPATH.
_se_path = os.environ.get(
    "STATE_ESTIMATION_PATH",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "state_estimation"),
)
if _se_path not in sys.path:
    sys.path.insert(0, os.path.dirname(_se_path))

from state_estimation import QuasiStaticKinematicsEstimator  # noqa: E402
from cr_common.configs import RodConfig, NoiseConfig, MeasurementPacket

from control_interface.msg import DeviceStream  # noqa: E402


class StateEstimationNode(Node):
    """ROS2 node wrapping QuasiStaticKinematicsEstimator.

    Loads rod and noise config from the path specified by the
    ``config_path`` ROS parameter (defaults to
    ``state_estimation/configs/catheter_ablation.yaml``).

    Topics
    ------
    Subscriptions:
        /device/state (DeviceStream) — joint positions [lin, rot, bend, ...]
    Publications:
        /estimator/tip_pose  (PoseStamped)
        /estimator/rod_state (PoseArray)   — one Pose per estimation node
    """

    def __init__(self) -> None:
        super().__init__("state_estimator")

        self.declare_parameter(
            "config_path",
            os.path.join(_se_path, "configs", "catheter_ablation.yaml"),
        )
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        noise = NoiseConfig.from_yaml(config_path)
        rod = RodConfig.from_yaml(config_path) if hasattr(RodConfig, "from_yaml") \
              else RodConfig(length=160.0, n_sections=32)

        self._estimator = QuasiStaticKinematicsEstimator(rod, noise, solver="lm")
        self._base_pose = np.array([0.0, 0.0, -160.0, 0.0, 0.0, 0.0, 1.0])
        self._cable_disp = 0.0

        # Subscriptions
        self.create_subscription(
            DeviceStream,
            "/device/state",
            self._device_state_cb,
            10,
        )

        # Publishers
        self._tip_pub = self.create_publisher(PoseStamped, "/estimator/tip_pose", 10)
        self._rod_pub = self.create_publisher(PoseArray, "/estimator/rod_state", 10)

        # Run estimator at device update rate via timer (10 ms = 100 Hz)
        self.create_timer(0.01, self._estimate)

        self.get_logger().info("StateEstimationNode ready.")

    def _device_state_cb(self, msg: DeviceStream) -> None:
        """Cache latest joint state for use in next estimation step."""
        if msg.predicate == DeviceStream.POS and len(msg.data) >= 3:
            # Joint layout: [catheter_lin, catheter_rot, catheter_bend, ...]
            self._cable_disp = float(msg.data[2])  # bend ≈ cable displacement

    def _estimate(self) -> None:
        """Build a MeasurementPacket from cached state and run estimation."""
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        packet = MeasurementPacket(
            timestamp=t,
            dt=0.01,
            base_pose=self._base_pose.copy(),
            cable_disp=self._cable_disp,
            # Sensor observations filled in by ROS sensor adapters (TBD)
            positions={},
            poses={},
            strains={},
        )

        state = self._estimator.update(packet)
        self._publish(state, now)

    def _publish(self, state, stamp) -> None:
        header_kwargs = {"frame_id": "world"}

        # Tip pose
        tip = state.tip
        tip_msg = PoseStamped()
        tip_msg.header.stamp = stamp.to_msg()
        tip_msg.header.frame_id = "world"
        _fill_pose(tip_msg.pose, tip.pose)
        self._tip_pub.publish(tip_msg)

        # Full rod state
        rod_msg = PoseArray()
        rod_msg.header.stamp = stamp.to_msg()
        rod_msg.header.frame_id = "world"
        for node in state.nodes:
            p = Pose()
            _fill_pose(p, node.pose)
            rod_msg.poses.append(p)
        self._rod_pub.publish(rod_msg)


def _fill_pose(pose_msg: Pose, vec7: np.ndarray) -> None:
    pose_msg.position.x = float(vec7[0])
    pose_msg.position.y = float(vec7[1])
    pose_msg.position.z = float(vec7[2])
    pose_msg.orientation.x = float(vec7[3])
    pose_msg.orientation.y = float(vec7[4])
    pose_msg.orientation.z = float(vec7[5])
    pose_msg.orientation.w = float(vec7[6])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateEstimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
