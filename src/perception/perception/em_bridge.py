"""EM tracker bridge: Slicer coil PointArray -> canonical /em_tracker/poses.

Slicer's ``AuroraTracker`` streams the EM coil positions as a single, batched
``ros2_igtl_bridge/PointArray`` (all coils in one message, positions in mm) over
OpenIGTLink. Keeping every coil in one message preserves their common sample
time, which is exactly what downstream fusion wants.

This node republishes that batch as the canonical ``geometry_msgs/PoseArray`` on
``/em_tracker/poses`` (positions in metres, identity orientation) that both the
estimator's ``EMCoilROSAdapter`` and the data-collection recorder consume.

Notes
-----
* ``ros2_igtl_bridge/PointArray`` has no header, so poses are stamped on arrival
  (adds serial + OpenIGTLink transport latency; acceptable for offline
  interpolation, and the reason we prefer a batched message over per-coil ones).
* Orientation is identity: the batched point stream carries positions only. When
  per-coil 6-DOF is available (e.g. a tip sensor sent as ``Transform``), merge it
  here — the ``/em_tracker/poses`` contract already carries orientation.
* Multiple POINT sources can share one bridge topic (e.g. the robot GUI also
  sends points), so filter by IGTL device ``name``. The observed names are logged
  once to help configure ``device_name``.
"""
from __future__ import annotations

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_igtl_bridge.msg import PointArray

_MM_TO_M = 0.001


class EMBridge(Node):
    """Republish batched EM coil points as a canonical PoseArray."""

    def __init__(self) -> None:
        super().__init__("em_bridge")

        self.declare_parameter("input_topic", "/IGTL_POINT_IN")
        self.declare_parameter("output_topic", "/em_tracker/poses")
        # IGTL device name of the Aurora coil message; "" accepts any (and logs
        # the names seen so it can be pinned down).
        self.declare_parameter("device_name", "")
        self.declare_parameter("input_units", "mm")        # "mm" | "m"
        self.declare_parameter("num_coils", 0)             # 0 = accept any count
        self.declare_parameter("frame_id", "aurora")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._device_name = self.get_parameter("device_name").value
        units = self.get_parameter("input_units").value
        self._scale = _MM_TO_M if units == "mm" else 1.0
        self._num_coils = int(self.get_parameter("num_coils").value)
        self._frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._sub = self.create_subscription(
            PointArray, input_topic, self._cb, qos)
        self._pub = self.create_publisher(PoseArray, output_topic, qos)

        self._seen_names: set[str] = set()
        self._warned_count = False
        self.get_logger().info(
            f"em_bridge: {input_topic} (PointArray) -> {output_topic} "
            f"(PoseArray, {units}->m, frame '{self._frame_id}', "
            f"device_name='{self._device_name or '*'}')"
        )

    def _cb(self, msg: PointArray) -> None:
        if msg.name not in self._seen_names:
            self._seen_names.add(msg.name)
            self.get_logger().info(f"em_bridge: receiving POINT device '{msg.name}'")

        if self._device_name and msg.name != self._device_name:
            return

        if self._num_coils and len(msg.pointdata) != self._num_coils:
            if not self._warned_count:
                self._warned_count = True
                self.get_logger().warn(
                    f"em_bridge: expected {self._num_coils} coils, got "
                    f"{len(msg.pointdata)} on device '{msg.name}' (suppressing repeats)"
                )
            return

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        for p in msg.pointdata:
            pose = Pose()
            pose.position.x = p.x * self._scale
            pose.position.y = p.y * self._scale
            pose.position.z = p.z * self._scale
            pose.orientation.w = 1.0  # identity; positions-only source
            out.poses.append(pose)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EMBridge()
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
