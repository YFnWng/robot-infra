"""Publish one target offset from the current simulated tip."""
from __future__ import annotations

import argparse
import sys
import time

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class TargetOffsetPublisher(Node):
    def __init__(self):
        super().__init__("catheter_sim_target")
        self.tip = None
        self.create_subscription(
            PointStamped, "/sim/catheter_sim/ground_truth_tip",
            self._tip_cb, 10)
        self.publisher = self.create_publisher(
            PointStamped, "/sim/catheter_mppi/target_tip", 10)

    def _tip_cb(self, message):
        self.tip = message


def _arguments(argv):
    parser = argparse.ArgumentParser(
        description="Set a simulated tip target relative to current truth")
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--allow-combined", action="store_true",
        help="allow more than one nonzero Cartesian component")
    result = parser.parse_args(remove_ros_args(args=argv)[1:])
    nonzero = sum(value != 0.0 for value in (
        result.dx_mm, result.dy_mm, result.dz_mm))
    if nonzero > 1 and not result.allow_combined:
        parser.error("use one axis at a time or pass --allow-combined")
    if result.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    return result


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    rclpy.init(args=argv)
    node = TargetOffsetPublisher()
    try:
        deadline = time.monotonic() + parsed.timeout_s
        while node.tip is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.tip is None:
            raise RuntimeError("timed out waiting for simulated ground-truth tip")
        target = PointStamped()
        target.header.stamp = node.get_clock().now().to_msg()
        target.header.frame_id = node.tip.header.frame_id
        target.point.x = node.tip.point.x + parsed.dx_mm * 1e-3
        target.point.y = node.tip.point.y + parsed.dy_mm * 1e-3
        target.point.z = node.tip.point.z + parsed.dz_mm * 1e-3
        # Repeat briefly so a concurrently starting controller discovers the
        # volatile publisher without making the target itself latched.
        for _ in range(3):
            node.publisher.publish(target)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info(
            "published target offset [%.3f, %.3f, %.3f] mm at "
            "[%.6f, %.6f, %.6f] m" % (
                parsed.dx_mm, parsed.dy_mm, parsed.dz_mm,
                target.point.x, target.point.y, target.point.z))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
