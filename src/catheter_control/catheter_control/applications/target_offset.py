"""Publish a guarded hardware target relative to the measured catheter tip."""
from __future__ import annotations

import argparse
import math
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud


CONTROLLER_STATUS_NAME = "catheter_control/mppi"


def _stamp_ns(message) -> int:
    return (int(message.header.stamp.sec)*1_000_000_000
            + int(message.header.stamp.nanosec))


def measured_tip(message: PointCloud, expected_frame: str):
    """Return marker ID 3 from a valid registered four-marker point cloud."""
    if message.header.frame_id != expected_frame:
        raise ValueError(
            f"marker frame is {message.header.frame_id!r}, expected "
            f"{expected_frame!r}")
    marker_ids = None
    for channel in message.channels:
        if channel.name == "marker_id":
            marker_ids = list(channel.values)
            break
    if marker_ids is None or len(marker_ids) != len(message.points):
        raise ValueError("marker point cloud lacks a matching marker_id channel")
    matches = [index for index, value in enumerate(marker_ids)
               if math.isfinite(value) and int(round(value)) == 3]
    if len(matches) != 1:
        raise ValueError("marker point cloud must contain marker ID 3 exactly once")
    point = message.points[matches[0]]
    xyz = (float(point.x), float(point.y), float(point.z))
    if not all(math.isfinite(value) for value in xyz):
        raise ValueError("measured tip contains a non-finite coordinate")
    return xyz


def controller_status(message: DiagnosticArray):
    """Extract ``(armed, state, reason)`` from controller diagnostics."""
    for status in message.status:
        if status.name != CONTROLLER_STATUS_NAME:
            continue
        values = {item.key: item.value for item in status.values}
        armed_text = values.get("armed")
        if armed_text not in ("True", "False", "true", "false"):
            raise ValueError("controller status lacks a valid armed field")
        return (armed_text.lower() == "true", str(status.message),
                str(values.get("reason", "none")))
    raise ValueError(f"status does not contain {CONTROLLER_STATUS_NAME}")


def controller_state(message: DiagnosticArray):
    """Compatibility wrapper returning only ``(armed, state)``."""
    armed, state, _ = controller_status(message)
    return armed, state


class TargetOffsetPublisher(Node):
    def __init__(self, frame_id: str):
        super().__init__("catheter_target_offset")
        self.frame_id = frame_id
        self.tip = None
        self.tip_stamp_ns = None
        self.status = None
        self.status_stamp_ns = None
        self.create_subscription(
            PointCloud, "/shape_tracking/markers", self._marker_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/catheter_mppi/status", self._status_cb, 10)
        self.publisher = self.create_publisher(
            PointStamped, "/catheter_mppi/target_tip", 10)

    def _marker_cb(self, message):
        try:
            tip = measured_tip(message, self.frame_id)
        except ValueError as error:
            self.get_logger().warn(f"ignored marker update: {error}")
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            self.get_logger().warn("ignored marker update with invalid stamp")
            return
        if self.tip_stamp_ns is None or stamp_ns >= self.tip_stamp_ns:
            self.tip = tip
            self.tip_stamp_ns = stamp_ns

    def _status_cb(self, message):
        try:
            status = controller_state(message)
        except ValueError:
            return
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            return
        if self.status_stamp_ns is None or stamp_ns >= self.status_stamp_ns:
            self.status = status
            self.status_stamp_ns = stamp_ns


def _arguments(argv):
    parser = argparse.ArgumentParser(description=(
        "Publish a hardware MPPI target relative to the latest measured tip"))
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=0.0)
    parser.add_argument("--frame-id", default="robot_base")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--maximum-offset-mm", type=float, default=10.0)
    parser.add_argument("--maximum-feedback-age-s", type=float, default=0.25)
    parser.add_argument(
        "--allow-armed-retarget", action="store_true",
        help="explicitly allow changing the target while MPPI is armed")
    result = parser.parse_args(remove_ros_args(args=argv)[1:])
    offset = (result.dx_mm, result.dy_mm, result.dz_mm)
    if not all(math.isfinite(value) for value in offset):
        parser.error("target offsets must be finite")
    norm = math.sqrt(sum(value*value for value in offset))
    if norm <= 0.0:
        parser.error("at least one target offset must be nonzero")
    if (not math.isfinite(result.maximum_offset_mm)
            or result.maximum_offset_mm <= 0.0):
        parser.error("--maximum-offset-mm must be finite and positive")
    if norm > result.maximum_offset_mm:
        parser.error(
            f"offset norm {norm:.3f} mm exceeds --maximum-offset-mm "
            f"{result.maximum_offset_mm:.3f} mm")
    if (result.timeout_s <= 0.0 or result.maximum_feedback_age_s <= 0.0
            or not result.frame_id):
        parser.error("timeouts and frame ID must be positive/nonempty")
    return result


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    rclpy.init(args=argv)
    node = TargetOffsetPublisher(parsed.frame_id)
    try:
        deadline = time.monotonic()+parsed.timeout_s
        while ((node.tip is None or node.status is None
                or node.publisher.get_subscription_count() < 1)
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.tip is None:
            raise RuntimeError("timed out waiting for measured marker tip")
        if node.status is None:
            raise RuntimeError("timed out waiting for catheter MPPI status")
        if node.publisher.get_subscription_count() < 1:
            raise RuntimeError("catheter MPPI target subscriber was not discovered")

        now_ns = node.get_clock().now().nanoseconds
        tip_age_s = (now_ns-node.tip_stamp_ns)*1e-9
        status_age_s = (now_ns-node.status_stamp_ns)*1e-9
        if tip_age_s < -0.01 or tip_age_s > parsed.maximum_feedback_age_s:
            raise RuntimeError(
                f"measured tip is not fresh: age={tip_age_s:.3f}s")
        if status_age_s < -0.01 or status_age_s > parsed.maximum_feedback_age_s:
            raise RuntimeError(
                f"controller status is not fresh: age={status_age_s:.3f}s")

        armed, state = node.status
        if armed and not parsed.allow_armed_retarget:
            raise RuntimeError(
                "controller is armed; disarm first or explicitly pass "
                "--allow-armed-retarget")
        target = PointStamped()
        target.header.stamp = node.get_clock().now().to_msg()
        target.header.frame_id = parsed.frame_id
        target.point.x = node.tip[0]+parsed.dx_mm*1e-3
        target.point.y = node.tip[1]+parsed.dy_mm*1e-3
        target.point.z = node.tip[2]+parsed.dz_mm*1e-3
        for _ in range(3):
            node.publisher.publish(target)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info(
            "published target from measured tip [%.3f, %.3f, %.3f] mm; "
            "offset [%.3f, %.3f, %.3f] mm; target "
            "[%.3f, %.3f, %.3f] mm; controller=%s armed=%s" % (
                *(1e3*value for value in node.tip),
                parsed.dx_mm, parsed.dy_mm, parsed.dz_mm,
                1e3*target.point.x, 1e3*target.point.y,
                1e3*target.point.z, state, armed))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
