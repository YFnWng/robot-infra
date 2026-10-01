"""Load a tip trajectory from YAML and submit it to the ROS action server."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

from control_interface.action import TrackTipTrajectory
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud
import yaml

from .target_offset import measured_tip


@dataclass(frozen=True)
class TrajectoryFileSpec:
    frame_id: str
    action_name: str
    marker_topic: str
    waypoints_m: np.ndarray
    waypoint_timeouts_s: np.ndarray
    tolerance_mm: float
    settle_time_s: float
    auto_arm: bool


def _positive_float(mapping, key):
    value = float(mapping[key])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{key} must be finite and positive")
    return value


def _circle_waypoints(generator, current_tip_m):
    reference_tip_m = generator.get("reference_tip_m", current_tip_m)
    if reference_tip_m is None:
        raise ValueError("circle generator requires the current measured tip")
    tip = np.asarray(reference_tip_m, dtype=np.float64)
    if tip.shape != (3,) or not np.all(np.isfinite(tip)):
        raise ValueError(
            "current measured tip must contain three finite values")
    generator_type = generator.get("type")
    if generator_type not in (
            "circle_xy_from_current_tip", "circle_yz_from_current_tip"):
        raise ValueError(f"unsupported generator type: {generator_type}")
    count = int(generator.get("waypoint_count", 24))
    if count < 3 or count > 100:
        raise ValueError("waypoint_count must be in [3, 100]")
    direction = str(generator.get("direction", "ccw")).lower()
    if direction not in ("ccw", "cw"):
        raise ValueError("direction must be 'ccw' or 'cw'")
    sign = 1.0 if direction == "ccw" else -1.0
    close_circle = bool(generator.get("close_circle", True))
    if generator_type == "circle_xy_from_current_tip":
        offset_mm = float(generator.get("radius_offset_mm", 3.0))
        if not math.isfinite(offset_mm):
            raise ValueError("radius_offset_mm must be finite")
        radius_m = float(np.hypot(tip[0], tip[1]))+offset_mm*1e-3
        if radius_m <= 0.0:
            raise ValueError("generated circle radius must be positive")
        start_angle = math.atan2(float(tip[1]), float(tip[0]))
        angles = start_angle+sign*np.linspace(
            0.0, 2.0*math.pi, count, endpoint=close_circle)
        waypoints = np.column_stack((
            radius_m*np.cos(angles),
            radius_m*np.sin(angles),
            np.full(count, tip[2], dtype=np.float64)))
    else:
        radius_m = 1e-3*float(generator.get("radius_mm", 10.0))
        x_offset_m = 1e-3*float(
            generator.get("center_x_offset_mm", 3.0))
        if (not math.isfinite(radius_m) or radius_m <= 0.0
                or not math.isfinite(x_offset_m)):
            raise ValueError(
                "radius_mm must be positive and center_x_offset_mm finite")
        center_x = tip[0]+x_offset_m
        center_z = tip[2]
        start_angle = math.atan2(0.0, float(tip[1]))
        angles = start_angle+sign*np.linspace(
            0.0, 2.0*math.pi, count, endpoint=close_circle)
        waypoints = np.column_stack((
            np.full(count, center_x, dtype=np.float64),
            radius_m*np.cos(angles),
            center_z+radius_m*np.sin(angles)))
    approach_count = int(generator.get("approach_waypoint_count", 0))
    if approach_count < 0 or approach_count > 100-len(waypoints):
        raise ValueError(
            "approach_waypoint_count must be nonnegative and keep the total "
            "waypoint count at or below 100")
    if approach_count:
        fractions = (np.arange(1, approach_count+1, dtype=np.float64)
                     / (approach_count+1))
        approach = tip[None, :]+fractions[:, None]*(waypoints[0]-tip)[None, :]
        waypoints = np.concatenate((approach, waypoints), axis=0)
    return waypoints


def load_trajectory_file(path, current_tip_m=None, *, action_override=None,
                         marker_override=None):
    """Parse and expand one trajectory YAML file into an action goal spec."""
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("trajectory YAML root must be a mapping")
    frame_id = str(document.get("frame_id", "robot_base"))
    action_name = str(action_override or document.get(
        "action_name", "/catheter_mppi/track_tip_trajectory"))
    marker_topic = str(marker_override or document.get(
        "marker_topic", "/shape_tracking/markers"))
    if not frame_id or not action_name or not marker_topic:
        raise ValueError(
            "frame_id, action_name, and marker_topic are required")

    generator = document.get("generator")
    explicit = document.get("waypoints_m")
    if (generator is None) == (explicit is None):
        raise ValueError("provide exactly one of generator or waypoints_m")
    if generator is not None:
        if not isinstance(generator, dict):
            raise ValueError("generator must be a mapping")
        waypoints = _circle_waypoints(generator, current_tip_m)
    else:
        waypoints = np.asarray(explicit, dtype=np.float64)
    if (waypoints.ndim != 2 or waypoints.shape[1:] != (3,)
            or len(waypoints) < 1 or len(waypoints) > 100
            or not np.all(np.isfinite(waypoints))):
        raise ValueError(
            "waypoints_m must be a finite Nx3 array, N in [1,100]")

    timeout_value = document.get("waypoint_timeouts_s")
    if timeout_value is None and generator is not None:
        timeout_value = generator.get("waypoint_timeout_s")
    if isinstance(timeout_value, (int, float)):
        timeouts = np.full(len(waypoints), float(timeout_value))
    else:
        timeouts = np.asarray(timeout_value, dtype=np.float64)
    if (timeouts.shape != (len(waypoints),)
            or not np.all(np.isfinite(timeouts)) or np.any(timeouts <= 0.0)):
        raise ValueError(
            "waypoint timeouts must be positive and match the waypoint count")

    tolerance_mm = _positive_float(document, "tolerance_mm")
    settle_time_s = float(document.get("settle_time_s", 0.0))
    if not math.isfinite(settle_time_s) or settle_time_s < 0.0:
        raise ValueError("settle_time_s must be finite and nonnegative")
    auto_arm = document.get("auto_arm", False)
    if not isinstance(auto_arm, bool):
        raise ValueError("auto_arm must be a YAML boolean")
    return TrajectoryFileSpec(
        frame_id=frame_id, action_name=action_name,
        marker_topic=marker_topic, waypoints_m=waypoints,
        waypoint_timeouts_s=timeouts, tolerance_mm=tolerance_mm,
        settle_time_s=settle_time_s, auto_arm=auto_arm)


class TrajectoryFileClient(Node):
    def __init__(self, frame_id, marker_topic, action_name):
        super().__init__("catheter_tip_trajectory_file")
        self.frame_id = frame_id
        self.marker_topic = marker_topic
        self.tip = None
        self.tip_received_at = None
        self.marker_rejection = None
        self.last_feedback_index = None
        self.marker_subscription = self.create_subscription(
            PointCloud, marker_topic, self._marker_cb, qos_profile_sensor_data)
        self.client = ActionClient(self, TrackTipTrajectory, action_name)

    def _marker_cb(self, message):
        try:
            tip = measured_tip(message, self.frame_id)
        except ValueError as error:
            self.marker_rejection = str(error)
            return
        self.tip = np.asarray(tip, dtype=np.float64)
        self.tip_received_at = time.monotonic()
        self.marker_rejection = None

    def feedback_cb(self, message):
        feedback = message.feedback
        if feedback.waypoint_index != self.last_feedback_index:
            self.last_feedback_index = feedback.waypoint_index
            self.get_logger().info(
                "waypoint %d/%d active; error=%.3f mm, budget=%.3f s" % (
                    feedback.waypoint_index+1, feedback.waypoint_count,
                    feedback.tracking_error_mm, feedback.remaining_s))


def _arguments(argv):
    parser = argparse.ArgumentParser(description=(
        "Load a catheter tip trajectory YAML file and send its action goal"))
    parser.add_argument("yaml_file")
    parser.add_argument("--action-name")
    parser.add_argument("--marker-topic")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    result = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not math.isfinite(result.timeout_s) or result.timeout_s <= 0.0:
        parser.error("--timeout-s must be finite and positive")
    return result


def _goal_from_spec(node, spec):
    goal = TrackTipTrajectory.Goal()
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.header.frame_id = spec.frame_id
    for xyz in spec.waypoints_m:
        point = Point()
        point.x, point.y, point.z = xyz.tolist()
        goal.waypoints.append(point)
    goal.waypoint_timeouts_s = spec.waypoint_timeouts_s.tolist()
    goal.tolerance_mm = spec.tolerance_mm
    goal.settle_time_s = spec.settle_time_s
    goal.auto_arm = spec.auto_arm
    return goal


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    # Read endpoint/frame metadata before creating the final client. Circle
    # expansion is repeated after fresh tip feedback is available.
    with Path(parsed.yaml_file).expanduser().open(
            "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError("trajectory YAML root must be a mapping")
    frame_id = str(metadata.get("frame_id", "robot_base"))
    action_name = str(parsed.action_name or metadata.get(
        "action_name", "/catheter_mppi/track_tip_trajectory"))
    marker_topic = str(parsed.marker_topic or metadata.get(
        "marker_topic", "/shape_tracking/markers"))

    rclpy.init(args=argv)
    node = TrajectoryFileClient(frame_id, marker_topic, action_name)
    goal_handle = None
    try:
        deadline = time.monotonic()+parsed.timeout_s
        while node.tip is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.tip is None:
            publishers = node.count_publishers(node.marker_topic)
            if publishers == 0:
                detail = (
                    f"no publisher discovered on {node.marker_topic}; "
                    "check ROS_DOMAIN_ID and that simulation is running")
            elif node.marker_rejection:
                detail = (
                    f"marker samples on {node.marker_topic} were rejected: "
                    f"{node.marker_rejection}")
            else:
                detail = (
                    f"{publishers} publisher(s) discovered on "
                    f"{node.marker_topic}, but no sample was received")
            raise RuntimeError(
                f"timed out waiting for the current measured tip: {detail}")
        spec = load_trajectory_file(
            parsed.yaml_file, node.tip, action_override=parsed.action_name,
            marker_override=parsed.marker_topic)
        if not node.client.wait_for_server(timeout_sec=parsed.timeout_s):
            raise RuntimeError(
                "timed out waiting for trajectory action server")
        node.get_logger().info(
            "loaded %d waypoints; initial tip=[%.3f, %.3f, %.3f] mm" % (
                len(spec.waypoints_m), *(1000.0*value for value in node.tip)))
        future = node.client.send_goal_async(
            _goal_from_spec(node, spec), feedback_callback=node.feedback_cb)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("trajectory action goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("trajectory action returned no result")
        result = wrapped.result
        node.get_logger().info(
            "%s; reached=%d timed_out=%d final_error=%.3f mm" % (
                result.message, result.reached_waypoints,
                result.timed_out_waypoints, result.final_error_mm))
        if not result.success:
            raise RuntimeError(
                f"trajectory did not complete: {result.message}")
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        node.get_logger().warn(
            "trajectory client interrupted; cancel requested")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
