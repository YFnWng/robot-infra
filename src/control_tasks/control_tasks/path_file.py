"""Load a continuous tip path from YAML and submit its action goal."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

from control_interface.action import TrackTipPath
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
from .trajectory_file import _circle_waypoints


@dataclass(frozen=True)
class PathFileSpec:
    frame_id: str
    action_name: str
    marker_topic: str
    knots_m: np.ndarray
    nominal_speed_mm_s: float
    total_timeout_s: float
    final_tolerance_mm: float
    final_settle_time_s: float
    soft_error_mm: float
    pause_error_mm: float
    resume_error_mm: float
    hard_error_mm: float
    auto_arm: bool


def _number(document, name, default=None, *, allow_zero=False):
    value = document.get(name, default)
    if value is None:
        raise ValueError(f"{name} is required")
    result = float(value)
    invalid = result < 0.0 if allow_zero else result <= 0.0
    if not math.isfinite(result) or invalid:
        qualifier = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


def load_path_file(path, current_tip_m, *, action_override=None,
                   marker_override=None, speed_override=None,
                   total_timeout_override=None):
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("path YAML root must be a mapping")
    tip = np.asarray(current_tip_m, dtype=np.float64)
    if tip.shape != (3,) or not np.all(np.isfinite(tip)):
        raise ValueError(
            "current measured tip must contain three finite values")
    generator = document.get("generator")
    explicit = document.get("path_knots_m")
    if (generator is None) == (explicit is None):
        raise ValueError("provide exactly one of generator or path_knots_m")
    knots = (_circle_waypoints(generator, tip) if generator is not None
             else np.asarray(explicit, dtype=np.float64))
    if (knots.ndim != 2 or knots.shape[1:] != (3,)
            or len(knots) < 2 or len(knots) > 500
            or not np.all(np.isfinite(knots))):
        raise ValueError("path knots must be a finite Nx3 array, N in [2,500]")
    # The action always begins at the measured catheter tip. This creates a
    # continuous approach segment and prevents an initial reference jump.
    if np.linalg.norm(knots[0]-tip) > 1e-7:
        knots = np.concatenate((tip[None, :], knots), axis=0)
    frame_id = str(document.get("frame_id", "robot_base"))
    action_name = str(action_override or document.get(
        "action_name", "/catheter_mppi/track_tip_path"))
    marker_topic = str(marker_override or document.get(
        "marker_topic", "/shape_tracking/markers"))
    auto_arm = document.get("auto_arm", False)
    if not isinstance(auto_arm, bool):
        raise ValueError("auto_arm must be a YAML boolean")
    if speed_override is not None:
        document["nominal_speed_mm_s"] = speed_override
    if total_timeout_override is not None:
        document["total_timeout_s"] = total_timeout_override
    return PathFileSpec(
        frame_id=frame_id, action_name=action_name,
        marker_topic=marker_topic, knots_m=knots,
        nominal_speed_mm_s=_number(document, "nominal_speed_mm_s", 2.0),
        total_timeout_s=_number(document, "total_timeout_s", 120.0),
        final_tolerance_mm=_number(document, "final_tolerance_mm", 1.8),
        final_settle_time_s=_number(
            document, "final_settle_time_s", 0.3, allow_zero=True),
        soft_error_mm=_number(document, "soft_error_mm", 2.0),
        pause_error_mm=_number(document, "pause_error_mm", 5.0),
        resume_error_mm=_number(document, "resume_error_mm", 1.5),
        hard_error_mm=_number(document, "hard_error_mm", 15.0),
        auto_arm=auto_arm)


class PathFileClient(Node):
    def __init__(self, frame_id, marker_topic, action_name):
        super().__init__("catheter_tip_path_file")
        self.frame_id = frame_id
        self.marker_topic = marker_topic
        self.tip = None
        self.marker_rejection = None
        self.last_governor_state = None
        self.create_subscription(
            PointCloud, marker_topic, self._marker_cb,
            qos_profile_sensor_data)
        self.client = ActionClient(self, TrackTipPath, action_name)

    def _marker_cb(self, message):
        try:
            self.tip = np.asarray(
                measured_tip(message, self.frame_id), dtype=np.float64)
            self.marker_rejection = None
        except ValueError as error:
            self.marker_rejection = str(error)

    def feedback_cb(self, message):
        feedback = message.feedback
        if feedback.governor_state != self.last_governor_state:
            self.last_governor_state = feedback.governor_state
            self.get_logger().info(
                "%s: progress=%.1f%% reference=%.3f mm path=%.3f mm" % (
                    feedback.governor_state, 100.0*feedback.progress,
                    feedback.reference_error_mm,
                    feedback.closest_path_error_mm))


def _arguments(argv):
    parser = argparse.ArgumentParser(
        description="Load and execute a continuous catheter tip path YAML")
    parser.add_argument("yaml_file")
    parser.add_argument("--action-name")
    parser.add_argument("--marker-topic")
    parser.add_argument(
        "--speed-mm-s", type=float,
        help="override nominal_speed_mm_s without editing the YAML")
    parser.add_argument(
        "--total-timeout-s", type=float,
        help="override the action's total path timeout")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parsed = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not math.isfinite(parsed.timeout_s) or parsed.timeout_s <= 0.0:
        parser.error("--timeout-s must be finite and positive")
    for name in ("speed_mm_s", "total_timeout_s"):
        value = getattr(parsed, name)
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            parser.error(
                f"--{name.replace('_', '-')} must be finite and positive")
    return parsed


def _goal(node, spec):
    goal = TrackTipPath.Goal()
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.header.frame_id = spec.frame_id
    goal.path_knots = [Point(x=float(x), y=float(y), z=float(z))
                       for x, y, z in spec.knots_m]
    for name in (
            "nominal_speed_mm_s", "total_timeout_s", "final_tolerance_mm",
            "final_settle_time_s", "soft_error_mm", "pause_error_mm",
            "resume_error_mm", "hard_error_mm", "auto_arm"):
        setattr(goal, name, getattr(spec, name))
    return goal


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    with Path(parsed.yaml_file).expanduser().open(
            "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError("path YAML root must be a mapping")
    frame_id = str(metadata.get("frame_id", "robot_base"))
    action_name = str(parsed.action_name or metadata.get(
        "action_name", "/catheter_mppi/track_tip_path"))
    marker_topic = str(parsed.marker_topic or metadata.get(
        "marker_topic", "/shape_tracking/markers"))
    rclpy.init(args=argv)
    node = PathFileClient(frame_id, marker_topic, action_name)
    goal_handle = None
    try:
        deadline = time.monotonic()+parsed.timeout_s
        while node.tip is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.tip is None:
            detail = node.marker_rejection or "no accepted marker sample"
            raise RuntimeError(f"timed out waiting for measured tip: {detail}")
        spec = load_path_file(
            parsed.yaml_file, node.tip, action_override=parsed.action_name,
            marker_override=parsed.marker_topic,
            speed_override=parsed.speed_mm_s,
            total_timeout_override=parsed.total_timeout_s)
        if not node.client.wait_for_server(timeout_sec=parsed.timeout_s):
            raise RuntimeError("timed out waiting for continuous path server")
        node.get_logger().info(
            "loaded %d path knots at %.3f mm/s, timeout %.1f s; "
            "start=[%.3f, %.3f, %.3f] mm"
            % (len(spec.knots_m), spec.nominal_speed_mm_s,
               spec.total_timeout_s,
               *(1000.0*value for value in node.tip)))
        future = node.client.send_goal_async(
            _goal(node, spec), feedback_callback=node.feedback_cb)
        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("continuous path goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("continuous path action returned no result")
        result = wrapped.result
        node.get_logger().info(
            "%s; final=%.3f mm RMS-path=%.3f mm P95-path=%.3f mm" % (
                result.message, result.final_error_mm,
                result.rms_closest_path_error_mm,
                result.p95_closest_path_error_mm))
        if not result.success:
            raise RuntimeError(f"continuous path failed: {result.message}")
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        node.get_logger().warn("continuous path interrupted; cancel requested")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
