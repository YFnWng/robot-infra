"""Load a continuous tip path from YAML and submit its action goal."""
from __future__ import annotations

from .task_outcomes import TaskFailure

import argparse
import json
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
    x_only: bool = False
    xz_only: bool = False


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


def _finite_json(value):
    """Retain unavailable action/feedback values as null in strict JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_finite_json(item) for item in value]
    return value


def load_path_file(path, current_tip_m, *, action_override=None,
                   marker_override=None, speed_override=None,
                   total_timeout_override=None, preserve_knots=False):
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
    if generator is not None:
        if not isinstance(generator, dict):
            raise ValueError("generator must be a mapping")
        if generator.get("type") in (
                "insertion_line_out_and_back_from_current_tip",
                "global_x_line_out_and_back_from_current_tip"):
            length_m = 1e-3*_number(generator, "length_mm")
            # Robot-base +Z is insertion. Revisit the same geometric segment
            # in reverse order; never use a model prediction for the anchor.
            offset = ([length_m, 0.0, 0.0] if generator.get("type") ==
                      "global_x_line_out_and_back_from_current_tip"
                      else [0.0, 0.0, length_m])
            knots = np.stack((tip, tip + offset, tip))
        else:
            knots = _circle_waypoints(generator, tip)
    else:
        knots = np.asarray(explicit, dtype=np.float64)
    if (knots.ndim != 2 or knots.shape[1:] != (3,)
            or len(knots) < 2 or len(knots) > 500
            or not np.all(np.isfinite(knots))):
        raise ValueError("path knots must be a finite Nx3 array, N in [2,500]")
    # The action always begins at the measured catheter tip. This creates a
    # continuous approach segment and prevents an initial reference jump.
    if not preserve_knots and np.linalg.norm(knots[0]-tip) > 1e-7:
        knots = np.concatenate((tip[None, :], knots), axis=0)
    frame_id = str(document.get("frame_id", "robot_base"))
    action_name = str(action_override or document.get(
        "action_name", "/catheter_mppi/track_tip_path"))
    marker_topic = str(marker_override or document.get(
        "marker_topic", "/shape_tracking/markers"))
    auto_arm = document.get("auto_arm", False)
    if not isinstance(auto_arm, bool):
        raise ValueError("auto_arm must be a YAML boolean")
    x_only = document.get("x_only", False)
    xz_only = document.get("xz_only", False)
    if not isinstance(xz_only, bool) or (x_only and xz_only):
        raise ValueError("xz_only must be boolean and mutually exclusive with x_only")
    if not isinstance(x_only, bool):
        raise ValueError("x_only must be a YAML boolean")
    if (x_only or xz_only) and (frame_id != "robot_base" or
                   not np.allclose(knots[:, 1:], knots[0, 1:], rtol=0, atol=1e-12)):
        raise ValueError("x_only/xz_only require a robot_base X line with constant Y/Z")
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
        auto_arm=auto_arm, x_only=x_only, xz_only=xz_only)


class PathFileClient(Node):
    def __init__(self, frame_id, marker_topic, action_name):
        super().__init__("catheter_tip_path_file")
        self.frame_id = frame_id
        self.marker_topic = marker_topic
        self.tip = None
        self.marker_rejection = None
        self.last_governor_state = None
        self.tip_received_at = None
        self.feedback_records = []
        self.create_subscription(
            PointCloud, marker_topic, self._marker_cb,
            qos_profile_sensor_data)
        self.client = ActionClient(self, TrackTipPath, action_name)

    def _marker_cb(self, message):
        try:
            self.tip = np.asarray(
                measured_tip(message, self.frame_id), dtype=np.float64)
            self.marker_rejection = None
            self.tip_received_at = time.monotonic()
        except ValueError as error:
            self.marker_rejection = str(error)

    def feedback_cb(self, message):
        feedback = message.feedback
        self.feedback_records.append({name: getattr(feedback, name) for name in (
            "elapsed_s", "progress", "reference_error_mm",
            "closest_path_error_mm", "along_track_error_mm",
            "cross_track_error_mm", "governor_state", "controller_state",
            "covered_arc_length_m", "path_coverage", "traversal_complete")})
        if feedback.governor_state != self.last_governor_state:
            self.last_governor_state = feedback.governor_state
            self.get_logger().info(
                "%s: reference-progress=%.1f%% coverage=%.1f%% reference=%.3f mm path=%.3f mm" % (
                    feedback.governor_state, 100.0*feedback.progress,
                    100.0*feedback.path_coverage,
                    feedback.reference_error_mm,
                    feedback.closest_path_error_mm))


def _arguments(argv):
    parser = argparse.ArgumentParser(
        description="Load and execute a continuous catheter tip path YAML")
    parser.add_argument("yaml_file")
    parser.add_argument("--action-name")
    parser.add_argument("--marker-topic")
    parser.add_argument("--freeze-path", type=Path,
                        help="Save absolute knots from a fresh tip; do not submit or arm")
    parser.add_argument("--result-file", type=Path,
                        help="Exclusive JSON output including feedback and failures")
    parser.add_argument("--preserve-knots", action="store_true",
                        help="Submit frozen knots unchanged after separate start acquisition")
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
            "resume_error_mm", "hard_error_mm", "auto_arm", "x_only", "xz_only"):
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
    record = {"success": False, "message": "interrupted before submission"}
    result_stream = None
    try:
        if parsed.result_file is not None:
            result_stream = parsed.result_file.open("x", encoding="utf-8")
        deadline = time.monotonic()+parsed.timeout_s
        while (node.tip_received_at is None
               or time.monotonic()-node.tip_received_at > .5):
            if time.monotonic() >= deadline:
                raise RuntimeError("timed out waiting for fresh measured tip")
            rclpy.spin_once(node, timeout_sec=.1)
        if node.tip is None:
            detail = node.marker_rejection or "no accepted marker sample"
            raise RuntimeError(f"timed out waiting for measured tip: {detail}")
        spec = load_path_file(
            parsed.yaml_file, node.tip, action_override=parsed.action_name,
            marker_override=parsed.marker_topic,
            speed_override=parsed.speed_mm_s,
            total_timeout_override=parsed.total_timeout_s,
            preserve_knots=parsed.preserve_knots)
        if parsed.freeze_path is not None:
            frozen = dict(metadata)
            frozen.pop("generator", None)
            frozen["path_knots_m"] = spec.knots_m.tolist()
            frozen["reference_tip_m"] = node.tip.tolist()
            with parsed.freeze_path.open("x", encoding="utf-8") as stream:
                yaml.safe_dump(frozen, stream, sort_keys=False)
            node.get_logger().info(f"frozen path -> {parsed.freeze_path}; no goal sent")
            return
        if parsed.preserve_knots:
            start_error_mm = 1000.0*float(np.linalg.norm(node.tip-spec.knots_m[0]))
            if start_error_mm > spec.final_tolerance_mm:
                raise RuntimeError(
                    f"path start not acquired: error={start_error_mm:.3f} mm, "
                    f"tolerance={spec.final_tolerance_mm:.3f} mm")
        record.update(path_knots_m=spec.knots_m.tolist(),
                      x_only=spec.x_only,
                      xz_only=spec.xz_only,
                      nominal_speed_mm_s=spec.nominal_speed_mm_s,
                      started_ros_ns=node.get_clock().now().nanoseconds,
                      started_monotonic_ns=time.monotonic_ns())
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
        record.update({name: getattr(result, name) for name in (
            "success", "message", "final_error_mm", "rms_closest_path_error_mm",
            "p95_closest_path_error_mm", "completed_arc_length_m",
            "reference_arc_length_m", "path_coverage", "traversal_complete",
            "fault_reason", "fault_snapshot_json")})
        node.get_logger().info(
            "%s; final=%.3f mm RMS-path=%.3f mm P95-path=%.3f mm" % (
                result.message, result.final_error_mm,
                result.rms_closest_path_error_mm,
                result.p95_closest_path_error_mm))
        if not result.success:
            raise TaskFailure(f"continuous path failed: {result.message}",
                              fault_reason=result.fault_reason,
                              fault_snapshot_json=result.fault_snapshot_json)
    except KeyboardInterrupt:
        if goal_handle is not None and goal_handle.accepted:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        node.get_logger().warn("continuous path interrupted; cancel requested")
        record["message"] = "interrupted; cancel requested"
    except Exception as error:
        record["message"] = str(error)
        if goal_handle is not None and goal_handle.accepted:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        if isinstance(error, TaskFailure):
            node.get_logger().error(str(error))
            return 2
        raise
    finally:
        if result_stream is not None:
            record.update(ended_ros_ns=node.get_clock().now().nanoseconds,
                          ended_monotonic_ns=time.monotonic_ns(),
                          feedback=node.feedback_records)
            json.dump(_finite_json(record), result_stream, indent=2, allow_nan=False)
            result_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
