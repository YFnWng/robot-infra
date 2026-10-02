"""Run and score one axis-separated robustness scenario on the /sim graph."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

from control_interface.msg import ControlStream
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool


DEFAULT_SESSION_ROOT = Path(
    "/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions")


class ScenarioMonitor(Node):
    def __init__(self):
        super().__init__("catheter_sim_scenario")
        self.tip = None
        self.controller_state = None
        self.controller_values = {}
        self.device_values = {}
        self.sensor_values = {}
        self.tip_samples = []
        self.command_samples = []
        self.target_pub = self.create_publisher(
            PointStamped, "/sim/catheter_mppi/target_tip", 10)
        self.arm_client = self.create_client(
            SetBool, "/sim/catheter_mppi/set_armed")
        self.create_subscription(
            PointStamped, "/sim/catheter_sim/ground_truth_tip",
            self._tip_cb, 10)
        self.create_subscription(
            ControlStream, "/sim/catheter_sim/realized_control",
            self._command_cb, 10)
        self.create_subscription(
            DiagnosticArray, "/sim/catheter_mppi/status",
            self._controller_cb, 10)
        self.create_subscription(
            DiagnosticArray, "/sim/catheter_sim/device_status",
            self._device_cb, 10)
        self.create_subscription(
            DiagnosticArray, "/sim/shape_tracking/marker_status",
            self._sensor_cb, 10)

    def _tip_cb(self, message):
        self.tip = message
        self.tip_samples.append((
            time.monotonic(), np.asarray([
                message.point.x, message.point.y, message.point.z])))

    def _command_cb(self, message):
        if len(message.joint_vel) == 6:
            self.command_samples.append((
                time.monotonic(), np.asarray(message.joint_vel, dtype=float)))

    def _controller_cb(self, message):
        for status in message.status:
            if status.name == "catheter_control/mppi":
                self.controller_state = status.message
                self.controller_values = {
                    value.key: value.value for value in status.values}

    def _device_cb(self, message):
        for status in message.status:
            if status.name == "catheter_control/simulated_device":
                self.device_values = {
                    value.key: value.value for value in status.values}

    def _sensor_cb(self, message):
        for status in message.status:
            if status.name == "automation/four_ring_markers":
                self.sensor_values = {
                    "message": status.message,
                    **{value.key: value.value for value in status.values}}


def _arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--startup-timeout-s", type=float, default=20.0)
    parser.add_argument("--settle-band-mm", type=float, default=0.5)
    parser.add_argument("--settle-hold-s", type=float, default=1.0)
    parser.add_argument("--pass-error-mm", type=float, default=1.0)
    parser.add_argument("--label", default="robustness")
    parser.add_argument("--session-root", type=Path,
                        default=DEFAULT_SESSION_ROOT)
    result = parser.parse_args(remove_ros_args(args=argv)[1:])
    offsets = (result.dx_mm, result.dy_mm, result.dz_mm)
    if sum(value != 0.0 for value in offsets) != 1:
        parser.error("exactly one of --dx-mm, --dy-mm, --dz-mm must be nonzero")
    positive = (result.duration_s, result.startup_timeout_s,
                result.settle_band_mm, result.settle_hold_s,
                result.pass_error_mm)
    if any(not math.isfinite(value) or value <= 0.0 for value in positive):
        parser.error("durations and error thresholds must be positive")
    return result


def _spin_until(node, predicate, timeout_s):
    deadline = time.monotonic()+timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _call_arm(node, armed, timeout_s=5.0):
    if not node.arm_client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError("simulated controller arm service is unavailable")
    request = SetBool.Request()
    request.data = bool(armed)
    future = node.arm_client.call_async(request)
    if not _spin_until(node, future.done, timeout_s):
        raise RuntimeError("simulated controller arm service timed out")
    response = future.result()
    if response is None or not response.success:
        detail = "no response" if response is None else response.message
        raise RuntimeError(f"simulated arm={armed} failed: {detail}")


def _integral(samples, transform):
    if len(samples) < 2:
        return 0.0
    result = 0.0
    previous_t, previous_v = samples[0]
    previous_y = transform(previous_v)
    for timestamp, value in samples[1:]:
        current_y = transform(value)
        result += 0.5*(previous_y+current_y)*(timestamp-previous_t)
        previous_t, previous_y = timestamp, current_y
    return float(result)


def run(arguments):
    rclpy.init(args=sys.argv)
    node = ScenarioMonitor()
    armed = False
    try:
        ready = _spin_until(node, lambda: (
            node.tip is not None
            and node.controller_values.get("manager_ready") == "True"
            and node.controller_values.get("estimator_health") == "TRACKING"
        ), arguments.startup_timeout_s)
        if not ready:
            raise RuntimeError(
                "simulation did not reach manager-ready estimator tracking")
        start_tip = np.asarray([
            node.tip.point.x, node.tip.point.y, node.tip.point.z])
        offset_mm = np.asarray([
            arguments.dx_mm, arguments.dy_mm, arguments.dz_mm])
        target = start_tip+1e-3*offset_mm
        target_message = PointStamped()
        target_message.header.stamp = node.get_clock().now().to_msg()
        target_message.header.frame_id = node.tip.header.frame_id
        target_message.point.x, target_message.point.y, target_message.point.z = (
            target.tolist())
        for _ in range(3):
            node.target_pub.publish(target_message)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.tip_samples.clear()
        node.command_samples.clear()
        _call_arm(node, True)
        armed = True
        started = time.monotonic()
        settled_since = None
        settled_at = None
        fault = None
        errors = []
        while time.monotonic()-started < arguments.duration_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.tip is not None:
                point = np.asarray([
                    node.tip.point.x, node.tip.point.y, node.tip.point.z])
                error_mm = 1e3*float(np.linalg.norm(target-point))
                errors.append((time.monotonic(), error_mm))
                if error_mm <= arguments.settle_band_mm:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic()-settled_since >= arguments.settle_hold_s:
                        settled_at = settled_since-started
                        break
                else:
                    settled_since = None
            if node.controller_state == "FAULTED":
                fault = node.controller_values.get("reason", "unknown")
                break
        _call_arm(node, False)
        armed = False
        disarmed_at = time.monotonic()
        _spin_until(node, lambda: time.monotonic()-disarmed_at >= 0.25, 0.5)
        if not errors:
            raise RuntimeError("no ground-truth samples arrived during scenario")
        error_samples = [(stamp, np.asarray([value]))
                         for stamp, value in errors]
        final_error = errors[-1][1]
        all_commands = [value for _, value in node.command_samples]
        post_stop_commands = [value for stamp, value in node.command_samples
                              if stamp >= disarmed_at]
        commands_finite = bool(
            all_commands and all(np.isfinite(value).all()
                                 for value in all_commands))
        stopped_at_zero = bool(
            post_stop_commands
            and np.linalg.norm(post_stop_commands[-1]) <= 1e-9)
        report = {
            "schema": "catheter-simulation-robustness-scenario-v1",
            "created": datetime.now().isoformat(),
            "label": arguments.label,
            "target_offset_mm": offset_mm.tolist(),
            "duration_requested_s": arguments.duration_s,
            "duration_observed_s": errors[-1][0]-errors[0][0],
            "settling_time_s": settled_at,
            "final_error_mm": final_error,
            "best_error_mm": min(value for _, value in errors),
            "maximum_error_mm": max(value for _, value in errors),
            "integrated_absolute_error_mm_s": _integral(
                error_samples, lambda value: float(value[0])),
            "command_energy": _integral(
                node.command_samples,
                lambda value: float(np.dot(value, value))),
            "commands_finite": commands_finite,
            "stopped_at_zero": stopped_at_zero,
            "controller_state": node.controller_state,
            "fault": fault,
            "controller_diagnostics": node.controller_values,
            "device_configuration": node.device_values,
            "sensor_configuration": node.sensor_values,
            "passed_tracking": bool(
                fault is None and final_error <= arguments.pass_error_mm),
            "passed_safety": bool(commands_finite and stopped_at_zero),
        }
        output_dir = arguments.session_root/(
            datetime.now().strftime("%Y%m%d_%H%M%S_")+arguments.label)
        output_dir.mkdir(parents=True, exist_ok=False)
        output = output_dir/"scenario_result.json"
        output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        print(json.dumps({"output": str(output), **{
            key: report[key] for key in (
                "passed_tracking", "passed_safety", "final_error_mm",
                "best_error_mm", "settling_time_s", "fault")}}, indent=2))
        return 0 if report["passed_tracking"] and report["passed_safety"] else 2
    finally:
        if armed:
            try:
                _call_arm(node, False)
            except Exception as error:
                node.get_logger().error(f"failed to disarm simulation: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    arguments = _arguments(sys.argv if args is None else args)
    raise SystemExit(run(arguments))


if __name__ == "__main__":
    main()
