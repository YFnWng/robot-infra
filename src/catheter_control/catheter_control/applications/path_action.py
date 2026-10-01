"""Continuous Cartesian tip-path action server and reference governor."""
from __future__ import annotations

import math
import threading
import time
import uuid

from control_interface.action import TrackTipPath
from control_interface.msg import PathTrackingTrace, TipReferenceHorizon
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Point32, PointStamped, Vector3
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud
from std_srvs.srv import SetBool

from ..planning.path_tracking import ArcLengthPath, GovernorConfig, PathProgressGovernor
from .target_offset import controller_status, measured_tip


class TipPathActionServer(Node):
    """Publish a continuously advancing, estimator-time-aligned preview."""

    def __init__(self):
        super().__init__("catheter_tip_path")
        defaults = {
            "frame_id": "robot_base",
            "update_rate_hz": 30.0,
            "feedback_timeout_s": 0.5,
            "service_timeout_s": 5.0,
            "maximum_knots": 500,
            "preview_history_s": 0.20,
            "preview_duration_s": 0.60,
            "preview_sample_period_s": 0.02,
            "reference_expiry_s": 0.15,
            "recovery_speed_scale": 0.20,
            "marker_topic": "/shape_tracking/markers",
            "status_topic": "/catheter_mppi/status",
            "reference_topic": "/catheter_mppi/reference_horizon",
            "path_topic": "/catheter_mppi/reference_path",
            "reference_point_topic": "/catheter_mppi/path_reference_point",
            "arm_service": "/catheter_mppi/set_armed",
            "action_name": "/catheter_mppi/track_tip_path",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value)
        self.service_timeout_s = float(
            self.get_parameter("service_timeout_s").value)
        self.maximum_knots = int(self.get_parameter("maximum_knots").value)
        self.history_s = float(self.get_parameter("preview_history_s").value)
        self.preview_s = float(
            self.get_parameter("preview_duration_s").value)
        self.preview_dt = float(
            self.get_parameter("preview_sample_period_s").value)
        self.reference_expiry_s = float(
            self.get_parameter("reference_expiry_s").value)
        self.recovery_speed_scale = float(
            self.get_parameter("recovery_speed_scale").value)
        numeric = (
            self.rate_hz, self.feedback_timeout_s, self.service_timeout_s,
            self.preview_s, self.preview_dt, self.reference_expiry_s,
            self.recovery_speed_scale)
        if (not self.frame_id or any(value <= 0.0 for value in numeric)
                or self.history_s < 0.0 or self.maximum_knots < 2
                or self.preview_s <= self.history_s):
            raise ValueError("continuous-path server parameters are invalid")

        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._goal_reserved = False
        self.tip = None
        self.tip_time = None
        self.controller_armed = False
        self.controller_state = None
        self.controller_reason = None
        self.controller_time = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.reference_pub = self.create_publisher(
            TipReferenceHorizon,
            str(self.get_parameter("reference_topic").value), qos)
        self.path_pub = self.create_publisher(
            PointCloud, str(self.get_parameter("path_topic").value), path_qos)
        self.point_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("reference_point_topic").value), 10)
        self.trace_pub = self.create_publisher(
            PathTrackingTrace, "/catheter_mppi/path_tracking_trace", 20)
        self.arm_client = self.create_client(
            SetBool, str(self.get_parameter("arm_service").value),
            callback_group=group)
        self.create_subscription(
            PointCloud, str(self.get_parameter("marker_topic").value),
            self._marker_cb, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            DiagnosticArray, str(self.get_parameter("status_topic").value),
            self._status_cb, 10, callback_group=group)
        self.action_server = ActionServer(
            self, TrackTipPath, str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

    def _marker_cb(self, message):
        try:
            tip = measured_tip(message, self.frame_id)
        except ValueError:
            return
        with self._lock:
            self.tip = np.asarray(tip, dtype=np.float64)
            self.tip_time = time.monotonic()

    def _status_cb(self, message):
        try:
            armed, state, reason = controller_status(message)
        except ValueError:
            return
        with self._lock:
            self.controller_armed = armed
            self.controller_state = state
            self.controller_reason = reason
            self.controller_time = time.monotonic()

    def _snapshot(self):
        with self._lock:
            tip = None if self.tip is None else self.tip.copy()
            return (tip, self.tip_time, self.controller_armed,
                    self.controller_state, self.controller_reason,
                    self.controller_time)

    def _spec(self, request):
        if request.header.frame_id != self.frame_id:
            raise ValueError(f"goal frame must be {self.frame_id!r}")
        if len(request.path_knots) > self.maximum_knots:
            raise ValueError(
                f"goal exceeds maximum_knots={self.maximum_knots}")
        path = ArcLengthPath([
            [point.x, point.y, point.z] for point in request.path_knots])
        config = GovernorConfig(
            nominal_speed_m_s=1e-3*float(request.nominal_speed_mm_s),
            total_timeout_s=float(request.total_timeout_s),
            final_tolerance_mm=float(request.final_tolerance_mm),
            final_settle_time_s=float(request.final_settle_time_s),
            soft_error_mm=float(request.soft_error_mm),
            pause_error_mm=float(request.pause_error_mm),
            resume_error_mm=float(request.resume_error_mm),
            hard_error_mm=float(request.hard_error_mm),
            recovery_speed_scale=self.recovery_speed_scale)
        return path, config

    def _goal_callback(self, request):
        try:
            self._spec(request)
        except ValueError as error:
            self.get_logger().warn(f"rejected path goal: {error}")
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _publish_path(self, path):
        message = PointCloud()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        samples, _ = path.evaluate(np.linspace(0.0, path.length_m, 240))
        message.points = [Point32(x=float(x), y=float(y), z=float(z))
                          for x, y, z in samples]
        self.path_pub.publish(message)

    def _publish_reference(self, path_id, sequence, governor, update):
        offsets = np.arange(
            -self.history_s, self.preview_s+self.preview_dt/2.0,
            self.preview_dt)
        # Recovery governs the *phase* conservatively, but MPPI still needs a
        # noncollapsed forward arc window to identify commands that move on
        # from an unreachable local point. Transmission and final holds are
        # the only states that intentionally collapse the preview.
        preview_scale = (
            0.0 if update.state in {"TRANSMISSION_HOLD", "FINAL_HOLD"}
            else 1.0 if update.state == "RECOVERY_ADVANCE"
            else update.speed_scale)
        points, tangents = governor.preview(preview_scale, offsets)
        now = self.get_clock().now()
        message = TipReferenceHorizon()
        message.header.stamp = Time(
            nanoseconds=now.nanoseconds-int(1e9*self.history_s)).to_msg()
        message.header.frame_id = self.frame_id
        message.path_id = path_id
        message.sequence = sequence
        message.sample_period_s = self.preview_dt
        message.positions = [Point(x=float(x), y=float(y), z=float(z))
                             for x, y, z in points]
        message.tangents = [Vector3(x=float(x), y=float(y), z=float(z))
                            for x, y, z in tangents]
        message.progress_m = update.progress_m
        message.total_length_m = governor.path.length_m
        message.nominal_speed_m_s = governor.config.nominal_speed_m_s
        message.final_hold = update.state == "FINAL_HOLD"
        message.progress_paused = update.speed_scale == 0.0
        message.expiry_s = self.reference_expiry_s
        self.reference_pub.publish(message)
        point = PointStamped()
        point.header.stamp = now.to_msg()
        point.header.frame_id = self.frame_id
        point.point = Point(
            x=float(update.reference_point_m[0]),
            y=float(update.reference_point_m[1]),
            z=float(update.reference_point_m[2]))
        self.point_pub.publish(point)

    def _invalidate(self, path_id, sequence):
        message = TipReferenceHorizon()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.path_id = path_id
        message.sequence = sequence
        self.reference_pub.publish(message)

    def _set_armed(self, armed, pump=None):
        if not self.arm_client.wait_for_service(
                timeout_sec=self.service_timeout_s):
            return False, "controller arm service unavailable"
        request = SetBool.Request()
        request.data = bool(armed)
        future = self.arm_client.call_async(request)
        deadline = time.monotonic()+self.service_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if pump is not None:
                pump()
            time.sleep(min(.02, 1.0/self.rate_hz))
        if not future.done() or future.result() is None:
            return False, "controller arm service timed out"
        response = future.result()
        return bool(response.success), str(response.message)

    def _result(self, success, message, errors, update=None):
        result = TrackTipPath.Result()
        result.success = bool(success)
        result.message = str(message)
        result.final_error_mm = (
            math.nan if update is None else update.reference_error_mm)
        result.rms_closest_path_error_mm = (
            math.nan if not errors
            else float(np.sqrt(np.mean(np.square(errors)))))
        result.p95_closest_path_error_mm = (
            math.nan if not errors else float(np.percentile(errors, 95)))
        result.completed_arc_length_m = (
            0.0 if update is None else update.progress_m)
        return result

    def _feedback(self, path_id, update, config, state, total_length_m):
        feedback = TrackTipPath.Feedback()
        feedback.path_id = path_id
        feedback.progress = update.progress_fraction
        feedback.arc_length_m = update.progress_m
        feedback.total_length_m = total_length_m
        feedback.elapsed_s = update.elapsed_s
        feedback.phase_lag_s = (
            (update.progress_m-update.closest.arc_m)
            / config.nominal_speed_m_s)
        feedback.reference_point = Point(
            x=float(update.reference_point_m[0]),
            y=float(update.reference_point_m[1]),
            z=float(update.reference_point_m[2]))
        feedback.tangent = Vector3(
            x=float(update.tangent[0]), y=float(update.tangent[1]),
            z=float(update.tangent[2]))
        feedback.reference_error_mm = update.reference_error_mm
        feedback.closest_path_error_mm = update.closest.distance_mm
        feedback.along_track_error_mm = update.closest.along_error_mm
        feedback.cross_track_error_mm = update.closest.cross_error_mm
        feedback.governor_state = update.state
        feedback.controller_state = str(state)
        return feedback

    def _publish_trace(self, path_id, sequence, update, config, tip, state,
                       total_length_m):
        message = PathTrackingTrace()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.path_id = path_id
        message.sequence = sequence
        message.progress_m = update.progress_m
        message.total_length_m = total_length_m
        message.nominal_speed_m_s = config.nominal_speed_m_s
        message.governed_speed_scale = update.speed_scale
        message.reference_point = Point(
            x=float(update.reference_point_m[0]),
            y=float(update.reference_point_m[1]),
            z=float(update.reference_point_m[2]))
        message.tangent = Vector3(
            x=float(update.tangent[0]), y=float(update.tangent[1]),
            z=float(update.tangent[2]))
        message.measured_tip = Point(
            x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))
        message.closest_path_point = Point(
            x=float(update.closest.point_m[0]),
            y=float(update.closest.point_m[1]),
            z=float(update.closest.point_m[2]))
        message.reference_error_mm = update.reference_error_mm
        message.closest_path_error_mm = update.closest.distance_mm
        message.along_track_error_mm = update.closest.along_error_mm
        message.cross_track_error_mm = update.closest.cross_error_mm
        message.governor_state = update.state
        message.controller_state = str(state)
        self.trace_pub.publish(message)

    def _execute(self, goal_handle):
        path, config = self._spec(goal_handle.request)
        governor = PathProgressGovernor(path, config)
        path_id = uuid.uuid4().hex
        sequence = 0
        errors = []
        update = None
        auto_armed = False
        self._publish_path(path)
        try:
            deadline = time.monotonic()+self.service_timeout_s
            while rclpy.ok() and time.monotonic() < deadline:
                (tip, tip_time, armed, state, controller_reason,
                 status_time) = self._snapshot()
                now = time.monotonic()
                if (tip is not None and tip_time is not None
                        and status_time is not None
                        and now-tip_time <= self.feedback_timeout_s
                        and now-status_time <= self.feedback_timeout_s):
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(False, "canceled before start", errors)
                time.sleep(.02)
            else:
                goal_handle.abort()
                return self._result(
                    False, "fresh feedback unavailable", errors)

            governor.start(time.monotonic())
            update = governor.update(tip, time.monotonic())

            def pump():
                nonlocal sequence
                sequence += 1
                self._publish_reference(path_id, sequence, governor, update)

            pump()
            if goal_handle.request.auto_arm and not armed:
                ok, detail = self._set_armed(True, pump)
                if not ok:
                    goal_handle.abort()
                    return self._result(
                        False, f"arming failed: {detail}", errors)
                auto_armed = True
            elif not armed:
                goal_handle.abort()
                return self._result(
                    False, "controller is not armed and auto_arm is false",
                    errors)

            armed_deadline = time.monotonic()+self.service_timeout_s
            while rclpy.ok() and time.monotonic() < armed_deadline:
                _, _, armed, state, _, _ = self._snapshot()
                if armed:
                    break
                if state == "FAULTED":
                    goal_handle.abort()
                    return self._result(
                        False, "controller faulted while arming", errors)
                pump()
                time.sleep(min(.02, 1.0/self.rate_hz))
            else:
                goal_handle.abort()
                return self._result(
                    False, "controller did not report armed", errors)

            period = 1.0/self.rate_hz
            while rclpy.ok():
                started = time.monotonic()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(False, "path canceled", errors, update)
                (tip, tip_time, armed, state, controller_reason,
                 status_time) = self._snapshot()
                if (tip is None or tip_time is None or status_time is None
                        or started-tip_time > self.feedback_timeout_s
                        or started-status_time > self.feedback_timeout_s):
                    goal_handle.abort()
                    return self._result(
                        False, "tip/controller feedback became stale",
                        errors, update)
                if state == "FAULTED" or not armed:
                    goal_handle.abort()
                    return self._result(
                        False, f"controller {state}", errors, update)
                transmission_hold = controller_reason in {
                    "takeup_active", "takeup_complete_replan",
                    "takeup_saturated_replan",
                    "replanning_after_takeup"}
                update = governor.update(
                    tip, started, external_hold=transmission_hold)
                errors.append(update.closest.distance_mm)
                pump()
                self._publish_trace(
                    path_id, sequence, update, config, tip, state,
                    path.length_m)
                goal_handle.publish_feedback(
                    self._feedback(
                        path_id, update, config, state, path.length_m))
                if update.complete:
                    goal_handle.succeed()
                    return self._result(True, "path complete", errors, update)
                if update.hard_error or update.timed_out:
                    goal_handle.abort()
                    reason = (
                        "hard path error" if update.hard_error else "timeout")
                    return self._result(False, reason, errors, update)
                time.sleep(max(0.0, period-(time.monotonic()-started)))
            goal_handle.abort()
            return self._result(False, "ROS shutdown", errors, update)
        finally:
            _, _, armed, _, _, _ = self._snapshot()
            if armed or auto_armed:
                ok, detail = self._set_armed(False)
                if not ok:
                    self.get_logger().error(f"path cleanup failed: {detail}")
            self._invalidate(path_id, sequence+1)
            with self._lock:
                self._goal_reserved = False

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TipPathActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
