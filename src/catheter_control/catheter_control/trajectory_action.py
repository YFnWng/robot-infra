"""ROS action server for time-budgeted catheter tip trajectories."""
from __future__ import annotations

import math
import threading
import time

from control_interface.action import TrackTipTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_srvs.srv import SetBool

from .target_offset import controller_state, measured_tip
from .trajectory import TrajectorySequencer


class TipTrajectoryActionServer(Node):
    """Sequence MPPI targets without bypassing its guarded arm service."""

    def __init__(self):
        super().__init__("catheter_tip_trajectory")
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("maximum_waypoints", 100)
        self.declare_parameter("marker_topic", "/shape_tracking/markers")
        self.declare_parameter("status_topic", "/catheter_mppi/status")
        self.declare_parameter("target_topic", "/catheter_mppi/target_tip")
        self.declare_parameter("arm_service", "/catheter_mppi/set_armed")
        self.declare_parameter(
            "action_name", "/catheter_mppi/track_tip_trajectory")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.update_rate_hz = float(
            self.get_parameter("update_rate_hz").value)
        self.feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value)
        self.service_timeout_s = float(
            self.get_parameter("service_timeout_s").value)
        self.maximum_waypoints = int(
            self.get_parameter("maximum_waypoints").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        target_topic = str(self.get_parameter("target_topic").value)
        arm_service = str(self.get_parameter("arm_service").value)
        action_name = str(self.get_parameter("action_name").value)
        if (not self.frame_id or self.update_rate_hz <= 0.0
                or self.feedback_timeout_s <= 0.0
                or self.service_timeout_s <= 0.0
                or self.maximum_waypoints < 1
                or not all((marker_topic, status_topic, target_topic,
                            arm_service, action_name))):
            raise ValueError("trajectory action parameters are invalid")

        self._lock = threading.Lock()
        self._goal_reserved = False
        self.tip = None
        self.tip_time = None
        self.controller_armed = False
        self.controller_state = None
        self.controller_time = None
        group = ReentrantCallbackGroup()
        self.target_pub = self.create_publisher(
            PointStamped, target_topic, 10)
        self.arm_client = self.create_client(
            SetBool, arm_service, callback_group=group)
        self.create_subscription(
            PointCloud, marker_topic, self._marker_cb,
            qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            DiagnosticArray, status_topic, self._status_cb, 10,
            callback_group=group)
        self.action_server = ActionServer(
            self, TrackTipTrajectory, action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
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
            armed, state = controller_state(message)
        except ValueError:
            return
        with self._lock:
            self.controller_armed = armed
            self.controller_state = state
            self.controller_time = time.monotonic()

    def _request_spec(self, request):
        if request.header.frame_id != self.frame_id:
            raise ValueError(
                f"goal frame must be {self.frame_id!r}")
        if len(request.waypoints) > self.maximum_waypoints:
            raise ValueError(
                f"goal exceeds maximum_waypoints={self.maximum_waypoints}")
        waypoints = [[point.x, point.y, point.z]
                     for point in request.waypoints]
        return TrajectorySequencer(
            waypoints, request.waypoint_timeouts_s,
            tolerance_mm=request.tolerance_mm,
            settle_time_s=request.settle_time_s)

    def _goal_callback(self, request):
        try:
            self._request_spec(request)
        except ValueError as error:
            self.get_logger().warn(f"rejected trajectory goal: {error}")
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warn(
                    "rejected trajectory goal: another goal is active")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _snapshot(self):
        with self._lock:
            tip = None if self.tip is None else self.tip.copy()
            return (tip, self.tip_time, self.controller_armed,
                    self.controller_state, self.controller_time)

    def _publish_target(self, target):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.point.x, message.point.y, message.point.z = target.tolist()
        self.target_pub.publish(message)

    def _set_armed(self, armed: bool):
        if not self.arm_client.wait_for_service(
                timeout_sec=self.service_timeout_s):
            return False, "controller arm service unavailable"
        request = SetBool.Request()
        request.data = bool(armed)
        future = self.arm_client.call_async(request)
        deadline = time.monotonic()+self.service_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or future.result() is None:
            return False, "controller arm service timed out"
        response = future.result()
        return bool(response.success), str(response.message)

    def _fresh_inputs(self, now):
        tip, tip_time, armed, state, status_time = self._snapshot()
        fresh = bool(
            tip is not None and tip_time is not None
            and status_time is not None
            and now-tip_time <= self.feedback_timeout_s
            and now-status_time <= self.feedback_timeout_s)
        return fresh, tip, armed, state

    def _wait_for_armed_status(self, goal_handle):
        """Wait for the arm transition to appear in fresh diagnostics."""
        deadline = time.monotonic()+self.service_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "trajectory canceled while arming"
            fresh, _, armed, state = self._fresh_inputs(time.monotonic())
            if fresh and armed:
                return True, state
            if fresh and state == "FAULTED":
                return False, "controller faulted while arming"
            time.sleep(0.01)
        return False, "controller did not report armed status"

    def _result(self, sequence, message, final_error=math.nan):
        result = TrackTipTrajectory.Result()
        # Time-budget expiry is an intended waypoint transition, not an
        # action-server failure. The separate reached/timed-out counts report
        # tracking quality; success reports completion of the full schedule.
        result.success = bool(sequence.complete)
        result.message = str(message)
        result.reached_waypoints = sequence.reached_waypoints
        result.timed_out_waypoints = sequence.timed_out_waypoints
        result.final_error_mm = float(final_error)
        return result

    def _execute(self, goal_handle):
        sequence = self._request_spec(goal_handle.request)
        auto_armed = False
        final_error = math.nan
        try:
            deadline = time.monotonic()+self.service_timeout_s
            while rclpy.ok() and time.monotonic() < deadline:
                fresh, _, armed, state = self._fresh_inputs(time.monotonic())
                if fresh and self.target_pub.get_subscription_count() > 0:
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(sequence, "canceled before start")
                time.sleep(0.02)
            else:
                goal_handle.abort()
                return self._result(
                    sequence, "fresh tip/controller feedback unavailable")

            self._publish_target(sequence.target)
            if goal_handle.request.auto_arm and not armed:
                ok, detail = self._set_armed(True)
                if not ok:
                    goal_handle.abort()
                    return self._result(sequence, f"arming failed: {detail}")
                auto_armed = True
                ok, detail = self._wait_for_armed_status(goal_handle)
                if not ok:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return self._result(sequence, detail)
            elif not armed:
                goal_handle.abort()
                return self._result(
                    sequence, "controller is not armed and auto_arm is false")

            started = time.monotonic()
            sequence.start(started)
            period = 1.0/self.update_rate_hz
            while rclpy.ok():
                loop_started = time.monotonic()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        sequence, "trajectory canceled", final_error)
                fresh, tip, armed, state = self._fresh_inputs(loop_started)
                if not fresh:
                    goal_handle.abort()
                    return self._result(
                        sequence, "tip/controller feedback became stale",
                        final_error)
                if state == "FAULTED":
                    goal_handle.abort()
                    return self._result(
                        sequence, "controller faulted", final_error)
                if not armed:
                    goal_handle.abort()
                    return self._result(
                        sequence, "controller disarmed during trajectory",
                        final_error)

                update = sequence.update(tip, loop_started)
                final_error = update.tracking_error_mm
                feedback = TrackTipTrajectory.Feedback()
                feedback.waypoint_index = update.waypoint_index
                feedback.waypoint_count = update.waypoint_count
                feedback.elapsed_s = update.elapsed_s
                feedback.remaining_s = update.remaining_s
                feedback.tracking_error_mm = update.tracking_error_mm
                feedback.within_tolerance = update.within_tolerance
                goal_handle.publish_feedback(feedback)
                if update.complete:
                    goal_handle.succeed()
                    message = (
                        f"trajectory complete: {update.reached_waypoints} "
                        f"reached, {update.timed_out_waypoints} timed out")
                    return self._result(sequence, message, final_error)
                if update.target_changed:
                    self._publish_target(sequence.target)
                time.sleep(max(0.0, period-(time.monotonic()-loop_started)))

            goal_handle.abort()
            return self._result(sequence, "ROS shutdown", final_error)
        finally:
            # A trajectory endpoint must not leave an armed controller running,
            # irrespective of whether this server performed the initial arm.
            _, _, armed, _, _ = self._snapshot()
            if armed or auto_armed:
                ok, detail = self._set_armed(False)
                if not ok:
                    self.get_logger().error(
                        f"trajectory cleanup could not disarm: {detail}")
            with self._lock:
                self._goal_reserved = False

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TipTrajectoryActionServer()
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
