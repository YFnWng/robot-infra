"""RViz marker adapter for the isolated model-in-the-loop graph."""
from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticArray
from control_interface.msg import EstimatorStateTrace, TipReferenceHorizon
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
    ReliabilityPolicy)
from sensor_msgs.msg import PointCloud
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _point(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _base_axis_specs(length_m=0.025):
    """Return conventional RViz XYZ endpoints and colors."""
    length = float(length_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("base-axis length must be finite and positive")
    return (
        ("X", _point(length, 0.0, 0.0), _color(1.0, 0.05, 0.05)),
        ("Y", _point(0.0, length, 0.0), _color(0.05, 1.0, 0.05)),
        ("Z", _point(0.0, 0.0, length), _color(0.1, 0.35, 1.0)),
    )


def _pose_axis_endpoints(pose, length_m):
    """Return an interface origin and ambient endpoints of material XYZ."""
    transform = np.asarray(pose, dtype=float)
    length = float(length_m)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not math.isfinite(length) or length <= 0.0):
        raise ValueError("pose must be finite 4x4 and length must be positive")
    origin = transform[:3, 3]
    endpoints = origin[:, None]+length*transform[:3, :3]
    return (_point(*origin), tuple(_point(*endpoints[:, axis])
                                   for axis in range(3)))


class SimulationVisualizer(Node):
    def __init__(self):
        super().__init__("catheter_sim_visualizer")
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("controller_label", "MPPI")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.controller_label = str(
            self.get_parameter("controller_label").value)
        self.markers = None
        self.tip = None
        self.target = None
        self.prediction = None
        self.reference_path = None
        self.reference_preview = None
        self.estimated_interface_pose = None
        self.truth_interface_pose = None
        self.controller_state = "unknown"
        self.create_subscription(
            PointCloud, "/sim/catheter_sim/ground_truth_markers",
            self._markers_cb, 10)
        self.create_subscription(
            PointStamped, "/sim/catheter_sim/ground_truth_tip",
            self._tip_cb, 10)
        self.create_subscription(
            PointStamped, "/sim/catheter_mppi/target_tip",
            self._target_cb, 10)
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PointCloud, "/sim/catheter_mppi/reference_path",
            self._path_cb, path_qos)
        self.create_subscription(
            PointStamped, "/sim/catheter_mppi/path_reference_point",
            self._target_cb, 10)
        self.create_subscription(
            TipReferenceHorizon,
            "/sim/catheter_mppi/reference_horizon",
            self._preview_cb, 10)
        self.create_subscription(
            PointCloud, "/sim/catheter_mppi/predicted_tip",
            self._prediction_cb, 10)
        self.create_subscription(
            DiagnosticArray, "/sim/catheter_mppi/status",
            self._status_cb, 10)
        self.create_subscription(
            EstimatorStateTrace, "/sim/catheter_mppi/estimator_trace",
            self._estimator_trace_cb, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped,
            "/sim/catheter_sim/ground_truth_interface_pose",
            self._truth_interface_pose_cb, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/sim/catheter_sim/visualization", 10)
        self.error_pub = self.create_publisher(
            Vector3Stamped, "/sim/catheter_sim/tip_error_mm", 10)
        self.timer = self.create_timer(0.05, self._publish)

    def _markers_cb(self, message):
        if (message.header.frame_id == self.frame_id
                and len(message.points) == 4):
            self.markers = list(message.points)

    def _tip_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.tip = message.point

    def _target_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.target = message.point

    def _prediction_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.prediction = list(message.points)

    def _path_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.reference_path = list(message.points)

    def _preview_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.reference_preview = list(message.positions)

    def _status_cb(self, message):
        for status in message.status:
            if status.name == "catheter_control/mppi":
                self.controller_state = status.message
                return

    @staticmethod
    def _matrix_from_pose(message):
        quaternion = np.asarray([
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w], dtype=float)
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-12:
            return None
        x, y, z, w = quaternion/norm
        rotation = np.asarray([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = [
            message.position.x, message.position.y, message.position.z]
        return transform if np.isfinite(transform).all() else None

    def _estimator_trace_cb(self, message):
        pose = np.asarray(message.interface_pose, dtype=float)
        if pose.shape == (16,) and np.isfinite(pose).all():
            self.estimated_interface_pose = pose.reshape(4, 4)

    def _truth_interface_pose_cb(self, message):
        if message.header.frame_id == self.frame_id:
            self.truth_interface_pose = self._matrix_from_pose(message.pose)

    def _base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "catheter_mppi_sim"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _interface_frame_markers(
            self, pose, marker_id, namespace, label, length_m, width_m,
            alpha):
        if pose is None:
            return []
        origin, endpoints = _pose_axis_endpoints(pose, length_m)
        colors = (
            _color(1.0, 0.05, 0.05, alpha),
            _color(0.05, 1.0, 0.05, alpha),
            _color(0.1, 0.35, 1.0, alpha),
        )
        output = []
        for axis_index, (endpoint, color) in enumerate(
                zip(endpoints, colors)):
            axis = self._base_marker(marker_id+axis_index, Marker.ARROW)
            axis.ns = namespace
            axis.scale.x = width_m
            axis.scale.y = 2.2*width_m
            axis.scale.z = 3.0*width_m
            axis.color = color
            axis.points = [origin, endpoint]
            output.append(axis)
        text = self._base_marker(marker_id+3, Marker.TEXT_VIEW_FACING)
        text.ns = namespace
        text.pose.position = _point(origin.x, origin.y, origin.z+0.006)
        text.scale.z = 0.0035
        text.color = _color(1.0, 1.0, 1.0, alpha)
        text.text = label
        output.append(text)
        return output

    def _publish(self):
        output = []
        origin = _point(0.0, 0.0, 0.0)
        for index, (label, endpoint, color) in enumerate(_base_axis_specs()):
            axis = self._base_marker(10+index, Marker.ARROW)
            axis.ns = "robot_base_axes"
            axis.scale.x = 0.0015
            axis.scale.y = 0.0035
            axis.scale.z = 0.0045
            axis.color = color
            axis.points = [origin, endpoint]
            output.append(axis)

            axis_label = self._base_marker(
                13+index, Marker.TEXT_VIEW_FACING)
            axis_label.ns = "robot_base_axes"
            axis_label.pose.position = endpoint
            axis_label.scale.z = 0.005
            axis_label.color = color
            axis_label.text = f"+{label}"
            output.append(axis_label)

        base_label = self._base_marker(16, Marker.TEXT_VIEW_FACING)
        base_label.ns = "robot_base_axes"
        base_label.pose.position = _point(0.0, 0.0, -0.006)
        base_label.scale.z = 0.004
        base_label.color = _color(0.9, 0.9, 0.9)
        base_label.text = "robot_base"
        output.append(base_label)

        # The shorter/thicker truth triad remains visible underneath a
        # coincident longer/thinner UKF triad. Axes are material-frame XYZ,
        # expressed in robot_base; in particular Z is the interface tangent.
        output.extend(self._interface_frame_markers(
            self.truth_interface_pose, 20, "truth_interface_frame",
            "truth interface", 0.018, 0.0018, 0.55))
        output.extend(self._interface_frame_markers(
            self.estimated_interface_pose, 30, "ukf_interface_frame",
            "UKF interface", 0.028, 0.0010, 1.0))

        if self.markers is not None:
            line = self._base_marker(0, Marker.LINE_STRIP)
            line.scale.x = 0.0015
            line.color = _color(0.15, 0.75, 1.0)
            line.points = [_point(point.x, point.y, point.z)
                           for point in self.markers]
            output.append(line)

            spheres = self._base_marker(1, Marker.SPHERE_LIST)
            spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.004
            spheres.color = _color(0.9, 0.2, 0.15)
            spheres.points = line.points
            output.append(spheres)

        if self.tip is not None:
            tip = self._base_marker(2, Marker.SPHERE)
            tip.pose.position = _point(self.tip.x, self.tip.y, self.tip.z)
            tip.scale.x = tip.scale.y = tip.scale.z = 0.005
            tip.color = _color(0.1, 1.0, 0.25)
            output.append(tip)

        if self.reference_path:
            path = self._base_marker(7, Marker.LINE_STRIP)
            path.ns = "continuous_reference_path"
            path.scale.x = 0.0008
            path.color = _color(1.0, 0.55, 0.05, 0.70)
            path.points = [_point(point.x, point.y, point.z)
                           for point in self.reference_path]
            output.append(path)

        if self.reference_preview:
            preview = self._base_marker(8, Marker.LINE_STRIP)
            preview.ns = "mppi_reference_preview"
            preview.scale.x = 0.0014
            preview.color = _color(1.0, 1.0, 0.15, 0.95)
            preview.points = [
                _point(point.x, point.y, point.z)
                for point in self.reference_preview]
            output.append(preview)

        if self.target is not None:
            target = self._base_marker(3, Marker.SPHERE)
            target.pose.position = _point(
                self.target.x, self.target.y, self.target.z)
            target.scale.x = target.scale.y = target.scale.z = 0.006
            target.color = _color(1.0, 0.75, 0.05, 0.85)
            output.append(target)

        error_norm_mm = None
        if self.tip is not None and self.target is not None:
            dx = self.target.x-self.tip.x
            dy = self.target.y-self.tip.y
            dz = self.target.z-self.tip.z
            error_norm_mm = 1000.0*math.sqrt(dx*dx+dy*dy+dz*dz)
            arrow = self._base_marker(4, Marker.ARROW)
            arrow.scale.x = 0.0012
            arrow.scale.y = 0.0025
            arrow.scale.z = 0.0035
            ratio = min(1.0, error_norm_mm/5.0)
            arrow.color = _color(ratio, 1.0-ratio, 0.1)
            arrow.points = [
                _point(self.tip.x, self.tip.y, self.tip.z),
                _point(self.target.x, self.target.y, self.target.z)]
            output.append(arrow)

            error = Vector3Stamped()
            error.header = arrow.header
            error.vector.x = 1000.0*dx
            error.vector.y = 1000.0*dy
            error.vector.z = 1000.0*dz
            self.error_pub.publish(error)

        if self.prediction:
            predicted = self._base_marker(5, Marker.LINE_STRIP)
            predicted.scale.x = 0.001
            predicted.color = _color(0.75, 0.25, 1.0)
            if self.tip is not None:
                predicted.points.append(
                    _point(self.tip.x, self.tip.y, self.tip.z))
            predicted.points.extend(
                _point(point.x, point.y, point.z)
                for point in self.prediction)
            output.append(predicted)

        text = self._base_marker(6, Marker.TEXT_VIEW_FACING)
        anchor = self.target if self.target is not None else self.tip
        if anchor is not None:
            text.pose.position = _point(anchor.x, anchor.y, anchor.z+0.012)
        else:
            text.pose.position = _point(0.0, 0.0, 0.08)
        text.scale.z = 0.005
        text.color = _color(1.0, 1.0, 1.0)
        error_text = ("no target" if error_norm_mm is None
                      else f"error {error_norm_mm:.3f} mm")
        text.text = (
            f"{self.controller_label} | {self.controller_state} | {error_text}")
        output.append(text)

        message = MarkerArray()
        message.markers = output
        self.marker_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationVisualizer()
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
