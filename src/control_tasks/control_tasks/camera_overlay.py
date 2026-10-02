"""Low-rate camera overlay for measured markers and estimated catheter shape.

This node is intentionally outside the control process. It consumes decimated
JPEG previews and the already-published estimator trace, reconstructs the
manifest-selected distal centerline on CPU through the deployment API, and
drops visualization work whenever it falls behind.
"""
from __future__ import annotations

from collections import deque
import re

import cv2
from control_interface.msg import EstimatorStateTrace
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud
import torch


_FORMAT = re.compile(
    r"source=(?P<width>[0-9]+)x(?P<height>[0-9]+);\s*eye=(?P<eye>\w+)")
_MARKER_COLORS = (
    (255, 160, 0),
    (0, 220, 255),
    (80, 220, 80),
    (255, 80, 220),
)


def _stamp_ns(message) -> int:
    return (int(message.header.stamp.sec)*1_000_000_000
            + int(message.header.stamp.nanosec))


def nearest_sample(samples, timestamp_ns: int, maximum_skew_ns: int):
    """Return the nearest timestamped payload inside the display-only gate."""
    if not samples:
        return None
    best = min(samples, key=lambda item: abs(item[0]-timestamp_ns))
    return best if abs(best[0]-timestamp_ns) <= maximum_skew_ns else None


def _preview_geometry(message: CompressedImage, decoded: np.ndarray):
    match = _FORMAT.search(str(message.format))
    if match is None:
        return decoded.shape[1], decoded.shape[0], "left"
    return (int(match.group("width")), int(match.group("height")),
            match.group("eye"))


def _project(project_points, registration, points_base_mm, eye):
    transform = (registration.right_camera_T_base
                 if eye == "right" else registration.left_camera_T_base)
    return project_points(registration.K, transform, points_base_mm)


def draw_camera_overlay(image: np.ndarray, registration, project_points,
                        source_size, eye: str,
                        measured_markers_mm=None, centerline_mm=None,
                        target_path_mm=None,
                        estimator_health="UNAVAILABLE",
                        marker_skew_ms=None, estimator_skew_ms=None,
                        marker_crosshair_size=10,
                        marker_crosshair_line_width=1,
                        target_path_line_width=1):
    """Draw base-frame observations in a decimated camera image."""
    output = np.asarray(image).copy()
    source_width, source_height = source_size
    panels = ("left", "right") if eye == "stereo" else (eye,)
    scale_x = output.shape[1]/float(
        source_width*(2 if eye == "stereo" else 1))
    scale_y = output.shape[0]/float(source_height)
    panel_width = output.shape[1]/len(panels)

    def pixel_coordinates(points_mm, panel_eye, offset):
        pixels, visible = _project(
            project_points, registration, points_mm, panel_eye)
        pixels[:, 0] = pixels[:, 0]*scale_x+offset
        pixels[:, 1] *= scale_y
        visible &= np.isfinite(pixels).all(axis=1)
        visible &= pixels[:, 0] >= offset
        visible &= pixels[:, 0] < offset+panel_width
        visible &= pixels[:, 1] >= 0
        visible &= pixels[:, 1] < output.shape[0]
        return pixels, visible

    for panel_index, panel_eye in enumerate(panels):
        offset = panel_index*panel_width
        cv2.putText(
            output, panel_eye.upper(), (int(offset)+12, 26),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
            cv2.LINE_AA)
        if target_path_mm is not None:
            pixels, visible = pixel_coordinates(
                np.asarray(target_path_mm), panel_eye, offset)
            for first, second, first_ok, second_ok in zip(
                    pixels[:-1], pixels[1:], visible[:-1], visible[1:]):
                if first_ok and second_ok:
                    cv2.line(
                        output, tuple(np.rint(first).astype(int)),
                        tuple(np.rint(second).astype(int)),
                        (0, 255, 255), target_path_line_width, cv2.LINE_AA)
        if centerline_mm is not None:
            pixels, visible = pixel_coordinates(
                np.asarray(centerline_mm), panel_eye, offset)
            for first, second, first_ok, second_ok in zip(
                    pixels[:-1], pixels[1:], visible[:-1], visible[1:]):
                if first_ok and second_ok:
                    cv2.line(
                        output, tuple(np.rint(first).astype(int)),
                        tuple(np.rint(second).astype(int)),
                        (255, 0, 255), 3, cv2.LINE_AA)
        if measured_markers_mm is not None:
            pixels, visible = pixel_coordinates(
                np.asarray(measured_markers_mm), panel_eye, offset)
            for marker_id, (pixel, valid) in enumerate(zip(pixels, visible)):
                if not valid:
                    continue
                point = tuple(np.rint(pixel).astype(int))
                color = _MARKER_COLORS[marker_id % len(_MARKER_COLORS)]
                cv2.drawMarker(
                    output, point, color, cv2.MARKER_CROSS,
                    marker_crosshair_size, marker_crosshair_line_width,
                    cv2.LINE_AA)
                cv2.putText(
                    output, f"M{marker_id}", (point[0]+6, point[1]-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    details = [f"UKF/model: {estimator_health}"]
    if estimator_skew_ms is not None:
        details.append(f"shape dt={estimator_skew_ms:+.1f}ms")
    if marker_skew_ms is not None:
        details.append(f"markers dt={marker_skew_ms:+.1f}ms")
    cv2.putText(
        output, " | ".join(details), (12, output.shape[0]-14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1,
        cv2.LINE_AA)
    return output


class CameraOverlayNode(Node):
    """Render decimated diagnostic overlays without entering control paths."""

    def __init__(self):
        super().__init__("catheter_camera_overlay")
        self.declare_parameter("model_manifest", "")
        self.declare_parameter("registration_file", "")
        self.declare_parameter("rig_ids", ["primary", "oblique"])
        self.declare_parameter("preview_eye", "left")
        self.declare_parameter("preview_topic_prefix", "/shape_tracking/preview")
        self.declare_parameter("output_topic_prefix", "/catheter_mppi/camera_overlay")
        self.declare_parameter("marker_topic", "/shape_tracking/markers")
        self.declare_parameter(
            "target_path_topic", "/catheter_mppi/reference_path")
        self.declare_parameter(
            "estimator_trace_topic", "/catheter_mppi/estimator_trace")
        self.declare_parameter("display_rate_hz", 5.0)
        self.declare_parameter("maximum_alignment_skew_ms", 100.0)
        self.declare_parameter("output_jpeg_quality", 75)
        self.declare_parameter("marker_crosshair_size", 10)
        self.declare_parameter("marker_crosshair_line_width", 1)
        self.declare_parameter("target_path_line_width", 1)
        self.declare_parameter("show_window", True)

        from cr_meta_lnn.deployment import load_runtime_bundle
        from shape_tracking.session import (
            load_session_registration, project_points)
        self.project_points = project_points
        registration_file = str(
            self.get_parameter("registration_file").value)
        if not registration_file:
            raise ValueError("registration_file is required")
        self.rig_ids = tuple(
            str(value) for value in self.get_parameter("rig_ids").value)
        self.eye = str(self.get_parameter("preview_eye").value)
        if not self.rig_ids or self.eye not in {"left", "right", "stereo"}:
            raise ValueError("invalid overlay rigs or preview eye")
        self.registrations = {
            rig: load_session_registration(
                registration_file, require_em=False, rig_id=rig)
            for rig in self.rig_ids
        }
        torch.set_num_threads(1)
        self.runtime_bundle = load_runtime_bundle(
            self.get_parameter("model_manifest").value,
            device="cpu", options={"adaptation_enabled": False})
        self.runtime = self.runtime_bundle.runtime

        rate = float(self.get_parameter("display_rate_hz").value)
        self.maximum_skew_ns = int(round(
            float(self.get_parameter("maximum_alignment_skew_ms").value)*1e6))
        self.jpeg_quality = int(
            self.get_parameter("output_jpeg_quality").value)
        self.marker_crosshair_size = int(
            self.get_parameter("marker_crosshair_size").value)
        self.marker_crosshair_line_width = int(
            self.get_parameter("marker_crosshair_line_width").value)
        self.target_path_line_width = int(
            self.get_parameter("target_path_line_width").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        if (rate <= 0.0 or self.maximum_skew_ns < 0
                or not 1 <= self.jpeg_quality <= 100
                or self.marker_crosshair_size < 1
                or self.marker_crosshair_line_width < 1
                or self.target_path_line_width < 1):
            raise ValueError("invalid overlay timing or JPEG configuration")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        preview_prefix = str(
            self.get_parameter("preview_topic_prefix").value).rstrip("/")
        output_prefix = str(
            self.get_parameter("output_topic_prefix").value).rstrip("/")
        self.images = {rig: None for rig in self.rig_ids}
        self.last_rendered_stamp = {rig: None for rig in self.rig_ids}
        self.shapes = deque(maxlen=64)
        self.markers = deque(maxlen=64)
        self.target_path_mm = None
        self.output_publishers = {}
        self.image_subscriptions = []
        for rig in self.rig_ids:
            self.image_subscriptions.append(self.create_subscription(
                CompressedImage,
                f"{preview_prefix}/{rig}/compressed",
                lambda message, selected=rig: self._image_cb(
                    selected, message), qos))
            self.output_publishers[rig] = self.create_publisher(
                CompressedImage, f"{output_prefix}/{rig}/compressed", qos)
        self.create_subscription(
            PointCloud, str(self.get_parameter("marker_topic").value),
            self._marker_cb, qos)
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PointCloud, str(self.get_parameter("target_path_topic").value),
            self._target_path_cb, path_qos)
        self.create_subscription(
            EstimatorStateTrace,
            str(self.get_parameter("estimator_trace_topic").value),
            self._estimator_cb, qos)
        self.timer = self.create_timer(1.0/rate, self._render)
        self.get_logger().info(
            f"diagnostic camera overlay ready at {rate:g} Hz; "
            f"rigs={list(self.rig_ids)}, eye={self.eye}, CPU-only model")

    def _image_cb(self, rig_id, message):
        self.images[rig_id] = message

    def _marker_cb(self, message):
        if not message.points:
            return
        points = np.asarray(
            [[point.x, point.y, point.z] for point in message.points],
            dtype=np.float64)*1e3
        self.markers.append((_stamp_ns(message), points))

    def _estimator_cb(self, message):
        pose = np.asarray(message.interface_pose, dtype=np.float32)
        strain = np.asarray(message.distal_strain, dtype=np.float32)
        if (pose.shape != (16,) or strain.size == 0
                or not np.isfinite(pose).all()
                or not np.isfinite(strain).all()):
            return
        self.shapes.append((
            int(message.observation_timestamp_ns), pose.reshape(4, 4),
            strain.copy(), str(message.estimator_health)))

    def _target_path_cb(self, message):
        points = np.asarray(
            [[point.x, point.y, point.z] for point in message.points],
            dtype=np.float64)
        self.target_path_mm = (
            None if not len(points) or not np.isfinite(points).all()
            else points*1e3)

    def _centerline(self, sample):
        _, pose, strain, _ = sample
        points = self.runtime.centerline_from_estimate(pose, strain)
        return points.detach().cpu().numpy()*1e3

    def _render(self):
        for rig_id, message in self.images.items():
            if message is None:
                continue
            publish_output = (
                self.output_publishers[rig_id].get_subscription_count() > 0)
            if not self.show_window and not publish_output:
                continue
            timestamp_ns = _stamp_ns(message)
            if self.last_rendered_stamp[rig_id] == timestamp_ns:
                continue
            self.last_rendered_stamp[rig_id] = timestamp_ns
            encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                continue
            source_width, source_height, eye = _preview_geometry(
                message, image)
            shape = nearest_sample(
                self.shapes, timestamp_ns, self.maximum_skew_ns)
            markers = nearest_sample(
                self.markers, timestamp_ns, self.maximum_skew_ns)
            centerline = None if shape is None else self._centerline(shape)
            output = draw_camera_overlay(
                image, self.registrations[rig_id], self.project_points,
                (source_width, source_height), eye,
                measured_markers_mm=(None if markers is None else markers[1]),
                centerline_mm=centerline,
                target_path_mm=self.target_path_mm,
                estimator_health=("UNAVAILABLE" if shape is None else shape[3]),
                marker_skew_ms=(None if markers is None else
                                (markers[0]-timestamp_ns)*1e-6),
                estimator_skew_ms=(None if shape is None else
                                   (shape[0]-timestamp_ns)*1e-6),
                marker_crosshair_size=self.marker_crosshair_size,
                marker_crosshair_line_width=(
                    self.marker_crosshair_line_width),
                target_path_line_width=self.target_path_line_width)
            if publish_output:
                success, payload = cv2.imencode(
                    ".jpg", output,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if success:
                    published = CompressedImage()
                    published.header = message.header
                    published.format = "jpeg"
                    published.data = payload.tobytes()
                    self.output_publishers[rig_id].publish(published)
            if self.show_window:
                cv2.imshow(f"Catheter overlay - {rig_id}", output)
        if self.show_window:
            cv2.waitKey(1)

    def close(self):
        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraOverlayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
