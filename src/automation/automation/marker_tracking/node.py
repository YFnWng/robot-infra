"""ROS 2 publisher for calibrated four-ring catheter feedback."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, PointCloud
import yaml


CAMERA_SETTING_KEYS = (
    "resolution", "fps", "sharpness", "exposure", "gain",
    "white_balance_temperature", "white_balance_auto_freeze_s",
    "white_balance_auto_freeze_retries", "brightness", "contrast", "hue",
    "saturation", "gamma",
)


def _load_shape_tracking(shape_tracking_root: str):
    """Load camera/marker code from an installed package or source tree."""
    if shape_tracking_root:
        root = Path(shape_tracking_root).expanduser().resolve()
        source = root / "src"
        import_path = source if source.is_dir() else root
        if str(import_path) not in sys.path:
            sys.path.insert(0, str(import_path))
    try:
        from shape_tracking.multi_camera import CameraCaptureWorker
        from shape_tracking.online_markers import (
            MarkerEstimate,
            MarkerTrackingFailure,
            StereoRingTracker,
            fuse_marker_estimates,
        )
        from shape_tracking.session import load_session_registration
        from shape_tracking.zed_capture import ZedCamera
    except ImportError as exc:
        location = shape_tracking_root or "the active Python environment"
        raise ImportError(
            "could not import the shape_tracking camera/marker library from "
            f"{location}; set shape_tracking_root to the repository root. "
            f"Underlying import error: {exc}"
        ) from exc
    return {
        "CameraCaptureWorker": CameraCaptureWorker,
        "MarkerEstimate": MarkerEstimate,
        "MarkerTrackingFailure": MarkerTrackingFailure,
        "StereoRingTracker": StereoRingTracker,
        "fuse_marker_estimates": fuse_marker_estimates,
        "load_session_registration": load_session_registration,
        "ZedCamera": ZedCamera,
    }


def load_live_camera_config(path: str | Path) -> tuple[dict, dict[str, int]]:
    """Return shared camera settings and stable rig serial numbers."""
    config_path = Path(path).expanduser().resolve()
    with config_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    common = dict(document.get("camera") or {})
    recording = dict(document.get("recording") or {})
    if "svo_compression" in recording:
        common["svo_compression"] = recording["svo_compression"]
    cameras = document.get("cameras") or {}
    serials = {}
    for rig_id, item in cameras.items():
        if not isinstance(item, dict) or "serial" not in item:
            raise ValueError(f"camera rig {rig_id!r} has no serial number")
        serials[str(rig_id)] = int(item["serial"])
    if not serials:
        raise ValueError(f"no camera rigs are configured in {config_path}")
    return common, serials


def marker_point_cloud(estimate, frame_id: str) -> PointCloud:
    """Convert a four-marker estimate from millimetres to a ROS point cloud."""
    message = PointCloud()
    message.header.frame_id = str(frame_id)
    message.header.stamp.sec = int(
        estimate.timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(
        estimate.timestamp_ns % 1_000_000_000)
    message.points = [
        Point32(x=float(point[0] * 1e-3),
                y=float(point[1] * 1e-3),
                z=float(point[2] * 1e-3))
        for point in estimate.points_base_mm
    ]
    message.channels = [
        ChannelFloat32(
            name="marker_id", values=[0.0, 1.0, 2.0, 3.0]),
        ChannelFloat32(
            name="confidence",
            values=[float(value) for value in estimate.confidence]),
        ChannelFloat32(
            name="reprojection_error_px",
            values=[float(value)
                    for value in estimate.reprojection_error_px]),
        ChannelFloat32(
            name="source_rig_count",
            values=[float(value) for value in estimate.source_count]),
    ]
    return message


class MarkerTrackingNode(Node):
    """Acquire ZED images and publish fresh, quality-gated marker positions."""

    def __init__(self):
        super().__init__("marker_tracking")
        self.declare_parameter(
            "shape_tracking_root",
            os.environ.get("SHAPE_TRACKING_ROOT", ""))
        self.declare_parameter("camera_config", "")
        self.declare_parameter("registration_file", "")
        self.declare_parameter("rig_ids", ["primary", "oblique"])
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("marker_topic", "/shape_tracking/markers")
        self.declare_parameter(
            "diagnostics_topic", "/shape_tracking/marker_status")
        self.declare_parameter("process_rate_hz", 30.0)
        self.declare_parameter("minimum_confidence", 0.20)
        self.declare_parameter("maximum_epipolar_error_px", 10.0)
        self.declare_parameter("maximum_reprojection_error_px", 3.0)
        self.declare_parameter("maximum_cross_rig_disagreement_mm", 5.0)
        self.declare_parameter("maximum_rig_timestamp_skew_ms", 25.0)
        self.declare_parameter("minimum_valid_rigs", 1)
        self.declare_parameter("feedback_timeout_s", 0.5)

        shape_root = str(self.get_parameter("shape_tracking_root").value)
        self.vision = _load_shape_tracking(shape_root)
        camera_config = str(self.get_parameter("camera_config").value)
        registration_file = str(self.get_parameter("registration_file").value)
        if not camera_config:
            raise ValueError("ROS parameter camera_config is required")
        if not registration_file:
            raise ValueError("ROS parameter registration_file is required")
        requested_rigs = tuple(
            str(value) for value in self.get_parameter("rig_ids").value)
        if not requested_rigs:
            raise ValueError("rig_ids must contain at least one camera rig")

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.maximum_cross_rig_disagreement_mm = float(
            self.get_parameter("maximum_cross_rig_disagreement_mm").value)
        self.maximum_rig_timestamp_skew_ns = int(round(
            float(self.get_parameter("maximum_rig_timestamp_skew_ms").value)
            * 1e6))
        self.minimum_valid_rigs = int(
            self.get_parameter("minimum_valid_rigs").value)
        if not 1 <= self.minimum_valid_rigs <= len(requested_rigs):
            raise ValueError(
                "minimum_valid_rigs must be between 1 and rig count")
        self.feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value)

        common, serials = load_live_camera_config(camera_config)
        missing = [rig for rig in requested_rigs if rig not in serials]
        if missing:
            raise ValueError(
                f"rigs {missing} are absent from camera config; "
                f"available={sorted(serials)}")

        self.marker_publisher = self.create_publisher(
            PointCloud, str(self.get_parameter("marker_topic").value),
            qos_profile_sensor_data)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value), 10)
        self.cameras = {}
        self.workers = {}
        self.trackers = {}
        self.last_sequences = {rig: 0 for rig in requested_rigs}
        self.processing_pool = ThreadPoolExecutor(
            max_workers=2 * len(requested_rigs),
            thread_name_prefix="marker-eye")
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.last_valid_monotonic = None
        self.last_status_monotonic = 0.0
        self.closed = False

        try:
            for rig_id in requested_rigs:
                registration = self.vision["load_session_registration"](
                    registration_file, require_em=False, rig_id=rig_id)
                kwargs = {
                    key: common[key] for key in CAMERA_SETTING_KEYS
                    if key in common
                }
                if "svo_compression" in common:
                    kwargs["svo_compression"] = common["svo_compression"]
                kwargs["serial_number"] = serials[rig_id]
                camera = self.vision["ZedCamera"](**kwargs)
                camera.open()
                self.cameras[rig_id] = camera
                info = camera.info()
                if (registration.zed_serial
                        and str(info["serial"]) != registration.zed_serial):
                    raise ValueError(
                        f"{rig_id} registration belongs to ZED "
                        f"{registration.zed_serial}, opened {info['serial']}")
                if (registration.resolution
                        and str(info["resolution"])
                        != registration.resolution):
                    raise ValueError(
                        f"{rig_id} registration resolution is "
                        f"{registration.resolution}, opened "
                        f"{info['resolution']}")
                worker = self.vision["CameraCaptureWorker"](
                    rig_id, camera, preview_stride=1)
                tracker = self.vision["StereoRingTracker"](
                    rig_id, registration,
                    minimum_confidence=float(
                        self.get_parameter("minimum_confidence").value),
                    maximum_epipolar_error_px=float(
                        self.get_parameter(
                            "maximum_epipolar_error_px").value),
                    maximum_reprojection_error_px=float(
                        self.get_parameter(
                            "maximum_reprojection_error_px").value),
                    executor=self.processing_pool,
                )
                self.workers[rig_id] = worker
                self.trackers[rig_id] = tracker
            for worker in self.workers.values():
                worker.start()
        except Exception:
            self.close()
            raise

        rate_hz = float(self.get_parameter("process_rate_hz").value)
        if rate_hz <= 0:
            self.close()
            raise ValueError("process_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate_hz, self._process_latest)
        self.watchdog_timer = self.create_timer(0.25, self._watchdog)
        self.get_logger().info(
            f"marker tracking ready: rigs={list(requested_rigs)}, "
            f"output={self.marker_publisher.topic_name}, "
            f"frame={self.frame_id}; "
            "marker-only processing, complete fresh sets only")
        self._publish_diagnostic(
            DiagnosticStatus.OK, "INITIALIZED",
            {"rigs": ",".join(requested_rigs)})

    def _process_latest(self):
        started = time.perf_counter()
        frames = {}
        camera_errors = []
        for rig_id, worker in self.workers.items():
            if worker.error is not None:
                camera_errors.append(f"{rig_id}:{worker.error}")
                continue
            frame = worker.latest()
            if frame is None or frame.sequence <= self.last_sequences[rig_id]:
                continue
            frames[rig_id] = frame
            self.last_sequences[rig_id] = frame.sequence
        if not frames:
            if camera_errors:
                self._reject(
                    "camera_failed", ";".join(camera_errors),
                    DiagnosticStatus.ERROR)
            return

        estimates = []
        failures = list(camera_errors)
        estimate_type = self.vision["MarkerEstimate"]
        for rig_id, frame in frames.items():
            result = self.trackers[rig_id].process(
                frame.timestamp_ns, frame.left_bgr, frame.right_bgr)
            if isinstance(result, estimate_type):
                estimates.append(result)
            else:
                failures.append(f"{rig_id}:{result.reason}:{result.detail}")
        if len(estimates) < self.minimum_valid_rigs:
            self._reject("insufficient_valid_rigs", ";".join(failures))
            return

        timestamp_span_ns = (
            max(item.timestamp_ns for item in estimates)
            - min(item.timestamp_ns for item in estimates))
        if (len(estimates) > 1
                and timestamp_span_ns > self.maximum_rig_timestamp_skew_ns):
            newest = max(estimates, key=lambda item: item.timestamp_ns)
            if self.minimum_valid_rigs > 1:
                self._reject(
                    "camera_timestamp_skew",
                    f"{timestamp_span_ns * 1e-6:.3f}ms")
                return
            estimates = [newest]

        fused = self.vision["fuse_marker_estimates"](
            estimates, self.maximum_cross_rig_disagreement_mm)
        if not isinstance(fused, estimate_type):
            self._reject(fused.reason, fused.detail)
            return
        self.marker_publisher.publish(self._point_cloud(fused))
        self.accepted_frames += 1
        self.last_valid_monotonic = time.monotonic()
        latency_ms = (time.perf_counter() - started) * 1000.0
        self._publish_diagnostic(
            DiagnosticStatus.OK, "TRACKING",
            {
                "image_timestamp_ns": str(fused.timestamp_ns),
                "rigs": ",".join(fused.rig_ids),
                "processing_latency_ms": f"{latency_ms:.3f}",
                "maximum_reprojection_error_px": (
                    f"{float(np.max(fused.reprojection_error_px)):.3f}"),
                "accepted_frames": str(self.accepted_frames),
                "rejected_frames": str(self.rejected_frames),
            })

    def _point_cloud(self, estimate) -> PointCloud:
        return marker_point_cloud(estimate, self.frame_id)

    def _reject(
            self, reason: str, detail: str,
            level: int = DiagnosticStatus.WARN):
        self.rejected_frames += 1
        self._publish_diagnostic(
            level, reason.upper(),
            {
                "detail": detail,
                "accepted_frames": str(self.accepted_frames),
                "rejected_frames": str(self.rejected_frames),
            })

    def _watchdog(self):
        now = time.monotonic()
        if (self.last_valid_monotonic is not None
                and now - self.last_valid_monotonic
                <= self.feedback_timeout_s):
            return
        if now - self.last_status_monotonic < self.feedback_timeout_s:
            return
        age = "never" if self.last_valid_monotonic is None else (
            f"{now - self.last_valid_monotonic:.3f}")
        self._publish_diagnostic(
            DiagnosticStatus.ERROR, "FEEDBACK_STALE",
            {"age_s": age})

    def _publish_diagnostic(self, level: int, message: str, values: dict):
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        # ROS distributions expose uint8 constants as either int or one-byte
        # values; assigning the constant directly matches that distribution's
        # generated field setter.
        status.level = level
        status.name = "automation/four_ring_markers"
        status.hardware_id = ",".join(self.cameras) or "zed"
        status.message = str(message)
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        report.status = [status]
        self.diagnostic_publisher.publish(report)
        self.last_status_monotonic = time.monotonic()

    def close(self):
        if self.closed:
            return
        self.closed = True
        worker_rigs = set(self.workers)
        for worker in self.workers.values():
            try:
                worker.close()
            except Exception as exc:
                self.get_logger().error(f"camera shutdown failed: {exc}")
        self.workers.clear()
        for rig_id, camera in self.cameras.items():
            if rig_id in worker_rigs:
                continue
            try:
                camera.close()
            except Exception:
                pass
        self.cameras.clear()
        self.processing_pool.shutdown(wait=True, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerTrackingNode()
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
