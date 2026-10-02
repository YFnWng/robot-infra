"""ROS 2 publisher for calibrated four-ring catheter feedback."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import os
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import ChannelFloat32, CompressedImage, PointCloud
import yaml


CAMERA_SETTING_KEYS = (
    "resolution", "fps", "sharpness", "exposure", "gain",
    "white_balance_temperature", "white_balance_auto_freeze_s",
    "white_balance_auto_freeze_retries", "white_balance_preflight_saturation",
    "exposure_white_balance_auto_freeze",
    "exposure_white_balance_keep_auto",
    "white_balance_require_neutral_scene",
    "brightness", "contrast", "hue",
    "saturation", "gamma",
)


@dataclass(frozen=True)
class EncodedPreview:
    """One decimated diagnostic image ready for nonblocking publication."""

    rig_id: str
    timestamp_ns: int
    frame_id: str
    source_width: int
    source_height: int
    eye: str
    jpeg: bytes


def encode_preview_frames(frames, eye: str, maximum_width: int,
                          jpeg_quality: int) -> tuple[EncodedPreview, ...]:
    """Resize and JPEG-encode at most one image per rig.

    This function is deliberately independent of ROS and the camera SDK so it
    can execute in a single-slot background worker. Camera capture already
    returns owned BGR arrays; retaining a frame here cannot block a later ZED
    grab.
    """
    import cv2

    if eye not in {"left", "right", "stereo"}:
        raise ValueError("preview eye must be left, right, or stereo")
    if maximum_width < 1:
        raise ValueError("preview maximum width must be positive")
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("preview JPEG quality must be in [1, 100]")
    encoded = []
    for rig_id, frame in frames.items():
        source_height, source_width = frame.left_bgr.shape[:2]
        if eye == "left":
            image = frame.left_bgr
        elif eye == "right":
            image = frame.right_bgr
        else:
            image = np.hstack((frame.left_bgr, frame.right_bgr))
        if image.shape[1] > maximum_width:
            scale = maximum_width / float(image.shape[1])
            image = cv2.resize(
                image, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_AREA)
        success, payload = cv2.imencode(
            ".jpg", image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
        if not success:
            raise RuntimeError(f"failed to encode preview for {rig_id}")
        encoded.append(EncodedPreview(
            str(rig_id), int(frame.timestamp_ns),
            f"{rig_id}_{eye}_optical", source_width, source_height, eye,
            payload.tobytes()))
    return tuple(encoded)


def _load_shape_tracking(shape_tracking_root: str):
    """Load camera/marker code from the installed deployment package."""
    try:
        from shape_tracking.multi_camera import (
            CameraCaptureWorker, LiveFrameSynchronizer,
            frame_pair_tolerance_ns, pair_recorded_frames,
            write_frame_pairs)
        from shape_tracking.dual_cli import _reuse_dual_registration
        from shape_tracking.online_markers import (
            MarkerEstimate,
            MarkerTrackingFailure,
            StereoRingTracker,
            fuse_marker_estimates,
        )
        from shape_tracking.session import load_session_registration
        from shape_tracking.zed_capture import ZedCamera
    except ImportError as exc:
        raise ImportError(
            "could not import the installed shape_tracking camera/marker "
            "library; install its wheel in the active Python environment. "
            f"The legacy shape_tracking_root value {shape_tracking_root!r} "
            f"is configuration-only. Underlying import error: {exc}"
        ) from exc
    return {
        "CameraCaptureWorker": CameraCaptureWorker,
        "LiveFrameSynchronizer": LiveFrameSynchronizer,
        "frame_pair_tolerance_ns": frame_pair_tolerance_ns,
        "pair_recorded_frames": pair_recorded_frames,
        "write_frame_pairs": write_frame_pairs,
        "reuse_dual_registration": _reuse_dual_registration,
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


def rig_liveness_values(rig_ids, sequences, frame_arrivals,
                        image_timestamps_ns, dropped_frames, errors, now):
    """Build bounded per-rig diagnostics without touching camera workers."""
    values = {}
    for rig_id in rig_ids:
        prefix = f"rig_{rig_id}"
        arrival = frame_arrivals.get(rig_id)
        values[f"{prefix}_sequence"] = str(sequences.get(rig_id, 0))
        values[f"{prefix}_frame_age_ms"] = (
            "never" if arrival is None
            else f"{1e3*max(0.0, now-arrival):.3f}")
        timestamp = image_timestamps_ns.get(rig_id)
        values[f"{prefix}_image_timestamp_ns"] = (
            "none" if timestamp is None else str(timestamp))
        values[f"{prefix}_dropped_frames"] = str(
            dropped_frames.get(rig_id, 0))
        values[f"{prefix}_worker_error"] = str(
            errors.get(rig_id) or "none")
    return values


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
        self.declare_parameter("workspace_boundary_margin_mm", 3.0)
        self.declare_parameter("maximum_cross_rig_disagreement_mm", 5.0)
        self.declare_parameter("maximum_rig_timestamp_skew_ms", 25.0)
        self.declare_parameter("maximum_rig_pairing_skew_ms", 20.0)
        self.declare_parameter("minimum_valid_rigs", 1)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("preview_enabled", False)
        self.declare_parameter("preview_rate_hz", 5.0)
        self.declare_parameter("preview_eye", "left")
        self.declare_parameter("preview_maximum_width", 640)
        self.declare_parameter("preview_jpeg_quality", 70)
        self.declare_parameter(
            "preview_topic_prefix", "/shape_tracking/preview")
        self.declare_parameter("recording_enabled", False)
        self.declare_parameter("recording_session_dir", "")

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
        pairing_skew_ns = int(round(
            float(self.get_parameter("maximum_rig_pairing_skew_ms").value)
            * 1e6))
        if not 0 <= pairing_skew_ns <= self.maximum_rig_timestamp_skew_ns:
            raise ValueError(
                "maximum_rig_pairing_skew_ms must be non-negative and no "
                "greater than maximum_rig_timestamp_skew_ms")
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
        self.rig_ids = requested_rigs
        self.last_sequences = {rig: 0 for rig in requested_rigs}
        self.last_frame_monotonic = {rig: None for rig in requested_rigs}
        self.last_frame_timestamp_ns = {rig: None for rig in requested_rigs}
        self.synchronizer = self.vision["LiveFrameSynchronizer"](
            requested_rigs, pairing_skew_ns)
        self.processing_pool = ThreadPoolExecutor(
            max_workers=2 * len(requested_rigs),
            thread_name_prefix="marker-eye")
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.last_valid_monotonic = None
        self.last_status_monotonic = 0.0
        self.started_monotonic = time.monotonic()
        self.closed = False
        self.preview_enabled = bool(
            self.get_parameter("preview_enabled").value)
        self.preview_rate_hz = float(
            self.get_parameter("preview_rate_hz").value)
        self.preview_eye = str(self.get_parameter("preview_eye").value)
        self.preview_maximum_width = int(
            self.get_parameter("preview_maximum_width").value)
        self.preview_jpeg_quality = int(
            self.get_parameter("preview_jpeg_quality").value)
        self.preview_topic_prefix = str(
            self.get_parameter("preview_topic_prefix").value).rstrip("/")
        if (self.preview_rate_hz <= 0.0
                or self.preview_eye not in {"left", "right", "stereo"}
                or self.preview_maximum_width < 1
                or not 1 <= self.preview_jpeg_quality <= 100
                or not self.preview_topic_prefix):
            raise ValueError("invalid diagnostic preview configuration")
        preview_qos = QoSProfile(depth=1)
        preview_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.preview_publishers = (
            {rig_id: self.create_publisher(
                CompressedImage,
                f"{self.preview_topic_prefix}/{rig_id}/compressed",
                preview_qos)
             for rig_id in requested_rigs}
            if self.preview_enabled else {})
        self.preview_pool = (
            ThreadPoolExecutor(max_workers=1, thread_name_prefix="preview-jpeg")
            if self.preview_enabled else None)
        self.preview_future = None
        self.next_preview_monotonic = self.started_monotonic
        self.preview_encoded = 0
        self.preview_dropped_busy = 0
        self.preview_errors = 0
        self.recording_enabled = bool(
            self.get_parameter("recording_enabled").value)
        recording_session_dir = str(
            self.get_parameter("recording_session_dir").value)
        self.recording_session_dir = None
        self.recording_metadata = None
        self.recording_active = False
        if self.recording_enabled:
            if len(requested_rigs) != 2:
                raise ValueError(
                    "full-shape SVO recording requires exactly two rigs")
            if not recording_session_dir:
                raise ValueError(
                    "recording_session_dir is required when recording is enabled")
            self.recording_session_dir = Path(
                recording_session_dir).expanduser().resolve()
            self.recording_session_dir.mkdir(parents=True, exist_ok=False)

        try:
            camera_infos = {}
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
                camera_infos[rig_id] = info
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
                    workspace_boundary_margin_mm=float(
                        self.get_parameter(
                            "workspace_boundary_margin_mm").value),
                    executor=self.processing_pool,
                )
                self.workers[rig_id] = worker
                self.trackers[rig_id] = tracker
            if self.recording_enabled:
                self._start_recording(
                    camera_config, registration_file, common, camera_infos)
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
            "marker-only processing, per-marker multi-rig fusion, "
            "complete fused sets only")
        if self.recording_active:
            self.get_logger().info(
                "dual SVO recording active at "
                f"{self.recording_session_dir}")
        self._publish_diagnostic(
            DiagnosticStatus.OK, "INITIALIZED",
            {"rigs": ",".join(requested_rigs)})

    def _process_latest(self):
        started = time.perf_counter()
        now = time.monotonic()
        self._publish_ready_preview()
        frames = {}
        camera_errors = []
        errors_by_rig = {}
        for rig_id, worker in self.workers.items():
            if worker.error is not None:
                camera_errors.append(f"{rig_id}:{worker.error}")
                errors_by_rig[rig_id] = worker.error
                continue
            frame = worker.latest()
            if frame is None or frame.sequence <= self.last_sequences[rig_id]:
                continue
            frames[rig_id] = frame
            self.last_sequences[rig_id] = frame.sequence
            self.last_frame_monotonic[rig_id] = now
            self.last_frame_timestamp_ns[rig_id] = frame.timestamp_ns
        for frame in frames.values():
            self.synchronizer.add(frame)
        synchronized = self.synchronizer.pop()
        if synchronized is None:
            if camera_errors:
                self._reject(
                    "camera_failed", ";".join(camera_errors),
                    DiagnosticStatus.ERROR,
                    self._rig_liveness_values(now, errors_by_rig))
            elif (now-(self.last_valid_monotonic
                       if self.last_valid_monotonic is not None
                       else self.started_monotonic) > self.feedback_timeout_s
                  and now-self.last_status_monotonic
                  >= self.feedback_timeout_s):
                values = {
                    "detail": "no synchronized frame pair",
                    "pair_age_s": (
                        "never" if self.last_valid_monotonic is None
                        else f"{now-self.last_valid_monotonic:.3f}"),
                    "synchronizer_dropped_frames": str(sum(
                        self.synchronizer.dropped_frames.values())),
                }
                values.update(self._rig_liveness_values(now, errors_by_rig))
                self._publish_diagnostic(
                    DiagnosticStatus.ERROR, "PAIRING_STALE", values)
            return
        frames = synchronized
        self._queue_preview(frames, now)

        estimates = []
        successful_rigs = set()
        failures = list(camera_errors)
        estimate_type = self.vision["MarkerEstimate"]
        for rig_id, frame in frames.items():
            result = self.trackers[rig_id].process(
                frame.timestamp_ns, frame.left_bgr, frame.right_bgr)
            if isinstance(result, estimate_type):
                estimates.append(result)
                successful_rigs.add(rig_id)
            else:
                failures.append(f"{rig_id}:{result.reason}:{result.detail}")
        if len(estimates) < self.minimum_valid_rigs:
            bootstrapped_rigs = []
            if estimates:
                reference = self.vision["fuse_marker_estimates"](
                    estimates, self.maximum_cross_rig_disagreement_mm)
                if isinstance(reference, estimate_type):
                    for rig_id in frames:
                        if (rig_id not in successful_rigs
                                and self.trackers[
                                    rig_id].bootstrap_from_reference(
                                        reference)):
                            bootstrapped_rigs.append(rig_id)
            if bootstrapped_rigs:
                sources = sorted(successful_rigs)
                targets = sorted(bootstrapped_rigs)
                detail = (
                    f"source={','.join(sources)};"
                    f"targets={','.join(targets)};"
                    f"failures={';'.join(failures)}")
                self._reject(
                    "multi_rig_bootstrap_pending", detail,
                    extra_values={
                        "bootstrap_source_rigs": ",".join(sources),
                        "bootstrap_target_rigs": ",".join(targets),
                    })
            else:
                self._reject(
                    "insufficient_valid_rigs", ";".join(failures))
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
        for tracker in self.trackers.values():
            tracker.seed_missing_from_fused(fused)
        self.marker_publisher.publish(self._point_cloud(fused))
        self.accepted_frames += 1
        completed = time.monotonic()
        self.last_valid_monotonic = completed
        latency_ms = (time.perf_counter() - started) * 1000.0
        marker_source_count = np.asarray(
            fused.source_count, dtype=np.float64)
        partial_markers = np.flatnonzero(
            marker_source_count < len(self.rig_ids)).tolist()
        values = {
            "image_timestamp_ns": str(fused.timestamp_ns),
            "rigs": ",".join(fused.rig_ids),
            "processing_latency_ms": f"{latency_ms:.3f}",
            "maximum_reprojection_error_px": (
                f"{float(np.max(fused.reprojection_error_px)):.3f}"),
            "marker_source_count": str([
                int(value) for value in marker_source_count]),
            "partial_markers": str(partial_markers),
            "rig_timestamp_skew_ms": f"{timestamp_span_ns * 1e-6:.3f}",
            "accepted_frames": str(self.accepted_frames),
            "rejected_frames": str(self.rejected_frames),
            "synchronizer_dropped_frames": str(sum(
                self.synchronizer.dropped_frames.values())),
        }
        if self.preview_enabled:
            values.update({
                "preview_encoded": str(self.preview_encoded),
                "preview_dropped_busy": str(self.preview_dropped_busy),
                "preview_errors": str(self.preview_errors),
            })
        values.update(self._rig_liveness_values(completed, errors_by_rig))
        partial_tracking = bool(partial_markers)
        self._publish_diagnostic(
            DiagnosticStatus.WARN if partial_tracking else DiagnosticStatus.OK,
            "PARTIAL_MULTI_RIG_TRACKING" if partial_tracking else "TRACKING",
            values)

    def _publish_ready_preview(self):
        """Publish completed JPEG work without waiting for the encoder."""
        future = self.preview_future
        if future is None or not future.done():
            return
        self.preview_future = None
        try:
            previews = future.result()
        except Exception as exc:
            self.preview_errors += 1
            self.get_logger().warn(
                "diagnostic preview encoding failed: "
                f"{type(exc).__name__}: {exc}")
            return
        for preview in previews:
            message = CompressedImage()
            message.header.stamp.sec = preview.timestamp_ns // 1_000_000_000
            message.header.stamp.nanosec = (
                preview.timestamp_ns % 1_000_000_000)
            message.header.frame_id = preview.frame_id
            message.format = (
                f"jpeg; source={preview.source_width}x"
                f"{preview.source_height}; eye={preview.eye}")
            message.data = preview.jpeg
            self.preview_publishers[preview.rig_id].publish(message)
            self.preview_encoded += 1

    def _queue_preview(self, frames, now):
        """Submit one replace/drop diagnostic frame without blocking vision."""
        if not self.preview_enabled or now < self.next_preview_monotonic:
            return
        self.next_preview_monotonic = now + 1.0/self.preview_rate_hz
        if not any(publisher.get_subscription_count() > 0
                   for publisher in self.preview_publishers.values()):
            return
        if self.preview_future is not None:
            self.preview_dropped_busy += 1
            return
        selected = {
            rig_id: frame for rig_id, frame in frames.items()
            if self.preview_publishers[rig_id].get_subscription_count() > 0
        }
        if not selected:
            return
        self.preview_future = self.preview_pool.submit(
            encode_preview_frames, selected, self.preview_eye,
            self.preview_maximum_width, self.preview_jpeg_quality)

    def _rig_liveness_values(self, now, errors=None):
        return rig_liveness_values(
            self.rig_ids, self.last_sequences, self.last_frame_monotonic,
            self.last_frame_timestamp_ns,
            self.synchronizer.dropped_frames, errors or {}, now)

    def _point_cloud(self, estimate) -> PointCloud:
        return marker_point_cloud(estimate, self.frame_id)

    def _reject(
            self, reason: str, detail: str,
            level: int = DiagnosticStatus.WARN,
            extra_values: dict | None = None):
        self.rejected_frames += 1
        values = {
            "detail": detail,
            "accepted_frames": str(self.accepted_frames),
            "rejected_frames": str(self.rejected_frames),
        }
        values.update(extra_values or {})
        self._publish_diagnostic(
            level, reason.upper(),
            values)

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
        errors = {
            rig_id: worker.error for rig_id, worker in self.workers.items()
            if worker.error is not None
        }
        values = {"age_s": age}
        if self.preview_enabled:
            values.update({
                "preview_encoded": str(self.preview_encoded),
                "preview_dropped_busy": str(self.preview_dropped_busy),
                "preview_errors": str(self.preview_errors),
            })
        values.update(self._rig_liveness_values(now, errors))
        self._publish_diagnostic(
            DiagnosticStatus.ERROR, "FEEDBACK_STALE",
            values)

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

    def _start_recording(
            self, camera_config, registration_file, common, camera_infos):
        """Start SVO recording in the existing camera-owner process."""
        session_id = self.recording_session_dir.name
        self.vision["reuse_dual_registration"](
            registration_file, self.recording_session_dir, session_id,
            self.rig_ids, camera_infos)
        self.recording_metadata = {
            "schema_version": 3,
            "session_id": session_id,
            "mode": "dual_camera_online_markers_and_svo",
            "modalities": {
                "stereo_camera": True,
                "camera_count": 2,
                "online_marker_tracking": True,
                "em_tracking": False,
            },
            "requested_resolution": common.get("resolution"),
            "requested_fps": common.get("fps"),
            "camera_config": str(Path(camera_config).expanduser().resolve()),
            "registration_file": "registration.json",
            "registration_reused_from": str(
                Path(registration_file).expanduser().resolve()),
            "svo_compression": common.get("svo_compression", "H264"),
            "camera_rigs": camera_infos,
            "camera_sync": {
                "method": "host_image_timestamp_nearest_neighbor",
                "reference_rig": self.rig_ids[0],
                "secondary_rig": self.rig_ids[1],
                "pair_file": "camera_frame_pairs.csv",
                "hardware_triggered": False,
            },
        }
        self._write_recording_metadata()
        started = []
        try:
            for rig_id, worker in self.workers.items():
                svo_path = self.recording_session_dir / (
                    f"{rig_id}_{session_id}.svo2")
                index_path = self.recording_session_dir / (
                    f"{rig_id}_frame_index.csv")
                if not worker.start_recording(svo_path, index_path):
                    raise RuntimeError(
                        f"could not start SVO recording for {rig_id}")
                started.append(worker)
        except Exception:
            for worker in reversed(started):
                worker.stop_recording()
            raise
        self.recording_active = True

    def _write_recording_metadata(self):
        if self.recording_metadata is None:
            return
        path = self.recording_session_dir / "session_metadata.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.recording_metadata, indent=2, sort_keys=True)
            + "\n", encoding="utf-8")
        os.replace(temporary, path)

    def _stop_recording(self):
        if not self.recording_active:
            return
        reports = {
            rig_id: worker.stop_recording()
            for rig_id, worker in self.workers.items()
        }
        self.recording_active = False
        reference = reports[self.rig_ids[0]]
        secondary = reports[self.rig_ids[1]]
        if reference is None or secondary is None:
            raise RuntimeError("both camera recordings must stop together")
        pairs = self.vision["pair_recorded_frames"](
            reference["records"], secondary["records"])
        pair_path = self.recording_session_dir / "camera_frame_pairs.csv"
        timing = self.vision["write_frame_pairs"](pair_path, pairs)
        timing.update({
            "reference_rig": self.rig_ids[0],
            "secondary_rig": self.rig_ids[1],
            "pair_file": pair_path.name,
            "pairing_tolerance_ms": self.vision[
                "frame_pair_tolerance_ns"](
                    reference["records"], secondary["records"]) * 1.0e-6,
        })
        for report in reports.values():
            report.pop("records", None)
        self.recording_metadata["recording_reports"] = reports
        self.recording_metadata["camera_sync"].update(timing)
        self._write_recording_metadata()
        message = (
            "dual SVO recording finalized: "
            f"paired={timing['paired_frames']} "
            f"p95_skew_ms={timing['p95_abs_delta_ms']}")
        if rclpy.ok(context=self.context):
            self.get_logger().info(message)
        else:
            # SIGINT may invalidate the ROS context before the node's finally
            # block finalizes the SVO sidecars.  Do not try to publish rosout
            # after that point; the recording has already been committed.
            print(message, flush=True)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.recording_active:
            try:
                self._stop_recording()
            except Exception as exc:
                message = (
                    "SVO recording finalization failed: "
                    f"{type(exc).__name__}: {exc}")
                if rclpy.ok(context=self.context):
                    self.get_logger().error(message)
                else:
                    print(message, flush=True)
        worker_rigs = set(self.workers)
        for worker in self.workers.values():
            try:
                worker.close()
            except Exception as exc:
                message = f"camera shutdown failed: {exc}"
                if rclpy.ok(context=self.context):
                    self.get_logger().error(message)
                else:
                    print(message, flush=True)
        self.workers.clear()
        for rig_id, camera in self.cameras.items():
            if rig_id in worker_rigs:
                continue
            try:
                camera.close()
            except Exception:
                pass
        self.cameras.clear()
        if self.preview_pool is not None:
            self.preview_pool.shutdown(wait=True, cancel_futures=True)
            self.preview_pool = None
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
