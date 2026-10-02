from pathlib import Path
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus

from perception.marker_tracking import (
    MarkerTrackingNode,
    encode_preview_frames,
    load_live_camera_config,
    marker_point_cloud,
    rig_liveness_values,
)


def test_diagnostics_report_owned_recording_readiness_without_mutating_input():
    messages = []
    node = SimpleNamespace(
        recording_active=True, recording_session_dir=Path("/tmp/descriptive_video"),
        cameras={"primary": object(), "oblique": object()},
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)),
        diagnostic_publisher=SimpleNamespace(publish=messages.append),
    )
    values = {"accepted_frames": "4"}
    MarkerTrackingNode._publish_diagnostic(node, DiagnosticStatus.OK, "TRACKING", values)
    report = {item.key: item.value for item in messages[0].status[0].values}
    assert report["recording_active"] == "true"
    assert report["recording_session_dir"] == "/tmp/descriptive_video"
    assert values == {"accepted_frames": "4"}


def test_preview_encoding_is_decimated_and_timestamped():
    import cv2

    frame = SimpleNamespace(
        timestamp_ns=123,
        left_bgr=np.full((72, 128, 3), 40, dtype=np.uint8),
        right_bgr=np.full((72, 128, 3), 80, dtype=np.uint8))
    previews = encode_preview_frames(
        {"primary": frame}, "left", 64, 70)

    assert len(previews) == 1
    preview = previews[0]
    assert preview.rig_id == "primary"
    assert preview.timestamp_ns == 123
    assert preview.source_width == 128
    assert preview.source_height == 72
    decoded = cv2.imdecode(
        np.frombuffer(preview.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (36, 64, 3)


def test_load_live_camera_config(tmp_path: Path):
    path = tmp_path / "camera.yaml"
    path.write_text(
        "camera:\n"
        "  resolution: HD1080\n"
        "  fps: 30\n"
        "recording:\n"
        "  svo_compression: H264\n"
        "cameras:\n"
        "  primary:\n"
        "    serial: 20757336\n"
        "  oblique:\n"
        "    serial: 26080456\n",
        encoding="utf-8")
    common, serials = load_live_camera_config(path)
    assert common["resolution"] == "HD1080"
    assert common["fps"] == 30
    assert common["svo_compression"] == "H264"
    assert serials == {
        "primary": 20757336,
        "oblique": 26080456,
    }


def test_marker_point_cloud_preserves_timestamp_order_and_units():
    estimate = SimpleNamespace(
        timestamp_ns=1_234_567_890,
        points_base_mm=np.array([
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
            [10.0, 11.0, 12.0],
        ]),
        confidence=np.array([0.9, 0.8, 0.7, 0.6]),
        reprojection_error_px=np.array([0.1, 0.2, 0.3, 0.4]),
        source_count=np.array([2.0, 2.0, 2.0, 2.0]),
    )
    message = marker_point_cloud(estimate, "robot_base")
    assert message.header.frame_id == "robot_base"
    assert message.header.stamp.sec == 1
    assert message.header.stamp.nanosec == 234_567_890
    assert [point.x for point in message.points] == [
        0.001, 0.004, 0.007, 0.010]
    assert message.channels[0].name == "marker_id"
    assert list(message.channels[0].values) == [0.0, 1.0, 2.0, 3.0]
    assert message.channels[3].name == "source_rig_count"
    assert list(message.channels[3].values) == [2.0, 2.0, 2.0, 2.0]


def test_rig_liveness_values_identify_the_stalled_camera():
    values = rig_liveness_values(
        ("primary", "oblique"),
        {"primary": 100, "oblique": 80},
        {"primary": 9.99, "oblique": 9.25},
        {"primary": 123, "oblique": 100},
        {"primary": 2, "oblique": 22},
        {"oblique": "capture stalled"},
        now=10.0,
    )

    assert values["rig_primary_frame_age_ms"] == "10.000"
    assert values["rig_oblique_frame_age_ms"] == "750.000"
    assert values["rig_oblique_sequence"] == "80"
    assert values["rig_oblique_dropped_frames"] == "22"
    assert values["rig_oblique_worker_error"] == "capture stalled"
