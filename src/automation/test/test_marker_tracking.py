from pathlib import Path
from types import SimpleNamespace

import numpy as np

from automation.marker_tracking.node import (
    load_live_camera_config,
    marker_point_cloud,
)


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
