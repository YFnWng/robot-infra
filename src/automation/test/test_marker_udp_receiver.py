import json

import numpy as np
import pytest

from automation.marker_tracking.udp_receiver import (
    PROTOCOL_VERSION,
    SCHEMA,
    decode_marker_packet,
    marker_packet_point_cloud,
)


def packet_document(now_ns):
    return {
        "schema": SCHEMA,
        "version": PROTOCOL_VERSION,
        "session_id": "session",
        "sequence": 7,
        "image_timestamp_ns": now_ns - 20_000_000,
        "send_timestamp_ns": now_ns - 2_000_000,
        "frame_id": "robot_base",
        "marker_ids": [0, 1, 2, 3],
        "points_m": (np.arange(12).reshape(4, 3) * 0.001).tolist(),
        "confidence": [0.9, 0.8, 0.7, 0.6],
        "reprojection_error_px": [0.1, 0.2, 0.3, 0.4],
        "source_rig_count": [2.0, 2.0, 2.0, 2.0],
        "rig_ids": ["primary", "oblique"],
    }


def decode(document, now_ns):
    return decode_marker_packet(
        json.dumps(document).encode(), now_ns, "robot_base",
        maximum_packet_age_s=0.5,
        maximum_future_skew_s=0.1,
        maximum_abs_position_m=2.0,
        maximum_reprojection_error_px=5.0)


def test_decode_and_point_cloud_preserve_acquisition_time_and_units():
    now_ns = 2_000_000_000
    packet = decode(packet_document(now_ns), now_ns)
    message = marker_packet_point_cloud(packet)
    assert packet.sequence == 7
    assert message.header.frame_id == "robot_base"
    assert message.header.stamp.sec == 1
    assert message.header.stamp.nanosec == 980_000_000
    np.testing.assert_allclose(
        [point.x for point in message.points],
        [0.0, 0.003, 0.006, 0.009])
    assert list(message.channels[0].values) == [0.0, 1.0, 2.0, 3.0]


@pytest.mark.parametrize("mutation,match", [
    (lambda item: item.update(marker_ids=[0, 1, 3, 2]), "marker_ids"),
    (lambda item: item.update(frame_id="camera"), "frame mismatch"),
    (lambda item: item.update(points_m=[[0, 0, 0]] * 3), "shape"),
    (lambda item: item.update(confidence=[2, 1, 1, 1]), "confidence"),
])
def test_rejects_malformed_or_unsafe_packets(mutation, match):
    now_ns = 2_000_000_000
    document = packet_document(now_ns)
    mutation(document)
    with pytest.raises(ValueError, match=match):
        decode(document, now_ns)


def test_rejects_stale_and_future_packets():
    now_ns = 2_000_000_000
    stale = packet_document(now_ns)
    stale["image_timestamp_ns"] = now_ns - 600_000_000
    with pytest.raises(ValueError, match="stale image"):
        decode(stale, now_ns)
    future = packet_document(now_ns)
    future["send_timestamp_ns"] = now_ns + 200_000_000
    with pytest.raises(ValueError, match="future"):
        decode(future, now_ns)
