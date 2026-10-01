"""Marker-message validation at the estimator ownership boundary."""
import numpy as np
from sensor_msgs.msg import PointCloud


def stamp_ns(header) -> int:
    return (int(header.stamp.sec)*1_000_000_000
            + int(header.stamp.nanosec))


def _channel_map(message: PointCloud) -> dict[str, np.ndarray]:
    return {
        str(channel.name): np.asarray(channel.values, dtype=np.float64)
        for channel in message.channels
    }


def marker_measurement(message: PointCloud, required_frame: str):
    """Validate and reorder one marker point cloud by marker ID."""
    if message.header.frame_id != required_frame:
        raise ValueError("marker_frame_mismatch")
    if len(message.points) != 4:
        raise ValueError("marker_count")
    channels = _channel_map(message)
    required = (
        "marker_id", "confidence", "reprojection_error_px",
        "source_rig_count")
    if any(name not in channels or channels[name].shape != (4,)
           for name in required):
        raise ValueError("marker_quality_channels")
    marker_id = channels["marker_id"]
    if (not np.all(np.isfinite(marker_id))
            or sorted(int(value) for value in marker_id) != [0, 1, 2, 3]
            or not np.allclose(marker_id, np.round(marker_id))):
        raise ValueError("marker_ids")
    order = np.argsort(marker_id.astype(int))
    points = np.asarray(
        [[point.x, point.y, point.z] for point in message.points],
        dtype=np.float64)[order]
    quality = {
        name: channels[name][order]
        for name in required if name != "marker_id"
    }
    if (not np.all(np.isfinite(points))
            or any(not np.all(np.isfinite(value))
                   for value in quality.values())):
        raise ValueError("nonfinite_marker_measurement")
    timestamp_ns = stamp_ns(message.header)
    if timestamp_ns <= 0:
        raise ValueError("invalid_marker_timestamp")
    return timestamp_ns, points, quality
