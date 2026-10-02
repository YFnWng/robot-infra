"""Deterministic source-time metrics; no running ROS graph."""
import pytest

from experiments.reaching_bag import reversals, window_metrics


def test_direction_counts_ignore_zero_and_require_persistence():
    values = [1, 1, 0, -1, -1, 1, 1]
    samples = [(i * 20_000_000, [value] * 3) for i, value in enumerate(values)]
    assert reversals(samples, [.02] * 3, True) == [2, 2, 2]
    assert reversals([], [2] * 3) is None


def test_gap_does_not_invent_reversal_or_travel():
    samples = [(1, [1] * 3), (20_000_001, [2] * 3),
               (800_000_001, [-10] * 3), (820_000_001, [-11] * 3)]
    result, _ = window_metrics({"positions": samples, "encoders": samples},
                               0, 900_000_000, [0, 0, 0])
    assert result["motor_travel_mm_deg_mm"] == [2, 2, 2]
    assert not result["stream_evidence"]["positions"]["coverage_ok"]
    assert result["encoder_reversals"] == [0, 0, 0]


def test_final_error_is_null_when_stale_and_maximum_is_observed_only():
    streams = {"markers": [(100_000_000, [0, 0, .001]),
                           (200_000_000, [0, 0, .003])], "commands": []}
    result, _ = window_metrics(streams, 0, 1_000_000_000, [0, 0, 0])
    assert result["maximum_error_mm"] == pytest.approx(3)
    assert result["bag_final_error_mm"] is None
    assert result["command_reversals"] is None


def test_trial_window_excludes_home_samples():
    result, _ = window_metrics({"markers": [(10, [1, 1, 1]), (100, [0, 0, .001])]},
                               50, 150, [0, 0, 0])
    assert result["maximum_error_mm"] == pytest.approx(1)


def test_finalized_bag_decode_and_plot_without_ros_node(tmp_path):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import ChannelFloat32, PointCloud
    from geometry_msgs.msg import Point32
    from experiments.reaching_bag import read_streams, write_trial_plot

    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topic = "/shape_tracking/markers"
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=topic, type="sensor_msgs/msg/PointCloud", serialization_format="cdr"))
    for index in range(3):
        message = PointCloud()
        message.header.frame_id = "robot_base"
        message.header.stamp.sec = 1
        message.header.stamp.nanosec = index * 100_000_000
        message.points = [Point32(x=0., y=0., z=.001 * (index + 1))]
        message.channels = [ChannelFloat32(name="marker_id", values=[3.])]
        writer.write(topic, serialize_message(message), 1_000_000_000 + index * 100_000_000)
    del writer
    streams, issues = read_streams(bag, {})
    assert len(streams["markers"]) == 3
    assert any("/device/state" in issue for issue in issues)
    metrics, windows = window_metrics(streams, 1_000_000_000, 1_200_000_000, [0, 0, 0])
    assert metrics["maximum_error_mm"] == pytest.approx(3)
    path = tmp_path / "trial.png"
    write_trial_plot(path, windows, [0, 0, 0], 1_000_000_000)
    assert path.stat().st_size > 1000
