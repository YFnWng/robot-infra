import pytest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, PointCloud

from control_tasks.target_offset import (
    _arguments, controller_state, measured_tip)


def _markers():
    message = PointCloud()
    message.header.frame_id = "robot_base"
    message.points = [
        Point32(x=.01, y=.02, z=.03),
        Point32(x=.04, y=.05, z=.06),
        Point32(x=.07, y=.08, z=.09),
        Point32(x=.10, y=.11, z=.12),
    ]
    # Deliberately shuffle IDs: the helper must not assume list position.
    message.channels = [ChannelFloat32(
        name="marker_id", values=[2., 3., 0., 1.])]
    return message


def test_measured_tip_resolves_marker_id_three():
    assert measured_tip(_markers(), "robot_base") == pytest.approx(
        [.04, .05, .06])


def test_measured_tip_rejects_frame_or_missing_ids():
    message = _markers()
    with pytest.raises(ValueError, match="expected"):
        measured_tip(message, "other_frame")
    message.channels = []
    with pytest.raises(ValueError, match="marker_id"):
        measured_tip(message, "robot_base")


def test_controller_state_extracts_armed_flag():
    report = DiagnosticArray()
    status = DiagnosticStatus()
    status.name = "catheter_control/mppi"
    status.message = "READY"
    status.values = [KeyValue(key="armed", value="False")]
    report.status = [status]
    assert controller_state(report) == (False, "READY")


def test_arguments_accept_combined_bounded_offset():
    parsed = _arguments([
        "catheter_target_offset", "--dx-mm", "3", "--dy-mm", "4"])
    assert parsed.dx_mm == 3.
    assert parsed.dy_mm == 4.


def test_arguments_reject_zero_or_excessive_offset():
    with pytest.raises(SystemExit):
        _arguments(["catheter_target_offset"])
    with pytest.raises(SystemExit):
        _arguments([
            "catheter_target_offset", "--dx-mm", "11"])
