from pathlib import Path

import numpy as np
import pytest
import yaml

from control_tasks.trajectory_file import (
    _circle_waypoints, load_trajectory_file)


CONFIG_ROOT = (Path(__file__).resolve().parents[2]
               / "catheter_control" / "config")


HARDWARE_CIRCLE = (CONFIG_ROOT / "circle_trajectory.yaml")


def test_circle_uses_current_tip_radius_angle_and_height():
    tip = np.array([.003, .004, .020])
    points = _circle_waypoints({
        "type": "circle_xy_from_current_tip",
        "radius_offset_mm": 3.0,
        "waypoint_count": 5,
        "direction": "ccw",
        "close_circle": True,
    }, tip)
    assert points.shape == (5, 3)
    assert np.linalg.norm(points[0, :2]) == pytest.approx(.008)
    assert points[0] == pytest.approx([.0048, .0064, .020])
    assert points[-1] == pytest.approx(points[0])
    assert np.all(points[:, 2] == pytest.approx(.020))


def test_clockwise_circle_reverses_initial_tangent():
    tip = [.010, 0.0, .020]
    ccw = _circle_waypoints({
        "type": "circle_xy_from_current_tip", "waypoint_count": 4,
        "direction": "ccw", "close_circle": False,
    }, tip)
    cw = _circle_waypoints({
        "type": "circle_xy_from_current_tip", "waypoint_count": 4,
        "direction": "cw", "close_circle": False,
    }, tip)
    assert ccw[1, 1] > 0.0
    assert cw[1, 1] < 0.0


def test_yz_circle_uses_requested_center_and_radius():
    tip = np.array([.020, .019, .055])
    points = _circle_waypoints({
        "type": "circle_yz_from_current_tip",
        "radius_mm": 10.0,
        "center_x_offset_mm": 3.0,
        "waypoint_count": 5,
        "direction": "ccw",
        "close_circle": True,
    }, tip)
    center = np.array([.023, 0.0, .055])
    assert points.shape == (5, 3)
    assert np.all(points[:, 0] == pytest.approx(center[0]))
    assert np.linalg.norm(points[:, 1:]-center[1:], axis=1) == pytest.approx(
        np.full(5, .010))
    assert points[0] == pytest.approx([.023, .010, .055])
    assert points[-1] == pytest.approx(points[0])


def test_fixed_reference_tip_overrides_live_tip():
    generator = {
        "type": "circle_yz_from_current_tip",
        "reference_tip_m": [.020, .019, .055],
        "radius_mm": 10.0,
        "center_x_offset_mm": 10.0,
        "waypoint_count": 5,
    }
    first = _circle_waypoints(generator, [.100, .100, .100])[0]
    assert first == pytest.approx([.030, .010, .055])


def test_approach_waypoints_connect_reference_to_circle_without_duplicates():
    reference = np.array([.020, .019, .055])
    generator = {
        "type": "circle_yz_from_current_tip",
        "reference_tip_m": reference.tolist(),
        "radius_mm": 10.0,
        "center_x_offset_mm": 10.0,
        "approach_waypoint_count": 5,
        "waypoint_count": 6,
    }
    points = _circle_waypoints(generator, [.100, .100, .100])
    circle_start = np.array([.030, .010, .055])
    assert points.shape == (11, 3)
    for index in range(5):
        fraction = (index+1)/6.0
        assert points[index] == pytest.approx(
            reference+fraction*(circle_start-reference))
    assert points[5] == pytest.approx(circle_start)
    assert not np.array_equal(points[4], points[5])


def test_hardware_circle_has_fixed_reviewed_center_and_tolerance():
    spec = load_trajectory_file(HARDWARE_CIRCLE, [.100, .100, .100])
    assert spec.action_name == "/catheter_mppi/track_tip_trajectory"
    assert spec.marker_topic == "/shape_tracking/markers"
    assert spec.tolerance_mm == pytest.approx(1.8)
    assert len(spec.waypoints_m) == 23
    assert np.all(spec.waypoints_m[5:, 0] == pytest.approx(
        .03190615423023701))
    center_yz = np.array([0.0, .07138097286224365])
    radii = np.linalg.norm(spec.waypoints_m[5:, 1:]-center_yz, axis=1)
    assert radii == pytest.approx(np.full(len(radii), .010))


def test_yaml_circle_expands_scalar_timeout(tmp_path: Path):
    path = tmp_path/"trajectory.yaml"
    path.write_text(yaml.safe_dump({
        "frame_id": "robot_base",
        "action_name": "/sim/action",
        "marker_topic": "/sim/markers",
        "auto_arm": True,
        "tolerance_mm": .5,
        "settle_time_s": .2,
        "generator": {
            "type": "circle_xy_from_current_tip",
            "radius_offset_mm": 3.0,
            "waypoint_count": 6,
            "waypoint_timeout_s": 2.5,
        },
    }), encoding="utf-8")
    spec = load_trajectory_file(path, [.003, .004, .020])
    assert spec.waypoints_m.shape == (6, 3)
    assert spec.waypoint_timeouts_s.tolist() == [2.5]*6
    assert spec.auto_arm


@pytest.mark.parametrize("change", [
    {"waypoint_count": 2},
    {"direction": "sideways"},
    {"radius_offset_mm": -100.0},
    {"approach_waypoint_count": -1},
])
def test_invalid_circle_is_rejected(change):
    generator = {
        "type": "circle_xy_from_current_tip",
        "waypoint_count": 6,
    }
    generator.update(change)
    with pytest.raises(ValueError):
        _circle_waypoints(generator, [.003, .004, .020])
