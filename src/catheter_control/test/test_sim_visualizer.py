from pathlib import Path

import pytest

import numpy as np

from catheter_control.sim_visualizer import (
    _base_axis_specs, _pose_axis_endpoints)
from catheter_control.rviz_record import _window_id


VISUALIZER_SOURCE = (Path(__file__).resolve().parents[1]
                     / "catheter_control" / "simulation" / "sim_visualizer.py")


def test_base_axes_use_conventional_xyz_colors_and_scale():
    axes = _base_axis_specs(.025)
    assert [item[0] for item in axes] == ["X", "Y", "Z"]
    endpoints = [item[1] for item in axes]
    assert [(point.x, point.y, point.z) for point in endpoints] == [
        pytest.approx((.025, 0.0, 0.0)),
        pytest.approx((0.0, .025, 0.0)),
        pytest.approx((0.0, 0.0, .025)),
    ]
    colors = [item[2] for item in axes]
    assert colors[0].r > colors[0].g
    assert colors[1].g > colors[1].r
    assert colors[2].b > colors[2].r


@pytest.mark.parametrize("length", [0.0, -1.0, float("nan")])
def test_base_axes_reject_invalid_length(length):
    with pytest.raises(ValueError):
        _base_axis_specs(length)


def test_pose_axes_express_material_directions_in_base_frame():
    pose = np.eye(4)
    pose[:3, 3] = [0.01, 0.02, 0.03]
    pose[:3, :3] = [[0.0, -1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]]
    origin, endpoints = _pose_axis_endpoints(pose, 0.01)
    assert (origin.x, origin.y, origin.z) == pytest.approx(
        (0.01, 0.02, 0.03))
    assert [(point.x, point.y, point.z) for point in endpoints] == [
        pytest.approx((0.01, 0.03, 0.03)),
        pytest.approx((0.0, 0.02, 0.03)),
        pytest.approx((0.01, 0.02, 0.04)),
    ]


def test_estimator_trace_subscription_matches_best_effort_publisher():
    source = VISUALIZER_SOURCE.read_text(encoding="utf-8")
    assert ('EstimatorStateTrace, "/sim/catheter_mppi/estimator_trace",\n'
            '            self._estimator_trace_cb, qos_profile_sensor_data)'
            in source)


def test_rviz_recorder_finds_one_matching_window():
    tree = '''
       0x02a00007 "catheter_sim_rviz - RViz": ("rviz2" "Rviz")
       0x03000001 "Terminal": ("gnome-terminal" "Gnome-terminal")
    '''
    assert _window_id(tree, "RViz") == 0x02a00007


def test_rviz_recorder_rejects_ambiguous_windows():
    tree = '''
       0x02a00007 "first RViz": ("rviz2" "Rviz")
       0x02a00008 "second RViz": ("rviz2" "Rviz")
    '''
    with pytest.raises(RuntimeError, match="found 2 windows"):
        _window_id(tree, "RViz")
