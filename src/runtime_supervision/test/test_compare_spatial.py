import copy

import pytest

from runtime_supervision.compare_spatial import cartesian_difference, frozen_difference


def trajectory():
    return [{"stage": "marker", "receipt_ns": 10, "source_ns": 9, "state_ns": 10,
             "markers_m": [[0., 0., 0.], [0., 0., .01]],
             "centerline_m": [[0., 0., 0.], [0., 0., .01]]}]


def test_cartesian_difference_mm_and_timestamps():
    old = trajectory()
    new = copy.deepcopy(old)
    new[0]["markers_m"][-1][0] = .002
    result = cartesian_difference(old, new)
    assert result["tip"]["maximum"]["error_mm"] == pytest.approx(2.)
    assert result["centerline"]["maximum"]["error_mm"] == 0.
    new[0]["state_ns"] += 1
    with pytest.raises(ValueError, match="grids"):
        cartesian_difference(old, new)


def test_frozen_difference_requires_identical_full_root_hash():
    old = {"fixture_sha256": "fixture", "rates_rad_s": [0.], "dt_s": [.2],
           "rows": [{"index": 0, "source_ns": 1, "input_state_sha256": "root",
                     "tips_m": [[0., 0., 0.]], "final_markers_m": [[0., 0., 0.]], "lambda": [0.]}]}
    new = copy.deepcopy(old)
    new["rows"][0]["tips_m"][0][0] = .001
    assert frozen_difference(old, new)["maximum_tip_difference_mm"] == pytest.approx(1.)
    new["rows"][0]["input_state_sha256"] = "different"
    with pytest.raises(ValueError, match="root state"):
        frozen_difference(old, new)


def test_ros_input_allowlist_rejects_all_commands():
    from types import SimpleNamespace
    from runtime_supervision.compare_ros import input_allowed
    assert input_allowed("/device/state", SimpleNamespace(predicate=ord("E")))
    assert input_allowed("/device/state", SimpleNamespace(predicate=ord("P")))
    assert not input_allowed("/device/state", SimpleNamespace(predicate=0))
    assert not input_allowed("/device/command", SimpleNamespace(predicate=ord("P")))
    assert not input_allowed("/teleop/control", SimpleNamespace())


def test_ros_probe_cannot_inherit_command_enable_or_sim_clock():
    from runtime_supervision.compare_ros import probe_parameters
    recorded = {"command_output_enabled": True, "use_sim_time": True, "samples": 512}
    values = probe_parameters(recorded, "model", "limits", "cuda")
    assert not values["command_output_enabled"]
    assert not values["use_sim_time"]
    assert recorded["command_output_enabled"]  # input is not mutated
    assert values["samples"] == 512
    assert values["estimator_device"] == "cuda" and values["estimator_dtype"] == "float32"
