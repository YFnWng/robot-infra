from pathlib import Path

import numpy as np
import pytest
import yaml

from catheter_control.sparse_point_experiment import (
    decoupled_tendon_prehome_target, load_sparse_point_experiment,
    sparse_circle_targets)


CONFIG = (Path(__file__).resolve().parents[1]
          / "config" / "sparse_circle_points_sim.yaml")
HARDWARE_CONFIG = (Path(__file__).resolve().parents[1]
                   / "config" / "sparse_circle_points_hardware.yaml")
HISTORY_SIM_CONFIG = (Path(__file__).resolve().parents[1]
                      / "config"
                      / "sparse_circle_points_sim_history_preserving.yaml")
AXIAL_HARDWARE_CONFIG = (Path(__file__).resolve().parents[1]
                         / "config" / "axial_points_v174_hardware.yaml")
V175_AXIAL_HARDWARE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "axial_points_v175_hardware_no_rotation.yaml")
V175_TWO_AXIS_HARDWARE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "two_axis_points_v175_hardware_no_rotation.yaml")
V175_TWO_AXIS_SIM_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "two_axis_points_v175_sim_no_rotation.yaml")
V175_FAR_TWO_AXIS_HARDWARE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "far_two_axis_points_v175_hardware_no_rotation.yaml")
V175_FAR_TWO_AXIS_SIM_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "far_two_axis_points_v175_sim_no_rotation.yaml")
V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "farther_two_axis_points_v175_hardware_no_rotation.yaml")
V175_TENDON_12_TWO_AXIS_HARDWARE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "farther_tendon_12_two_axis_points_v175_hardware_no_rotation.yaml")
V175_PLAIN_TAKEUP_TARGET_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "farther_tendon_12_two_axis_points_v175_plain_takeup_hardware_no_rotation.yaml")
V171_PLAIN_TARGET_CONFIG = (
    Path(__file__).resolve().parents[1] / "config"
    / "farther_tendon_12_two_axis_points_v171_plain_hardware_no_rotation.yaml")
V175_PLAIN_TAKEUP_PROFILE = (
    Path(__file__).resolve().parents[1] / "config"
    / "v175_plain_takeup_hardware_no_rotation.yaml")
V171_PLAIN_PROFILE = (
    Path(__file__).resolve().parents[1] / "config"
    / "v171_plain_hardware_no_rotation.yaml")
V174_PROFILE = (Path(__file__).resolve().parents[1]
                / "config" / "v174_fixed_hardware_no_rotation.yaml")
CAUSAL_V2_PROFILE = (Path(__file__).resolve().parents[1]
                     / "config" / "causal_v2_fixed_hardware.yaml")
V175_NO_ROTATION_PROFILE = (
    Path(__file__).resolve().parents[1] / "config"
    / "v175_grouped_hardware_no_rotation.yaml")


def test_sparse_circle_uses_eight_unique_absolute_targets():
    spec = load_sparse_point_experiment(CONFIG)
    home_tip = np.array([.020, .017, .060])
    targets = sparse_circle_targets(spec, home_tip)

    assert targets.shape == (8, 3)
    assert len(np.unique(np.round(targets, decimals=12), axis=0)) == 8
    assert targets[:, 0] == pytest.approx(np.full(8, .035))
    center_yz = np.array([0.0, .060])
    assert np.linalg.norm(
        targets[:, 1:]-center_yz, axis=1) == pytest.approx(
            np.full(8, .010))


def test_sparse_experiment_has_reviewed_home_and_independent_action():
    spec = load_sparse_point_experiment(CONFIG)

    assert spec.home_position == pytest.approx([20, 0, 0, 0, 0, 0])
    assert np.all(spec.home_speed > 0.0)
    assert np.all(spec.home_tolerance > 0.0)
    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.home_mode_settle_s == pytest.approx(0.15)
    assert spec.action_name == "/sim/catheter_mppi/track_tip_trajectory"
    assert spec.generator["close_circle"] is False
    assert spec.plant_reset_mode == "full_simulation"
    assert spec.plant_reset_services == (
        "/sim/catheter_sim/reset_plant",
        "/sim/catheter_sim/reset_perception",
        "/sim/catheter_mppi/reset_simulation_state")


def test_hardware_sparse_experiment_preserves_physical_history():
    spec = load_sparse_point_experiment(HARDWARE_CONFIG)

    assert spec.plant_reset_mode == "history_preserving"
    assert spec.plant_reset_services == ()


def test_axial_hardware_targets_are_frozen_relative_to_home_tip():
    spec = load_sparse_point_experiment(AXIAL_HARDWARE_CONFIG)
    home_tip = np.array([.021, .017, .071])
    targets = sparse_circle_targets(spec, home_tip)

    assert spec.plant_reset_mode == "history_preserving"
    assert spec.home_position == pytest.approx([20, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(
        targets,
        np.asarray([
            [.021, .017, .076],
            [.021, .017, .066],
        ]),
        rtol=0.0,
        atol=1.0e-12,
    )


def test_v175_axial_hardware_test_is_fail_closed_no_rotation():
    spec = load_sparse_point_experiment(V175_AXIAL_HARDWARE_CONFIG)

    assert spec.rotation_guard_enabled is True
    assert spec.rotation_guard_axis == 1
    assert spec.rotation_guard_tolerance == pytest.approx(1.0e-9)
    assert spec.required_controller_status == {
        "command_output_enabled": True,
        "marker_estimator": "ukf",
        "model_adaptation_enabled": False,
        "engaged_gain_enabled": True,
        "mppi_engaged_gain_scenarios": True,
        "mppi_samples": 512,
        "mppi_point_rollout_step_s": 0.2,
        "mppi_point_rollout_coarse_steps": True,
        "mppi_point_prediction_tail_steps": 0,
        "controller_velocity_max": [10.0, 0.0, 4.5, 4.0, 25.0, 25.0],
    }


def test_v175_two_axis_targets_are_model_generated_and_reserve_qualified():
    spec = load_sparse_point_experiment(V175_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.generator["type"] == (
        "controller_model_joint_displacements")
    assert spec.generator["logical_displacements"] == [
        [5.0, 0.0, 0.0],
        [0.0, 0.0, 3.0],
        [7.0, 0.0, 3.0],
        [-4.0, 0.0, 3.0],
    ]
    assert spec.generator["minimum_endpoint_reserve"] == [
        2.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True

    with pytest.raises(ValueError, match="preview service"):
        sparse_circle_targets(spec, np.array([.02, .01, .07]))


def test_v175_two_axis_sim_uses_full_reset_and_simulated_preview_service():
    spec = load_sparse_point_experiment(V175_TWO_AXIS_SIM_CONFIG)

    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.plant_reset_mode == "full_simulation"
    assert spec.plant_reset_services == (
        "/sim/catheter_sim/reset_plant",
        "/sim/catheter_sim/reset_perception",
        "/sim/catheter_mppi/reset_simulation_state",
    )
    assert spec.generator["service"] == (
        "/sim/catheter_mppi/generate_sparse_targets")
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.required_controller_status["controller_velocity_max"][1] == 0.0


@pytest.mark.parametrize("path", [
    V175_FAR_TWO_AXIS_HARDWARE_CONFIG,
    V175_FAR_TWO_AXIS_SIM_CONFIG,
])
def test_v175_far_targets_are_per_home_model_generated(path):
    spec = load_sparse_point_experiment(path)

    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [8.0, 0.0, 0.0],
        [0.0, 0.0, 4.5],
        [-8.0, 0.0, 3.0],
        [-6.0, 0.0, 4.5],
    ]
    assert spec.generator["minimum_tip_displacement_mm"] == 3.0
    assert spec.generator["maximum_tip_displacement_mm"] == 20.0
    assert spec.rotation_guard_enabled is True


def test_v175_farther_hardware_targets_extend_qualified_block():
    spec = load_sparse_point_experiment(
        V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.target_timeout_s == pytest.approx(30.0)
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [10.0, 0.0, 0.0],
        [0.0, 0.0, 6.0],
        [-10.0, 0.0, 4.5],
        [-8.0, 0.0, 6.0],
    ]
    assert spec.generator["minimum_endpoint_reserve"] == [
        2.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    assert spec.generator["minimum_tip_displacement_mm"] == 4.0
    assert spec.generator["maximum_tip_displacement_mm"] == 25.0
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True


def test_v175_tendon_12_targets_preserve_insertion_range():
    baseline = load_sparse_point_experiment(
        V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG)
    spec = load_sparse_point_experiment(
        V175_TENDON_12_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.target_timeout_s == pytest.approx(30.0)
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [10.0, 0.0, 0.0],
        [0.0, 0.0, 12.0],
        [-10.0, 0.0, 9.0],
        [-8.0, 0.0, 12.0],
    ]
    assert [row[0] for row in spec.generator["logical_displacements"]] == [
        row[0] for row in baseline.generator["logical_displacements"]]
    assert spec.generator["minimum_endpoint_reserve"] == [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert spec.generator["minimum_tip_displacement_mm"] == 4.0
    assert spec.generator["maximum_tip_displacement_mm"] == 35.0
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True


@pytest.mark.parametrize("path, expected", [
    (
        V175_PLAIN_TAKEUP_TARGET_CONFIG,
        {
            "backlash_compensation_enabled": True,
            "takeup_transaction_enabled": True,
            "backlash_model_encoder_input": "estimated_transmitted",
            "engaged_gain_enabled": True,
            "mppi_engaged_gain_scenarios": True,
        },
    ),
    (
        V171_PLAIN_TARGET_CONFIG,
        {
            "backlash_compensation_enabled": False,
            "takeup_transaction_enabled": False,
            "backlash_model_encoder_input": "raw_shaft",
            "engaged_gain_enabled": False,
            "mppi_engaged_gain_scenarios": False,
        },
    ),
])
def test_plain_baseline_targets_require_exact_runtime_identity(path, expected):
    spec = load_sparse_point_experiment(path)

    assert spec.generator["type"] == "absolute_targets_m"
    np.testing.assert_allclose(spec.generator["targets_m"], [
        [0.024685867130756378, 0.020163964480161667, 0.08448133617639542],
        [0.03320372849702835, 0.02986534871160984, 0.06324673444032669],
        [0.031191052868962288, 0.02978038229048252, 0.0510878749191761],
        [0.031725503504276276, 0.029091984033584595, 0.050746046006679535],
    ], rtol=0.0, atol=1.0e-12)
    np.testing.assert_allclose(
        sparse_circle_targets(spec, np.zeros(3)),
        spec.generator["targets_m"], rtol=0.0, atol=0.0)
    common = {
        "command_output_enabled": True,
        "marker_estimator": "ukf",
        "model_adaptation_enabled": False,
        "mppi_grouped_mode_sampling": False,
        "mppi_best_candidate_guard": False,
        "mppi_takeup_risk_cost_weight": 0,
        "reversal_mode_selector": "plain_mppi",
        "mppi_samples": 512,
        "mppi_point_rollout_step_s": 0.2,
        "mppi_point_rollout_coarse_steps": True,
        "mppi_point_prediction_tail_steps": 0,
        "controller_velocity_max": [10.0, 0.0, 4.5, 4.0, 25.0, 25.0],
    }
    assert spec.required_controller_status == {**common, **expected}


def test_plain_hardware_profiles_are_controlled_ablations():
    grouped = yaml.safe_load(
        V175_NO_ROTATION_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]
    compensated = yaml.safe_load(
        V175_PLAIN_TAKEUP_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]
    uncompensated = yaml.safe_load(
        V171_PLAIN_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]

    assert set(compensated) == set(grouped)
    assert {
        key for key in grouped if compensated[key] != grouped[key]
    } == {
        "mppi_grouped_mode_sampling",
        "mppi_best_candidate_guard",
        "mppi_takeup_risk_weight",
        "reversal_scheduler_enabled",
    }
    assert compensated["backlash_compensation_enabled"] is True
    assert compensated["takeup_transaction_enabled"] is True

    assert set(uncompensated) == set(grouped)
    assert {
        key for key in grouped if uncompensated[key] != grouped[key]
    } == {
        "interface_transmission_checkpoint",
        "backlash_compensation_enabled",
        "takeup_transaction_enabled",
        "engaged_gain_enabled",
        "mppi_grouped_mode_sampling",
        "mppi_engaged_gain_scenarios",
        "mppi_best_candidate_guard",
        "mppi_takeup_risk_weight",
        "reversal_scheduler_enabled",
    }
    assert uncompensated["interface_transmission_checkpoint"] == ""


def test_decoupled_tendon_prehome_holds_physical_chassis_axis():
    position = np.asarray([16.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    home = np.asarray([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    target = decoupled_tendon_prehome_target(position, home)

    assert target == pytest.approx([13.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert target[0]-target[2] == pytest.approx(position[0]-position[2])


def test_v175_no_rotation_profile_only_disables_rotation_velocity():
    baseline = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config"
         / "v175_grouped_hardware.yaml").read_text(encoding="utf-8"))
    isolated = yaml.safe_load(
        V175_NO_ROTATION_PROFILE.read_text(encoding="utf-8"))
    baseline_parameters = baseline["catheter_mppi"]["ros__parameters"]
    isolated_parameters = isolated["catheter_mppi"]["ros__parameters"]

    assert set(baseline_parameters) == set(isolated_parameters)
    changed = {
        key for key in baseline_parameters
        if baseline_parameters[key] != isolated_parameters[key]
    }
    assert changed == {"controller_velocity_max"}
    assert isolated_parameters["controller_velocity_max"] == pytest.approx(
        [10.0, 0.0, 4.5, 4.0, 25.0, 25.0])


def test_v174_no_rotation_profile_only_changes_jacobian_artifact():
    causal = yaml.safe_load(CAUSAL_V2_PROFILE.read_text(encoding="utf-8"))
    v174 = yaml.safe_load(V174_PROFILE.read_text(encoding="utf-8"))
    causal_parameters = causal["catheter_mppi"]["ros__parameters"]
    v174_parameters = v174["catheter_mppi"]["ros__parameters"]

    assert set(causal_parameters) == set(v174_parameters)
    changed = {
        key for key in causal_parameters
        if causal_parameters[key] != v174_parameters[key]
    }
    assert changed == {"jacobian_initialization_json"}
    assert v174_parameters["jacobian_initialization_json"].endswith(
        "/real_joint_local_distal_v174.json")
    assert v174_parameters["adaptation_enabled"] is False
    assert v174_parameters["controller_velocity_max"][1] == 0.0


@pytest.mark.parametrize("offsets", [
    [[0.0, 0.0, 0.0]],
    [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
])
def test_relative_sparse_targets_reject_zero_or_duplicate_offsets(
        tmp_path, offsets):
    content = AXIAL_HARDWARE_CONFIG.read_text(encoding="utf-8")
    document = yaml.safe_load(content)
    document["generator"]["offsets_mm"] = offsets
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError):
        load_sparse_point_experiment(path)


def test_history_preserving_simulation_is_an_explicit_comparison_mode():
    spec = load_sparse_point_experiment(HISTORY_SIM_CONFIG)

    assert spec.action_name.startswith("/sim/")
    assert spec.plant_reset_mode == "history_preserving"
    assert spec.plant_reset_services == ()
