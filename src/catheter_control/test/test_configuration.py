from pathlib import Path

import pytest
import yaml

from catheter_control.orchestration.configuration import (
    ACTIVE_CONTROLLER_PARAMETERS, locate_stack, resolve_stack)


CONFIG = Path(__file__).resolve().parents[1] / "config"


@pytest.mark.parametrize('mode', ['plain', 'takeup'])
def test_adapj_position_profiles_reuse_existing_layers(mode):
    def resolve(name):
        return resolve_stack(locate_stack(name, CONFIG), config_root=CONFIG,
            allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
            expected_node_name='catheter_mppi').parameters
    original = resolve(f'hardware_adapj_{mode}_twoaxis')
    position = resolve(f'hardware_adapj_{mode}_position_twoaxis')
    assert position['control_method'] == 'adapj'
    assert position['mppi_execution_mode'] == 'position_stream'
    assert position['task_gc_enabled'] is True
    assert {key for key in original if original[key] != position[key]} <= {
        'mppi_execution_mode', 'task_gc_enabled'}


def test_synchronized_circle_reuses_baseline_and_existing_yz_geometry():
    stack = resolve_stack(locate_stack(
        'hardware_path_grouped_takeup_synchronized_rotation_circle', CONFIG),
        config_root=CONFIG, allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
        expected_node_name='catheter_mppi')
    assert stack.parameters['synchronized_sheath_rotation'] is True
    assert stack.parameters['controller_velocity_max'][1] == 25.
    assert stack.parameters['task_gc_enabled'] is True
    assert stack.parameters['mppi_grouped_mode_sampling'] is True
    assert stack.parameters['backlash_compensation_enabled'] is True
    assert not stack.parameters.get('engagement_control_enabled', False)


@pytest.mark.parametrize('name', [
    'hardware_engagement_twoaxis',
    'hardware_path_plain_no_rotation',
    'hardware_path_grouped_takeup_no_rotation',
])
def test_session_profiles_enable_shared_bounded_task_gc(name):
    stack = resolve_stack(locate_stack(name, CONFIG), config_root=CONFIG,
                          allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
                          expected_node_name='catheter_mppi')
    assert stack.parameters['task_gc_enabled'] is True
    assert stack.parameters['task_gc_maximum_s'] == 300.0
    assert stack.parameters['task_gc_growth_mb'] == 256.0
    assert stack.parameters['backlash_minimum_distal_marker_deformation_mm'] == 0.3


@pytest.mark.parametrize("row", ["plain", "plain_takeup", "grouped_takeup", "grouped_no_takeup"])
def test_twoaxis_path_profiles_reuse_model_and_disable_rotation(row):
    stack = resolve_stack(locate_stack(f"hardware_path_{row}_no_rotation", CONFIG),
                          config_root=CONFIG,
                          allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
                          expected_node_name="catheter_mppi")
    assert stack.parameters["controller_velocity_max"][1] == 0
    assert stack.parameters["horizon_steps"] == 4
    assert stack.parameters["rollout_step_s"] == .04
    assert stack.parameters["samples"] == 512
    assert stack.parameters["engaged_gain_enabled"] is False
    assert not stack.parameters.get("backlash_tendon_takeup_joint_range_enabled", False)


def test_g5_profile_is_separate_and_explicit():
    stack=resolve_stack(locate_stack('hardware_engagement_twoaxis',CONFIG),
        config_root=CONFIG,allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
        expected_node_name='catheter_mppi')
    assert stack.parameters['engagement_control_enabled'] is True
    assert stack.parameters['backlash_tendon_takeup_joint_range_enabled'] is True
    assert stack.parameters['planner_process_enabled'] is True
    assert stack.parameters['samples']==512
    assert stack.parameters['estimator_device']=='cpu'
    assert not stack.parameters['engaged_gain_enabled']
    assert not stack.parameters['mppi_engaged_gain_scenarios']
    assert stack.parameters['controller_velocity_max'][1]==0


@pytest.mark.parametrize(
    ("stack_name", "legacy_name"),
    [
        (
            "hardware_grouped_no_rotation_farther_tendon_12",
            "v175_grouped_hardware_no_rotation.yaml",
        ),
        (
            "hardware_plain_with_takeup_no_rotation_farther_tendon_12",
            "v175_plain_takeup_hardware_no_rotation.yaml",
        ),
        (
            "hardware_plain_no_rotation_farther_tendon_12",
            "v171_plain_hardware_no_rotation.yaml",
        ),
        (
            "hardware_grouped_all_axes",
            "v175_grouped_hardware.yaml",
        ),
    ],
)
def test_semantic_stack_matches_legacy_profile(stack_name, legacy_name):
    stack_path = locate_stack(stack_name, CONFIG)
    resolved = resolve_stack(
        stack_path,
        config_root=CONFIG,
        allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
        expected_node_name="catheter_mppi",
    )
    legacy = yaml.safe_load((CONFIG / legacy_name).read_text())
    expected = dict(legacy["catheter_mppi"]["ros__parameters"])
    # Flat historical profiles omit the newly enabled shared node defaults.
    expected.update(task_gc_enabled=True, task_gc_maximum_s=300.0,
                    task_gc_growth_mb=256.0,
                    backlash_minimum_distal_marker_deformation_mm=0.3)
    expected["backlash_distal_confirmation_release_ratio"] = 0.75
    assert resolved.parameters == expected
    assert legacy_name in resolved.compatibility_aliases


def _write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value), encoding="utf-8")


def _stack(tmp_path, layers):
    path = tmp_path / "stacks" / "test.yaml"
    _write(
        path,
        {
            "schema_version": 1,
            "name": "test.stack",
            "node_name": "catheter_mppi",
            "layers": layers,
        },
    )
    return path


def test_stack_rejects_unknown_parameter(tmp_path):
    _write(
        tmp_path / "controller.yaml",
        {"catheter_mppi": {"ros__parameters": {"typo": 1}}},
    )
    with pytest.raises(ValueError, match="unknown parameters: typo"):
        resolve_stack(
            _stack(tmp_path, {"controller": "controller.yaml"}),
            config_root=tmp_path,
            allowed_parameters={"known"},
        )


def test_stack_rejects_parameter_owned_by_two_layers(tmp_path):
    profile = {"catheter_mppi": {"ros__parameters": {"samples": 32}}}
    _write(tmp_path / "controller.yaml", profile)
    _write(tmp_path / "performance.yaml", profile)
    with pytest.raises(ValueError, match="duplicates parameters: samples"):
        resolve_stack(
            _stack(
                tmp_path,
                {
                    "controller": "controller.yaml",
                    "performance": "performance.yaml",
                },
            ),
            config_root=tmp_path,
            allowed_parameters={"samples"},
        )


def test_stack_rejects_hardware_output_interlock(tmp_path):
    _write(
        tmp_path / "controller.yaml",
        {
            "catheter_mppi": {
                "ros__parameters": {"command_output_enabled": True}
            }
        },
    )
    with pytest.raises(ValueError, match="protected parameter"):
        resolve_stack(
            _stack(tmp_path, {"controller": "controller.yaml"}),
            config_root=tmp_path,
            allowed_parameters={"command_output_enabled"},
        )


def test_stack_rejects_path_escape(tmp_path):
    with pytest.raises(ValueError, match="stay inside"):
        resolve_stack(
            _stack(tmp_path, {"controller": "../outside.yaml"}),
            config_root=tmp_path,
            allowed_parameters=set(),
        )


def test_stack_accepts_colcon_style_package_data_symlink(tmp_path):
    source = tmp_path / "source"
    install = tmp_path / "install"
    _write(
        source / "controller.yaml",
        {"catheter_mppi": {"ros__parameters": {"samples": 32}}},
    )
    install.mkdir()
    (install / "controller.yaml").symlink_to(source / "controller.yaml")
    resolved = resolve_stack(
        _stack(install, {"controller": "controller.yaml"}),
        config_root=install,
        allowed_parameters={"samples"},
    )
    assert resolved.parameters == {"samples": 32}


def test_manifest_contains_resolved_sources_and_references():
    resolved = resolve_stack(
        locate_stack(
            "hardware_grouped_no_rotation_farther_tendon_12", CONFIG),
        config_root=CONFIG,
        allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
        expected_node_name="catheter_mppi",
    )
    manifest = resolved.as_manifest()
    assert manifest["name"] == (
        "hardware.grouped.no_rotation.farther_tendon_12")
    assert manifest["references"]["experiment"].endswith(
        "experiments/sparse_points/"
        "farther_tendon_12_grouped.yaml")
    assert manifest["parameter_sources"]["samples"].endswith(
        "performance/gpu_512.yaml")


def test_simulation_semantic_stack_resolves_references():
    resolved = resolve_stack(
        locate_stack("simulation_grouped_circle", CONFIG),
        config_root=CONFIG,
        allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
        expected_node_name="sim_catheter_mppi",
    )
    assert resolved.name == "simulation.grouped.circle"
    assert resolved.references["experiment"].name == "circle_trajectory_sim.yaml"
    assert resolved.references["visualization"].name == "simulation.rviz"


def test_historical_artifact_selecting_profiles_are_explicitly_legacy():
    names = (
        "causal_v2_shadow.yaml",
        "causal_v2_fixed_hardware.yaml",
        "v174_fixed_hardware_no_rotation.yaml",
    )
    for name in names:
        assert not (CONFIG / name).exists()
        assert (CONFIG / "legacy" / name).is_file()

    for stack in (CONFIG / "stacks").glob("*.yaml"):
        content = stack.read_text(encoding="utf-8")
        assert all(name not in content for name in names), stack
