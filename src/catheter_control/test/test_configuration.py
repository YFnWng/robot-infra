from pathlib import Path

import pytest
import yaml

from catheter_control.orchestration.configuration import (
    ACTIVE_CONTROLLER_PARAMETERS, locate_stack, resolve_stack)


CONFIG = Path(__file__).resolve().parents[1] / "config"


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
    expected = legacy["catheter_mppi"]["ros__parameters"]
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
