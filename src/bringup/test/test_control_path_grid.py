"""Non-actuating launch composition of the opt-in path time grid."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def source():
    path = Path(__file__).resolve().parents[1] / "launch/control.launch.py"
    spec = importlib.util.spec_from_file_location("control_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_semantic_stack_point_grid_with_output_interlock(monkeypatch, tmp_path):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))
    module = source()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({
        "stack_config": "hardware_path_grouped_no_takeup_no_rotation",
        "path_prediction_grid": "point", "command_output_enabled": "true",
        "record": "false", "recording_manifest_path": "",
    })
    config_root = Path(__file__).resolve().parents[2] / "catheter_control"
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(config_root))
    monkeypatch.setattr(module, "resolve_model_selection", lambda *a, **kw:
        SimpleNamespace(manifest_path="/tmp/test-manifest.json"))
    monkeypatch.setattr(module, "Node", lambda **kw: SimpleNamespace(**kw))
    nodes = module._setup(context)
    assert nodes[0].parameters[-2] == {
        "rollout_step_s": .2, "mppi_path_rollout_coarse_steps": True}
    assert nodes[2].parameters[0]["preview_duration_s"] == 1.0
    assert nodes[2].parameters[0]["position_lookahead_s"] == .6
    context.launch_configurations['point_rollout_step_override_s'] = '0.06666666666666667'
    nodes = module._setup(context)
    assert nodes[0].parameters[-3] == {'mppi_point_rollout_step_s': 1./15.}
    context.launch_configurations['horizon_steps_override'] = '12'
    nodes = module._setup(context)
    assert nodes[0].parameters[-3] == {
        'mppi_point_rollout_step_s': 1./15., 'horizon_steps': 12}
    context.launch_configurations["performance_config"] = "/tmp/legacy.yaml"
    with pytest.raises(RuntimeError, match="cannot be combined"):
        module._setup(context)


def test_grid_preserves_defaults_and_rejects_invalid_selection():
    module = source()
    assert module._path_prediction_grid({}, "") == {}
    assert module._point_rollout_override('') == {}
    assert module._point_rollout_override('', '12') == {'horizon_steps': 12}
    for value in ('0', '-1', '1.5', 'nan', 'inf'):
        with pytest.raises(ValueError):
            module._point_rollout_override('', value)
    assert module._point_rollout_override('0.06666666666666667') == {
        'mppi_point_rollout_step_s': 1./15.}
    for value in ('0', '-1', 'nan', 'inf'):
        with pytest.raises(ValueError):
            module._point_rollout_override(value)
    with pytest.raises(ValueError):
        module._path_prediction_grid({}, "arbitrary")
    for step in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            module._path_prediction_grid({"mppi_point_rollout_step_s": step}, "point")
