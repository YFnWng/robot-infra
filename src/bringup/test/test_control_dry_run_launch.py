"""Launch composition of the reduced-filter dry run: no actual launch, devices or hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def setup(monkeypatch, tmp_path, **configurations):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))
    path = Path(__file__).resolve().parents[1]/"launch/control.launch.py"
    spec = importlib.util.spec_from_file_location("control_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({"record": "false", "recording_manifest_path": "", **configurations})
    config_root = Path(__file__).resolve().parents[2]/"catheter_control"
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(config_root))
    monkeypatch.setattr(module, "resolve_model_selection", lambda *a, **kw:
                        SimpleNamespace(manifest_path="/tmp/test-manifest.json", compatibility_alias=None))
    monkeypatch.setattr(module, "Node", lambda **kw: SimpleNamespace(**kw))
    return module._setup(context)


def test_dry_run_stack_selects_its_executable_and_source_tree(monkeypatch, tmp_path):
    manifest = tmp_path/"session"/"controller_manifest.json"
    manifest.parent.mkdir()
    nodes = setup(monkeypatch, tmp_path, stack_config="hardware_reduced_filter_dry_run_no_rotation",
                  command_output_enabled="false", recording_manifest_path=str(manifest))
    node = nodes[0]
    assert node.executable == "catheter_mppi_reduced_dry_run" and node.name == "catheter_mppi"
    assert node.additional_env["PYTHONPATH"].startswith("/home/chen-lab/Yifan")
    assert (node.additional_env["OPENBLAS_NUM_THREADS"], node.additional_env["OMP_NUM_THREADS"]) == ("1", "1")
    assert {"reduced_filter_report_dir": str(manifest.parent)} in node.parameters


def test_dry_run_refuses_command_output(monkeypatch, tmp_path):
    with pytest.raises(RuntimeError, match="cannot enable command output"):
        setup(monkeypatch, tmp_path, stack_config="hardware_reduced_filter_dry_run_no_rotation",
              command_output_enabled="true")


def test_october_stack_keeps_its_executable_and_environment(monkeypatch, tmp_path):
    node = setup(monkeypatch, tmp_path, stack_config="hardware_grouped_no_rotation_farther_tendon_12",
                 command_output_enabled="false")[0]
    assert node.executable == "catheter_mppi" and node.additional_env == {}
    assert not any("reduced_filter_report_dir" in item for item in node.parameters if isinstance(item, dict))


def test_commanding_reduced_stack_runs_catheter_mppi_with_the_research_environment(monkeypatch, tmp_path):
    for output in ("false", "true"):   # command output stays launch-controlled; only the dry run refuses it
        node = setup(monkeypatch, tmp_path, stack_config="hardware_reduced_filter_no_rotation_half_speed",
                     command_output_enabled=output)[0]
        assert node.executable == "catheter_mppi" and node.name == "catheter_mppi"
        assert node.additional_env["PYTHONPATH"].startswith("/home/chen-lab/Yifan")
        assert (node.additional_env["OPENBLAS_NUM_THREADS"], node.additional_env["OMP_NUM_THREADS"]) == ("1", "1")
        assert not any("reduced_filter_report_dir" in item for item in node.parameters if isinstance(item, dict))
