"""Recording composition tests: no actual launch, cameras, or device links."""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest


ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location(
        "research_session_launch", ROOT / "launch" / "research_session.launch.py")
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


@pytest.mark.parametrize("estimator_device,estimator_dtype", [("", ""), ("cpu", "float64")])
@pytest.mark.parametrize("planner_process", ["", "false", "true"])
@pytest.mark.parametrize("temperature", ["", "0.3"])
@pytest.mark.parametrize("shadow", ["false", "true"])
def test_composition_has_one_recorder_and_output_disabled(
        tmp_path, monkeypatch, estimator_device, estimator_dtype, planner_process,
        temperature, shadow):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))
    source = module()
    registration = tmp_path / "registration.json"
    registration.write_text("{}")
    registration_config = tmp_path / "registration_config.yaml"
    registration_config.write_text("units: mm\n")
    camera = tmp_path / "camera.yaml"
    camera.write_text("camera: {resolution: HD720, fps: 30}\n")
    camera_python = tmp_path / "venv" / "bin" / "python"
    camera_python.parent.mkdir(parents=True)
    camera_python.symlink_to(sys.executable)
    context = LaunchContext()
    context.launch_configurations.update({
        "video": "true", "command_output_enabled": "false",
        "point_rollout_step_override_s": "0.06666666666666667",
        "horizon_steps_override": "12",
        "camera_config": str(camera), "registration_file": str(registration),
        "registration_config": str(registration_config), "registration_frames": "20",
        "stack_config": "hardware_grouped_no_rotation_farther_tendon_12",
        "startup_timeout_s": "60", "session_root": str(tmp_path / "sessions"),
        "session_label": "attribution_grouped_n512_repeat01",
        "shape_tracking_root": str(tmp_path),
        "camera_python": str(camera_python),
        "estimator_device": estimator_device, "estimator_dtype": estimator_dtype,
        "planner_process_enabled": planner_process,
        "mppi_temperature": temperature,
        "path_prediction_grid": "point",
        "model_manifest": "/tmp/explicit-engagement-manifest.json",
        "camera_stall_diagnostics": "false", "fused_timestamp_policy": "newest",
        "reduced_filter_shadow": shadow, "research_code_root": "/home/chen-lab/Yifan",
    })
    monkeypatch.setattr(source, "Node", lambda **kw: SimpleNamespace(**kw))
    actions = source._setup(context)
    assert len(actions) == 1
    import json
    manifest_path = Path(actions[0].arguments[1])
    manifest = json.loads(manifest_path.read_text())
    assert manifest["state"] == "allocated"
    assert not manifest["command_output_enabled"]
    assert manifest["estimator_device"] == estimator_device
    assert manifest["planner_process_enabled"] == planner_process
    assert manifest["mppi_temperature"] == temperature
    assert manifest["path_prediction_grid"] == "point"
    assert "path_prediction_grid:=point" in manifest["commands"]["controller"]
    assert manifest['point_rollout_step_override_s'] == '0.06666666666666667'
    assert 'point_rollout_step_override_s:=0.06666666666666667' in manifest['commands']['controller']
    camera_command = manifest['commands']['camera']
    assert 'fused_timestamp_policy:=newest' in camera_command  # defaults unchanged
    assert 'stall_diagnostics_enabled:=false' in camera_command
    assert manifest['horizon_steps_override'] == '12'
    assert 'horizon_steps_override:=12' in manifest['commands']['controller']
    for name, value in (("estimator_device", estimator_device),
                        ("estimator_dtype", estimator_dtype),
                        ("planner_process_enabled", planner_process),
                        ("mppi_temperature", temperature)):
        if value:
            assert f"{name}:={value}" in manifest["commands"]["controller"]
        else:
            assert not any(argument.startswith(f"{name}:=")
                           for argument in manifest["commands"]["controller"])
    assert "record:=false" in manifest["commands"]["controller"]
    assert "model_manifest:=/tmp/explicit-engagement-manifest.json" in manifest['commands']['controller']
    assert "command_output_enabled:=false" in manifest["commands"]["controller"]
    assert manifest["commands"]["bag"].count("record") == 1
    assert manifest["reduced_filter_shadow"] == (shadow == "true")
    if shadow == "true":
        observer = manifest["commands"]["reduced_filter_shadow"]
        assert observer[0] == "env" and observer[1].startswith("PYTHONPATH=/home/chen-lab/Yifan")
        assert observer[2:4] == ["OPENBLAS_NUM_THREADS=1", "OMP_NUM_THREADS=1"]   # single-threaded pools
        assert observer[4:8] == ["ros2", "run", "runtime_supervision", "reduced_filter_shadow"]
        assert "PYTHONPATH" not in " ".join(manifest["commands"]["controller"])
        assert "NUM_THREADS" not in " ".join(manifest["commands"]["camera"]+manifest["commands"]["controller"])
        assert observer[observer.index("--outdir")+1].startswith(manifest["bag_directory"].rsplit("/", 1)[0])
    else:
        assert "reduced_filter_shadow" not in manifest["commands"]
    assert "recording_enabled:=true" in manifest["commands"]["camera"]
    assert "registration_refresh_enabled:=true" in manifest["commands"]["camera"]
    assert "registration_frames:=20" in manifest["commands"]["camera"]
    assert not any('registration_file:=' in item for item in manifest['commands']['camera'])
    assert manifest['registration_policy'] == 'session_charuco_independent_per_rig'
    assert manifest["commands"]["camera"][0] == str(camera_python)
    assert not Path(manifest["video_directory"]).exists()
    assert not Path(manifest["bag_directory"]).exists()
    assert "session_status" in " ".join(manifest["record_topics"])


def test_invalid_boolean_fails_closed():
    with pytest.raises(ValueError):
        module()._boolean("yes")


def test_defaults_never_start_motor_interface_or_task():
    text = (ROOT / "launch" / "research_session.launch.py").read_text()
    assert '"command_output_enabled", default_value="false"' in text
    assert '"video", default_value="true"' in text
    assert "SET_ZERO" not in text
    assert '"qualify_driver_power"' not in text
    assert '"catheter_sparse_point_experiment"' not in text
