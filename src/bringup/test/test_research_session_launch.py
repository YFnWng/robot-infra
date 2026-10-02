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


def test_composition_has_one_recorder_and_output_disabled(tmp_path, monkeypatch):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))
    source = module()
    registration = tmp_path / "registration.json"
    registration.write_text("{}")
    camera = tmp_path / "camera.yaml"
    camera.write_text("camera: {resolution: HD720, fps: 30}\n")
    camera_python = tmp_path / "venv" / "bin" / "python"
    camera_python.parent.mkdir(parents=True)
    camera_python.symlink_to(sys.executable)
    context = LaunchContext()
    context.launch_configurations.update({
        "video": "true", "command_output_enabled": "false",
        "camera_config": str(camera), "registration_file": str(registration),
        "stack_config": "hardware_grouped_no_rotation_farther_tendon_12",
        "startup_timeout_s": "60", "session_root": str(tmp_path / "sessions"),
        "session_label": "attribution_grouped_n512_repeat01",
        "shape_tracking_root": str(tmp_path),
        "camera_python": str(camera_python),
    })
    monkeypatch.setattr(source, "Node", lambda **kw: SimpleNamespace(**kw))
    actions = source._setup(context)
    assert len(actions) == 1
    import json
    manifest_path = Path(actions[0].arguments[1])
    manifest = json.loads(manifest_path.read_text())
    assert manifest["state"] == "allocated"
    assert not manifest["command_output_enabled"]
    assert "record:=false" in manifest["commands"]["controller"]
    assert "command_output_enabled:=false" in manifest["commands"]["controller"]
    assert manifest["commands"]["bag"].count("record") == 1
    assert "recording_enabled:=true" in manifest["commands"]["camera"]
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
