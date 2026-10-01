from pathlib import Path

import pytest

from catheter_control import bootstrap


def test_default_shared_environment():
    assert bootstrap.DEFAULT_CR_VENV == Path(
        "/home/chen-lab/Yifan/cr-venv")


def test_environment_override(monkeypatch, tmp_path):
    python = tmp_path/"bin"/"python3"
    python.parent.mkdir()
    python.touch(mode=0o755)
    monkeypatch.setenv("CR_VENV", str(tmp_path))

    assert bootstrap.configured_venv() == tmp_path
    assert bootstrap.venv_python() == python


def test_missing_environment_has_actionable_error(tmp_path):
    with pytest.raises(RuntimeError, match="set CR_VENV"):
        bootstrap.venv_python(tmp_path/"missing")


def test_simulation_entry_points_use_shared_environment(monkeypatch):
    calls = []
    monkeypatch.setattr(bootstrap, "_run_in_venv", calls.append)
    bootstrap.catheter_sim_device()
    bootstrap.catheter_sim_perception()
    bootstrap.catheter_sim_visualizer()
    bootstrap.catheter_sim_target()
    bootstrap.catheter_sim_scenario()
    assert calls == [
        "catheter_control.sim_device",
        "catheter_control.sim_perception",
        "catheter_control.sim_visualizer",
        "catheter_control.sim_target",
        "catheter_control.sim_scenario",
    ]
