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


def test_controller_entry_points_use_canonical_modules(monkeypatch):
    calls = []
    monkeypatch.setattr(bootstrap, "run_in_venv", calls.append)

    bootstrap.catheter_mppi()
    bootstrap.phase5_preflight()

    assert calls == [
        "catheter_control.node",
        "catheter_control.safety.validation",
    ]
