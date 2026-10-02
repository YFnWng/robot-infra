"""Start catheter-control executables in the shared Python environment.

ROS-generated console scripts use the interpreter that built the workspace.
That is normally ``/usr/bin/python3`` even though the learned controller needs
the shared ``cr-venv`` PyTorch/NumPy environment.  This module stays free of
NumPy and PyTorch imports so it can safely re-exec before loading the runtime.
"""
from __future__ import annotations

import importlib
import os
from pathlib import Path
import sys


DEFAULT_CR_VENV = Path("/home/chen-lab/Yifan/cr-venv")


def configured_venv() -> Path:
    """Return the configured shared environment directory."""
    return Path(os.environ.get("CR_VENV", str(DEFAULT_CR_VENV))).expanduser()


def venv_python(venv: Path | None = None) -> Path:
    """Resolve and validate the Python interpreter used by the controller."""
    root = configured_venv() if venv is None else Path(venv).expanduser()
    python = root/"bin"/"python3"
    if not python.is_file() or not os.access(python, os.X_OK):
        raise RuntimeError(
            f"catheter control requires the Python 3.10 environment at "
            f"{root}; set CR_VENV to override it")
    return python


def _in_configured_venv(root: Path) -> bool:
    # Do not compare resolved interpreter paths: venv/bin/python3 commonly
    # resolves to /usr/bin/python3 even while sys.prefix correctly names the
    # active environment.
    return Path(sys.prefix).absolute() == root.absolute()


def run_in_venv(module: str):
    root = configured_venv().absolute()
    python = venv_python(root)
    if not _in_configured_venv(root):
        environment = os.environ.copy()
        environment["VIRTUAL_ENV"] = str(root)
        environment["PYTHONNOUSERSITE"] = "1"
        environment["PATH"] = (
            str(root/"bin")+os.pathsep+environment.get("PATH", ""))
        os.execve(
            str(python),
            [str(python), "-m", module, *sys.argv[1:]],
            environment)
    return importlib.import_module(module).main()


def catheter_mppi():
    """Console entry point for the guarded ROS controller."""
    return run_in_venv("catheter_control.node")


def control_shadow_worker():
    """Console entry point for the non-commanding shadow adapter."""
    return run_in_venv("catheter_control.orchestration.shadow_worker")


def phase5_preflight():
    """Console entry point for the offline validation harness."""
    return run_in_venv("catheter_control.safety.validation")


def phase5_conformance():
    """Console entry point for deterministic non-actuating replay."""
    return run_in_venv("catheter_control.safety.conformance")
