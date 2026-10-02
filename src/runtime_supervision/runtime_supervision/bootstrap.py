"""Run identity capture in the same installed environment as the model."""
from __future__ import annotations

import importlib
import os
from pathlib import Path
import sys


def compute_profile():
    """Run offline profiling with the controller's established environment."""
    from catheter_control.bootstrap import run_in_venv

    return run_in_venv("runtime_supervision.compute_profile")


def runtime_identity():
    root = Path(os.environ.get(
        "CR_VENV", "/home/chen-lab/Yifan/cr-venv")).expanduser().absolute()
    python = root / "bin" / "python3"
    if Path(sys.prefix).absolute() != root:
        if not python.is_file() or not os.access(python, os.X_OK):
            raise RuntimeError(f"runtime identity requires CR_VENV at {root}")
        environment = os.environ.copy()
        environment.update({
            "VIRTUAL_ENV": str(root),
            "PYTHONNOUSERSITE": "1",
            "PATH": str(root / "bin") + os.pathsep + environment.get("PATH", ""),
        })
        os.execve(
            str(python),
            [str(python), "-m", "runtime_supervision.runtime_identity",
             *sys.argv[1:]],
            environment)
    return importlib.import_module(
        "runtime_supervision.runtime_identity").main()
