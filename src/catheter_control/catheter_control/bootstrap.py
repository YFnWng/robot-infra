"""Start catheter-control executables in the shared Python environment.

ROS-generated console scripts use the interpreter that built the workspace.
That is normally ``/usr/bin/python3`` even though the learned controller needs
the shared ``cr-venv`` PyTorch/NumPy environment.  This module stays free of
NumPy and PyTorch imports so it can safely re-exec before loading the runtime.
"""
from __future__ import annotations

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


def _run_in_venv(module: str):
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
    if module == "catheter_control.node":
        from .node import main
    elif module == "catheter_control.validation":
        from .validation import main
    elif module == "catheter_control.sim_device":
        from .sim_device import main
    elif module == "catheter_control.sim_perception":
        from .sim_perception import main
    elif module == "catheter_control.sim_visualizer":
        from .sim_visualizer import main
    elif module == "catheter_control.sim_target":
        from .sim_target import main
    elif module == "catheter_control.sim_scenario":
        from .sim_scenario import main
    elif module == "catheter_control.target_offset":
        from .target_offset import main
    elif module == "catheter_control.trajectory_action":
        from .trajectory_action import main
    elif module == "catheter_control.trajectory_file":
        from .trajectory_file import main
    elif module == "catheter_control.sparse_point_experiment":
        from .sparse_point_experiment import main
    elif module == "catheter_control.path_action":
        from .path_action import main
    elif module == "catheter_control.path_file":
        from .path_file import main
    elif module == "catheter_control.camera_overlay":
        from .camera_overlay import main
    else:  # pragma: no cover - only fixed entry points call this helper.
        raise ValueError(f"unsupported catheter-control module: {module}")
    return main()


def catheter_mppi():
    """Console entry point for the guarded ROS controller."""
    return _run_in_venv("catheter_control.node")


def phase5_preflight():
    """Console entry point for the offline validation harness."""
    return _run_in_venv("catheter_control.validation")


def catheter_sim_device():
    """Console entry point for the isolated actuator/device plant."""
    return _run_in_venv("catheter_control.sim_device")


def catheter_sim_perception():
    """Console entry point for independent exact-model marker feedback."""
    return _run_in_venv("catheter_control.sim_perception")


def catheter_sim_visualizer():
    """Console entry point for the RViz simulation adapter."""
    return _run_in_venv("catheter_control.sim_visualizer")


def catheter_sim_target():
    """Console entry point for an axis-separated simulation target."""
    return _run_in_venv("catheter_control.sim_target")


def catheter_sim_scenario():
    """Console entry point for one scored simulation robustness trial."""
    return _run_in_venv("catheter_control.sim_scenario")


def catheter_target_offset():
    """Publish a guarded target relative to the measured hardware tip."""
    return _run_in_venv("catheter_control.target_offset")


def catheter_tip_trajectory():
    """Run the guarded time-budgeted tip-trajectory action server."""
    return _run_in_venv("catheter_control.trajectory_action")


def catheter_tip_trajectory_file():
    """Load a YAML trajectory and send it to the guarded action server."""
    return _run_in_venv("catheter_control.trajectory_file")


def catheter_sparse_point_experiment():
    """Home before each independent sparse-circle target."""
    return _run_in_venv("catheter_control.sparse_point_experiment")


def catheter_tip_path():
    """Run the guarded continuous tip-path action server."""
    return _run_in_venv("catheter_control.path_action")


def catheter_tip_path_file():
    """Load a YAML continuous path and send it to the action server."""
    return _run_in_venv("catheter_control.path_file")


def catheter_camera_overlay():
    """Render low-rate measured-marker and UKF-shape camera overlays."""
    return _run_in_venv("catheter_control.camera_overlay")
