"""Shared-environment entry points for controller task clients."""

from catheter_control.bootstrap import run_in_venv


def catheter_target_offset():
    return run_in_venv("control_tasks.target_offset")


def catheter_tip_trajectory():
    return run_in_venv("control_tasks.trajectory_action")


def catheter_tip_trajectory_file():
    return run_in_venv("control_tasks.trajectory_file")


def catheter_sparse_point_experiment():
    return run_in_venv("control_tasks.sparse_point_experiment")


def catheter_tip_path():
    return run_in_venv("control_tasks.path_action")


def catheter_tip_path_file():
    return run_in_venv("control_tasks.path_file")


def catheter_camera_overlay():
    return run_in_venv("control_tasks.camera_overlay")
