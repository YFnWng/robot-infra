"""Shared-environment entry points for the isolated simulation stack."""

from catheter_control.bootstrap import run_in_venv


def catheter_sim_device():
    return run_in_venv("simulation.sim_device")


def catheter_sim_perception():
    return run_in_venv("simulation.sim_perception")


def catheter_sim_visualizer():
    return run_in_venv("simulation.sim_visualizer")


def catheter_sim_target():
    return run_in_venv("simulation.sim_target")


def catheter_sim_scenario():
    return run_in_venv("simulation.sim_scenario")
