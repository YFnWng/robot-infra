"""Closed-loop catheter control utilities.

Imports are lazy so the system-Python console shim can select ``cr-venv``
before importing NumPy and PyTorch.
"""

from importlib import import_module

__all__ = [
    "ENCODER_RADIANS_PER_COUNT",
    "HardwareContract",
    "ProjectedVelocity",
    "load_hardware_contract",
    "CatheterMppi",
    "MppiConfig",
    "MppiPlan",
    "ControllerState",
    "FreshnessLimits",
    "GateInputs",
    "readiness",
    "ModelInLoopPlant",
    "PlantSnapshot",
    "SimulatedActuatorPlant",
    "ActuatorSnapshot",
    "MarkerSensorConfig",
    "MarkerSensorModel",
    "ActuatorConfig",
    "ActuatorPerturbation",
    "JacobianConfig",
    "JacobianPerturbation",
]


_MODULE_BY_NAME = {
    "ENCODER_RADIANS_PER_COUNT": "safety.hardware_contract",
    "HardwareContract": "safety.hardware_contract",
    "ProjectedVelocity": "safety.hardware_contract",
    "load_hardware_contract": "safety.hardware_contract",
    "CatheterMppi": "planning.mppi",
    "MppiConfig": "planning.mppi",
    "MppiPlan": "planning.mppi",
    "ControllerState": "safety.lifecycle",
    "FreshnessLimits": "safety.lifecycle",
    "GateInputs": "safety.lifecycle",
    "readiness": "safety.lifecycle",
    "ModelInLoopPlant": "simulation.sim_plant",
    "PlantSnapshot": "simulation.sim_plant",
    "SimulatedActuatorPlant": "simulation.sim_plant",
    "ActuatorSnapshot": "simulation.sim_plant",
    "MarkerSensorConfig": "simulation.sim_perturbations",
    "MarkerSensorModel": "simulation.sim_perturbations",
    "ActuatorConfig": "simulation.sim_perturbations",
    "ActuatorPerturbation": "simulation.sim_perturbations",
    "JacobianConfig": "simulation.sim_perturbations",
    "JacobianPerturbation": "simulation.sim_perturbations",
}


def __getattr__(name):
    module_name = _MODULE_BY_NAME.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value
