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
    "ENCODER_RADIANS_PER_COUNT": "hardware_contract",
    "HardwareContract": "hardware_contract",
    "ProjectedVelocity": "hardware_contract",
    "load_hardware_contract": "hardware_contract",
    "CatheterMppi": "mppi",
    "MppiConfig": "mppi",
    "MppiPlan": "mppi",
    "ControllerState": "lifecycle",
    "FreshnessLimits": "lifecycle",
    "GateInputs": "lifecycle",
    "readiness": "lifecycle",
    "ModelInLoopPlant": "sim_plant",
    "PlantSnapshot": "sim_plant",
    "SimulatedActuatorPlant": "sim_plant",
    "ActuatorSnapshot": "sim_plant",
    "MarkerSensorConfig": "sim_perturbations",
    "MarkerSensorModel": "sim_perturbations",
    "ActuatorConfig": "sim_perturbations",
    "ActuatorPerturbation": "sim_perturbations",
    "JacobianConfig": "sim_perturbations",
    "JacobianPerturbation": "sim_perturbations",
}


def __getattr__(name):
    module_name = _MODULE_BY_NAME.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value
