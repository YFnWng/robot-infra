"""Canonical catheter-control package-boundary contracts."""
import ast
from pathlib import Path

import catheter_control


PACKAGE = Path(__file__).resolve().parents[1] / "catheter_control"
CANONICAL_DIRECTORIES = (
    PACKAGE / "planning",
    PACKAGE / "transmission",
    PACKAGE / "safety",
    PACKAGE / "orchestration",
)
RETIRED_ROOT_MODULES = frozenset({
    "backlash",
    "causal_schedule",
    "compute_device",
    "configuration",
    "engaged_gain",
    "hardware_contract",
    "lifecycle",
    "mppi",
    "path_tracking",
    "reversal_scheduler",
    "timing",
    "tracking",
    "trajectory",
    "validation",
})
RETIRED_SIMULATION_EXPORTS = (
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
)


def _absolute_imports(source: Path):
    tree = ast.parse(source.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            yield node.module or ""


def test_root_compatibility_modules_are_retired():
    assert not (PACKAGE / "_compat.py").exists()
    for module in RETIRED_ROOT_MODULES:
        assert not (PACKAGE / f"{module}.py").exists()


def test_canonical_modules_do_not_import_retired_paths():
    retired_paths = tuple(
        f"catheter_control.{module}" for module in RETIRED_ROOT_MODULES)
    for directory in CANONICAL_DIRECTORIES:
        for source in directory.glob("*.py"):
            for imported in _absolute_imports(source):
                assert not any(
                    imported == retired or imported.startswith(retired + ".")
                    for retired in retired_paths
                ), source


def test_controller_package_no_longer_advertises_simulation_symbols():
    assert all(
        name not in catheter_control.__all__
        and not hasattr(catheter_control, name)
        for name in RETIRED_SIMULATION_EXPORTS
    )
