"""Dependency-direction contract for the standalone simulation package."""
import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1] / "simulation"
FORBIDDEN_PREFIXES = (
    "automation",
    "catheter_control.simulation",
    "catheter_control.sim_",
    "catheter_control.backlash",
    "catheter_control.hardware_contract",
)


def test_simulation_implementation_uses_canonical_dependencies():
    for source in PACKAGE.glob("*.py"):
        tree = ast.parse(source.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = tuple(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                modules = (node.module or "",)
            else:
                continue
            assert not any(
                module == prefix or module.startswith(prefix)
                for module in modules
                for prefix in FORBIDDEN_PREFIXES
            ), source
