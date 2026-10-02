"""Dependency-direction contract for the standalone experiments package."""
import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1] / "experiments"


def test_experiments_implementation_does_not_import_automation():
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
                module == "automation" or module.startswith("automation.")
                for module in modules
            ), source
