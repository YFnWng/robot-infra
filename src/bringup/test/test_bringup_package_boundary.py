"""Dependency and ownership contracts for canonical bringup launch files."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch"
LIMITS = ROOT.parent / "control_interface" / "config" / "catheter_limits.yaml"


def test_bringup_launches_do_not_import_automation():
    for source in LAUNCH.glob("*.launch.py"):
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


def test_shared_limits_are_owned_by_control_interface():
    assert LIMITS.is_file()
    for source in LAUNCH.glob("*.launch.py"):
        text = source.read_text(encoding="utf-8")
        assert "get_package_share_directory(\"automation\")" not in text
