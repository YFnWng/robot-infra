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


def test_phase4_shadow_is_opt_in_and_non_commanding():
    source = (LAUNCH / "control.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    declared_defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (isinstance(function, ast.Name)
                and function.id == "DeclareLaunchArgument"):
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if (keyword.arg == "default_value"
                    and isinstance(keyword.value, ast.Constant)):
                declared_defaults[node.args[0].value] = keyword.value.value
    assert declared_defaults["start_cpp_shadow"] == "false"
    assert declared_defaults["start_shadow_worker"] == "false"
    assert 'package="control_cpp"' in source
    assert 'executable="control_shadow"' in source
    assert 'executable="control_shadow_worker"' in source


def test_cpp_shadow_has_no_manager_or_device_command_surface():
    cpp = ROOT.parent / "control_cpp" / "src" / "control_shadow_node.cpp"
    text = cpp.read_text(encoding="utf-8")
    forbidden = (
        '"/teleop/control"',
        '"/manager/control"',
        '"/device/command"',
        "control_interface::msg::ControlStream",
        "control_interface::srv::DeviceCmd",
        "SET_ZERO",
    )
    assert all(token not in text for token in forbidden)
    assert "command_publisher_present\", \"false" in text
    assert "command_output_enabled\", \"false" in text
