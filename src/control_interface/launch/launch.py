import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_limits():
    """Resolve the shared limits installed by the command authority package.

    An absent contract leaves the default empty so the manager fails closed.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("control_interface"), "config", "catheter_limits.yaml")
    except Exception:
        return ""


def generate_launch_description():
    return LaunchDescription([
        # The manager hard-clamps every forwarded command to this catheter
        # profile (bounds automation AND manual teleop). Missing limits fail closed.
        DeclareLaunchArgument("limits_file", default_value=_default_limits()),
        DeclareLaunchArgument("catheter", default_value="imricor_test"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("feedback_timeout_s", default_value="0.25"),
        DeclareLaunchArgument("command_max_age_s", default_value="0.10"),
        DeclareLaunchArgument(
            "limit_recovery_max_violation", default_value="0.25"),
        DeclareLaunchArgument(
            "limit_recovery_interior_margin", default_value="0.10"),
        DeclareLaunchArgument(
            "limit_recovery_timeout_s", default_value="0.50"),
        DeclareLaunchArgument(
            "limit_recovery_command_rate_hz", default_value="100.0"),
        DeclareLaunchArgument(
            "limit_recovery_outward_tolerance", default_value="0.02"),
        Node(
            package='control_interface',
            executable='manager.py',
            name='manager',
            parameters=[{
                "limits_file": LaunchConfiguration("limits_file"),
                "catheter": LaunchConfiguration("catheter"),
                "feedback_timeout_s": ParameterValue(
                    LaunchConfiguration("feedback_timeout_s"),
                    value_type=float),
                "command_max_age_s": ParameterValue(
                    LaunchConfiguration("command_max_age_s"),
                    value_type=float),
                "limit_recovery_max_violation": ParameterValue(
                    LaunchConfiguration("limit_recovery_max_violation"),
                    value_type=float),
                "limit_recovery_interior_margin": ParameterValue(
                    LaunchConfiguration("limit_recovery_interior_margin"),
                    value_type=float),
                "limit_recovery_timeout_s": ParameterValue(
                    LaunchConfiguration("limit_recovery_timeout_s"),
                    value_type=float),
                "limit_recovery_command_rate_hz": ParameterValue(
                    LaunchConfiguration("limit_recovery_command_rate_hz"),
                    value_type=float),
                "limit_recovery_outward_tolerance": ParameterValue(
                    LaunchConfiguration("limit_recovery_outward_tolerance"),
                    value_type=float),
            }],
        ),
        Node(
            package='control_interface',
            executable='device_serial_com.py',
            name='device_serial_com',
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "command_max_age_s": ParameterValue(
                    LaunchConfiguration("command_max_age_s"),
                    value_type=float),
            }],
        )
    ])
