import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_limits():
    """Path to the shared catheter limits YAML (lives in the automation
    package). Resolved at launch time; empty string if automation isn't
    installed, which disables the manager's joint-limit clamp."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("automation"), "config", "catheter_limits.yaml")
    except Exception:
        return ""


def generate_launch_description():
    return LaunchDescription([
        # The manager hard-clamps every forwarded command to this catheter
        # profile (bounds automation AND manual teleop). limits_file:="" disables.
        DeclareLaunchArgument("limits_file", default_value=_default_limits()),
        DeclareLaunchArgument("catheter", default_value="default"),
        Node(
            package='control_interface',
            executable='manager.py',
            name='manager',
            parameters=[{
                "limits_file": LaunchConfiguration("limits_file"),
                "catheter": LaunchConfiguration("catheter"),
            }],
        ),
        Node(
            package='control_interface',
            executable='device_serial_com.py',
            name='device_serial_com'
        )
    ])
