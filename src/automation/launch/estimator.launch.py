"""Launch file for the state estimation node."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_estimation_path = os.environ.get(
        "STATE_ESTIMATION_PATH",
        os.path.join(os.path.expanduser("~"), "state_estimation"),
    )
    python_path = os.pathsep.join(filter(None, [
        os.path.dirname(os.path.abspath(state_estimation_path)),
        os.environ.get("PYTHONPATH", ""),
    ]))
    config_path = os.environ.get(
        "STATE_ESTIMATION_CONFIG",
        os.path.join(
            get_package_share_directory("automation"),
            "config", "live_coil_estimation.yaml",
        ),
    )

    return LaunchDescription([
        Node(
            package="automation",
            executable="state_estimator",
            name="state_estimator",
            output="screen",
            parameters=[{
                "config_path": config_path,
            }],
            additional_env={"PYTHONPATH": python_path},
        ),
    ])
