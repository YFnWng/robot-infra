"""Launch file for the state estimation node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.environ.get(
        "STATE_ESTIMATION_CONFIG",
        os.path.join(
            os.path.dirname(__file__),
            "..", "..", "..", "..", "..",
            "state_estimation", "configs", "catheter_ablation.yaml",
        ),
    )

    return LaunchDescription([
        Node(
            package="automation",
            executable="state_estimator",
            name="state_estimator",
            output="screen",
            parameters=[{"config_path": config_path}],
        ),
    ])
