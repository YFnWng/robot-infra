from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():
    state_estimation_path = os.environ.get(
        "STATE_ESTIMATION_PATH",
        os.path.join(os.path.expanduser("~"), "state_estimation"),
    )
    cr_common_path = os.environ.get(
        "CR_COMMON_PATH",
        os.path.join(os.path.expanduser("~"), "cr-common"),
    )
    cr_venv = os.environ.get(
        "CR_VENV",
        os.path.join(os.path.expanduser("~"), "cr-venv"),
    )
    cr_venv_site_packages = os.path.join(
        cr_venv,
        "lib",
        f"python{sys.version_info.major}.{sys.version_info.minor}",
        "site-packages",
    )
    estimator_python_path = os.pathsep.join(filter(None, [
        os.path.dirname(os.path.abspath(state_estimation_path)),
        os.path.abspath(cr_common_path),
        cr_venv_site_packages,
        os.environ.get("PYTHONPATH", ""),
    ]))
    config = os.path.join(
        get_package_share_directory('teleop'),
        'config',
        'params.yaml'
    )

    # params = {
    #     "joints": ['catheter_lin', 'catheter_rot', 'catheter_bend',
    #   'sheath_lin', 'sheath_rot', 'sheath_bend'],
    #     "keys": ['i', 'k', 'l', 'j', 'o', 'u',
    #   'w', 's', 'd', 'a', 'e', 'q'],
    #     "key_joint_idx": [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5],
    #     "directions": [1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1],
    #     "joint_vels": [5.0, 30.0, 30.0, 5.0, 30.0, 30.0]
    # }

    igtl_bridge = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        name="igtl_gui_bridge",
        parameters=[{
            "RIB_server_ip": "127.0.0.1",
            "RIB_port": 18944,
            "RIB_type": "server",
        }],
    )

    slicer_handler = Node(
            package='teleop',
            executable='slicer',
            name='slicer',
            parameters=[config],
            output='screen',
            emulate_tty=True
    )

    teleop_bag_recorder = Node(
        package='teleop',
        executable='bag_recorder',
        name='teleop_bag_recorder',
        parameters=[config],
        output='screen',
        emulate_tty=True,
    )

    state_estimator = Node(
        package="automation",
        executable="state_estimator",
        name="state_estimator",
        parameters=[{
            "config_path": os.path.join(
                get_package_share_directory("automation"),
                "config", "live_coil_estimation.yaml",
            ),
        }],
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONPATH": estimator_python_path},
        condition=IfCondition(LaunchConfiguration("enable_state_estimator")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_state_estimator",
            default_value="false",
            description=(
                "Launch the optional live catheter shape estimator. "
                "Requires STATE_ESTIMATION_PATH, CR_COMMON_PATH, and CR_VENV "
                "or their default ~/state_estimation, ~/cr-common, and "
                "~/cr-venv locations."
            ),
        ),
        igtl_bridge,
        slicer_handler,
        teleop_bag_recorder,
        state_estimator,
    ])
