from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    state_estimation_path = os.environ.get(
        "STATE_ESTIMATION_PATH",
        os.path.join(os.path.expanduser("~"), "state_estimation"),
    )
    estimator_python_path = os.pathsep.join(filter(None, [
        os.path.dirname(os.path.abspath(state_estimation_path)),
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
    #     "joint_vels": [5.0, 30.0, 30.0, 5.0, 30.0, 1.0]
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

    tracking_remappings = [
        ("IGTL_STRING_IN", "/tracking/igtl/string_in"),
        ("IGTL_STRING_OUT", "/tracking/igtl/string_out"),
        ("IGTL_TRANSFORM_IN", "/tracking/igtl/transform_in"),
        ("IGTL_TRANSFORM_OUT", "/tracking/igtl/transform_out"),
        ("IGTL_POINT_IN", "/tracking/igtl/point_in"),
        ("IGTL_POINT_OUT", "/tracking/igtl/point_out"),
        ("IGTL_POSEARRAY_IN", "/tracking/igtl/posearray_in"),
        ("IGTL_POSEARRAY_OUT", "/tracking/igtl/posearray_out"),
    ]
    tracking_bridge = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        name="igtl_tracking_bridge",
        parameters=[{
            "RIB_server_ip": "127.0.0.1",
            "RIB_port": 18945,
            "RIB_type": "server",
        }],
        remappings=tracking_remappings,
    )

    slicer_handler = Node(
            package='teleop',
            executable='slicer',
            name='slicer',
            parameters=[config],
            output='screen',
            emulate_tty=True
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
    )

    return LaunchDescription([
        igtl_bridge,
        tracking_bridge,
        slicer_handler,
        state_estimator,
    ])
