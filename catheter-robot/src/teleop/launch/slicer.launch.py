from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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
        parameters=[config]
    )

    slicer_handler = Node(
            package='teleop',
            executable='slicer',
            name='slicer',
            parameters=[config],
            output='screen',
    )

    return LaunchDescription([
        igtl_bridge,
        slicer_handler
    ])
