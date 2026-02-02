from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_interface',
            executable='manager.py',
            name='manager',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='control_interface',
            executable='device_serial_com.py',
            name='device_serial_com',
            ros_arguments=['--log-level', 'warn']
        )
    ])