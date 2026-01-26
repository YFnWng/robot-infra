
from control_interface.msg import TeleopIntents
import ros2_igtl_bridge.msg
import rclpy
from rclpy.node import Node


msg = """
This node takes commands from slicer through OpenIGTLink and publishes them
as TeleopIntents messages. 

CTRL-C to quit
"""

class TeleopSlicer(Node):

    def __init__(self):
        super().__init__(
            'slicer',
            automatically_declare_parameters_from_overrides=True
            )

        # --- parameters ---
        # read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.joint_names = self.get_parameter('joint_names').value
        self.key_bindings = self.get_parameter('key_bindings').value
        self.vel = self.get_parameter('joint_vels').value

        self.add_on_set_parameters_callback(
            self._on_parameter_update
        )

        # --- message type ---
        self.teleop_msg = TeleopIntents()
        self.teleop_msg.source = "slicer"

        # --- subscriptions ---
        self.string_sub = self.create_subscription(
            ros2_igtl_bridge.msg.String,
            '/IGTL_STRING_OUT',
            self.string_callback,
            10
        )
        self.string_sub

        self.point_sub = self.create_subscription(
            ros2_igtl_bridge.msg.PointArray,
            '/IGTL_POINT_OUT',
            self.point_callback,
            10
        )
        self.point_sub

        # --- publisher ---
        self.pub = self.create_publisher(TeleopIntents, 'teleop', 10)

        # --- state ---
        # self.x = self.y = self.z = self.th = 0.0
        self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_NONE

        # Currently pressed keys
        # self.pressed_keys = set()

        # Spin publishing thread
        # self.timer = self.create_timer(0.05, self.publish_teleop)  # 20 Hz

    # Set target points or velocities
    def point_callback(self, msg):
        if msg.name == "jv":
            self.vel = [msg.pointdata[0].x,
                        msg.pointdata[0].y,
                        msg.pointdata[0].z,
                        msg.pointdata[1].x,
                        msg.pointdata[1].y,
                        msg.pointdata[1].z]
        elif msg.name == "jp":
            self.teleop_msg.joint_pos = [msg.pointdata[0].x,
                                         msg.pointdata[0].y,
                                         msg.pointdata[0].z,
                                         msg.pointdata[1].x,
                                         msg.pointdata[1].y,
                                         msg.pointdata[1].z]
            
            self.teleop_msg.header.stamp = \
                self.get_clock().now().to_msg()
            self.pub.publish(self.teleop_msg)

    # Compute current motion based on pressed keys and publish
    def string_callback(self, msg):
        if msg.name == "m": # mode selection
            if msg.data == "jv":
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_JOINT_VEL
            elif msg.data == "jp":
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_JOINT_POS
            elif msg.data == "cv":
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_CARTESIAN_VEL
            elif msg.data == "cp":
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_CARTESIAN_POS
            elif msg.data == "a":
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_AUTO
            else:
                self.teleop_msg.control_mode = self.teleop_msg.CONTROL_MODE_NONE
        elif msg.name == "k": # key command
            self.teleop_msg.joint_vel = [0.0] * 6
            for key in msg.data:
                if key in self.key_bindings:
                    for i in range(6):
                        self.teleop_msg.joint_vel[i] += \
                            self.key_bindings[key][i]*self.vel[key][i]

            self.teleop_msg.header.stamp = \
                self.get_clock().now().to_msg()
            self.pub.publish(self.teleop_msg)


def main():
    rclpy.init()
    node = TeleopSlicer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
