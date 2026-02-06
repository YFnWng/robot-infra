
from control_interface.msg import ControlStream, ManagerStream, ManagerEvent
# from control_interface.srv import DeviceCmd, Debug
import ros2_igtl_bridge.msg
import rclpy
from rclpy.node import Node


msg = """
This node takes commands from slicer through OpenIGTLink and publishes them
as ControlStream messages. 

CTRL-C to quit
"""

DOMAIN = "robot"

class SlicerHandler(Node):

    def __init__(self):
        super().__init__('slicer')

        # --- parameters ---
        # read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.declare_parameter('joints', [''])
        self.declare_parameter('keys', [''])
        self.declare_parameter('key_joint_idx', [0])
        self.declare_parameter('directions', [0])
        self.declare_parameter('joint_vels', [0.0])
        # must declare default value with the correct type, otherwise loading from yaml will fail silently.

        self.joints = self.get_parameter('joints').value
        self.keys = self.get_parameter('keys').value
        key_joint_idx = self.get_parameter('key_joint_idx').value
        directions = self.get_parameter('directions').value
        self.vel = self.get_parameter('joint_vels').value

        self.key_bindings = {
            k: (j, d)
            for k, j, d in zip(self.keys, key_joint_idx, directions)
        }
        # print(self.keys)
        # print(self.key_bindings)
        
        # for param_name in ['joints', 'keys', 'key_joint_idx', 'directions', 'joint_vels']:
        #     param = self.get_parameter(param_name)
        #     self.get_logger().info(f"{param_name} = {param.value}")
        # self.get_logger().info(f"{self.key_bindings}")

        # self.add_on_set_parameters_callback(
        #     self._on_parameter_update
        # )

        # --- message types ---
        self.teleop_stream = ControlStream()
        self.teleop_stream.header.frame_id = "slicer"
        self.teleop_event = ManagerEvent()
        self.teleop_event.header.frame_id = "slicer"
        self.joint_pos_stream = ros2_igtl_bridge.msg.PointArray()
        self.joint_pos_stream.name = DOMAIN + "/joint_pos_stream"
        self.manager_event = ros2_igtl_bridge.msg.String()
        self.manager_event.name = DOMAIN + "/event"

        # --- subscriptions ---
        self.string_sub = self.create_subscription(
            ros2_igtl_bridge.msg.String,
            '/IGTL_STRING_IN',
            self.string_callback,
            10
        )
        self.string_sub

        self.point_sub = self.create_subscription(
            ros2_igtl_bridge.msg.PointArray,
            '/IGTL_POINT_IN',
            self.point_callback,
            10
        )
        self.point_sub

        self.manager_event_sub = self.create_subscription(
            ManagerEvent,
            '/manager/event',
            self.manager_event_callback,
            10
        )
        self.manager_event_sub

        self.manager_stream_sub = self.create_subscription(
            ManagerStream,
            '/manager/stream',
            self.manager_stream_callback,
            10
        )
        self.manager_stream_sub

        # --- publisher ---
        self.intent_pub = self.create_publisher(ControlStream, '/teleop/control', 10)
        self.teleop_event_pub = self.create_publisher(ManagerEvent, '/teleop/event', 10)
        self.manager_event_pub = self.create_publisher(ros2_igtl_bridge.msg.String,
                                         '/IGTL_STRING_OUT', 10)
        self.joint_pos_pub = self.create_publisher(ros2_igtl_bridge.msg.PointArray,
                                         '/IGTL_POINT_OUT', 10)


        # Spin publishing thread
        # self.timer = self.create_timer(0.05, self.publish_teleop)  # 20 Hz

    # Set targets
    def point_callback(self, msg):
        domain, role = msg.name.split("/", 1)
        if domain != DOMAIN:
            return

        if role == "joint_vel_target":
            self.vel = [msg.pointdata[0].x,
                        msg.pointdata[0].y,
                        msg.pointdata[0].z,
                        msg.pointdata[1].x,
                        msg.pointdata[1].y,
                        msg.pointdata[1].z]
            self.get_logger().info(f"Set velocity: {self.vel}")

        elif msg.name == "joint_pos_target":
            self.teleop_stream.joint_pos = [msg.pointdata[0].x,
                                         msg.pointdata[0].y,
                                         msg.pointdata[0].z,
                                         msg.pointdata[1].x,
                                         msg.pointdata[1].y,
                                         msg.pointdata[1].z]
            
            self.teleop_stream.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_stream)
            self.get_logger().info(f"Control position: {self.teleop_stream.joint_pos}")

    # Compute current motion based on pressed keys and publish
    def string_callback(self, msg):
        domain, role = msg.name.split("/", 1)
        if domain != DOMAIN:
            return
        
        if role == "cmd": 

            n = len(msg.data)
            if n <= 0:
                return
            
            self.teleop_event.predicate = ord(msg.data[0])
            self.teleop_event.state = []
            self.teleop_event.data = []
            
            if n > 1:
                self.teleop_event.text = msg.data[1:]
            else:
                self.teleop_event.text = ""

            self.teleop_event.header.stamp = \
                self.get_clock().now().to_msg()
            self.teleop_event_pub.publish(self.teleop_event)
            self.get_logger().info('Command: "%s"' % msg.data)

        elif role == "key": # key command
            self.teleop_stream.joint_vel = [0.0] * 6
            for key in msg.data:
                # self.get_logger().info(f"key pressed: {key}")
                if key in self.keys:
                    # later key overrides
                    joint, dir = self.key_bindings[key]
                    self.teleop_stream.joint_vel[joint] = self.vel[joint]*dir

            self.teleop_stream.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_stream)
            self.get_logger().info(f"Control joint vel: {self.teleop_stream.joint_vel}")

    def manager_event_callback(self, msg: ManagerEvent):
        if msg.text != "":
            subject = msg.text
        elif msg.state != []:
            subject = "".join(map(chr, msg.state))
        else:
            subject = ""

        self.manager_event.data = chr(msg.predicate) + subject
        self.manager_event_pub.publish(self.manager_event)

    def manager_stream_callback(self, msg: ManagerStream):
        self.joint_pos_stream.pointdata[0].x = msg.joint_pos[0]
        self.joint_pos_stream.pointdata[0].y = msg.joint_pos[1]
        self.joint_pos_stream.pointdata[0].z = msg.joint_pos[2]
        self.joint_pos_stream.pointdata[1].x = msg.joint_pos[3]
        self.joint_pos_stream.pointdata[1].y = msg.joint_pos[4]
        self.joint_pos_stream.pointdata[1].z = msg.joint_pos[5]

        self.joint_pos_pub.publish(self.joint_pos_stream)


def main():
    rclpy.init()
    node = SlicerHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
