
from control_interface.msg import TeleopIntents
from control_interface.srv import ManagerCmd, Debug
import ros2_igtl_bridge.msg
import rclpy
from rclpy.node import Node


msg = """
This node takes commands from slicer through OpenIGTLink and publishes them
as TeleopIntents messages. 

CTRL-C to quit
"""

DOMAIN = "robot/"

MODE_PREFIX             = "M"
MODE_NONE               = "non"
MODE_CARTESIAN_TWIST    = "cv"
MODE_CARTESIAN_POS      = "cp"
MODE_JOINT_VEL          = "jv"
MODE_JOINT_POS          = "jp"
MODE_DEBUG              = "d"
START_MOTOR             = "I"
STOP_MOTOR              = "S"
SET_ZERO                = "Z"
LIMIT                   = "L"
LIMIT_REACHED           = "t"
DEBUG                   = "D"

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
        self.feedback_string_msg = ros2_igtl_bridge.msg.String()
        self.feedback_string_msg.name = DOMAIN + "response"

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

        # --- publisher ---
        self.intent_pub = self.create_publisher(TeleopIntents, 'teleop', 10)
        self.feedback_string_pub = self.create_publisher(ros2_igtl_bridge.msg.String,
                                         '/IGTL_STRING_OUT', 10)

        # --- services ---
        self.cmd_client = self.create_client(ManagerCmd, 'cmd_to_manager')
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cmd_req = ManagerCmd.Request()
        self.cmd_req.source = "slicer"
        self.pending_cmd_call = False

        self.debug_client = self.create_client(Debug, 'debug')
        while not self.debug_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.debug_req = Debug.Request()
        self.debug_req.source = "slicer"


        # Spin publishing thread
        # self.timer = self.create_timer(0.05, self.publish_teleop)  # 20 Hz

    # Set targets
    def point_callback(self, msg):
        domain, role = msg.name.split("/", 1)
        if domain != DOMAIN:
            return
        
        self.get_logger().info('Publishing: "%s"' % msg.data)

        if role == "joint_vel":
            self.vel = [msg.pointdata[0].x,
                        msg.pointdata[0].y,
                        msg.pointdata[0].z,
                        msg.pointdata[1].x,
                        msg.pointdata[1].y,
                        msg.pointdata[1].z]
        elif msg.name == "joint_pos":
            self.teleop_msg.joint_pos = [msg.pointdata[0].x,
                                         msg.pointdata[0].y,
                                         msg.pointdata[0].z,
                                         msg.pointdata[1].x,
                                         msg.pointdata[1].y,
                                         msg.pointdata[1].z]
            
            self.teleop_msg.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_msg)

    # Compute current motion based on pressed keys and publish
    def string_callback(self, msg):
        domain, role = msg.name.split("/", 1)
        if domain != DOMAIN:
            return
        
        if role == "cmd": 
            
            if msg.data[0] == MODE_PREFIX: # mode selection
                if self.pending_cmd_call:
                    return

                if msg.data[1:] == MODE_JOINT_VEL:
                    self.cmd_req.mode = self.cmd_req.JOINT_VEL
                elif msg.data[1:] == MODE_JOINT_POS:
                    self.cmd_req.mode = self.cmd_req.JOINT_POS
                elif msg.data[1:] == MODE_CARTESIAN_TWIST:
                    self.cmd_req.mode = self.cmd_req.CARTESIAN_VEL
                elif msg.data[1:] == MODE_CARTESIAN_POS:
                    self.cmd_req.mode = self.cmd_req.CARTESIAN_POS
                elif msg.data[1:] == MODE_DEBUG:
                    self.cmd_req.mode = self.cmd_req.DEBUG
                else:
                    self.cmd_req.mode = self.cmd_req.NONE

                future = self.cmd_client.call_async(self.cmd_req)
                future.add_done_callback(self.cmd_callback)
                self.get_logger().info(f'Request changing mode to: {msg.data[1:]}')

            elif msg.data[0] == START_MOTOR:
                self.cmd_req.mode = self.cmd_req.START_MOTOR
                future = self.cmd_client.call_async(self.cmd_req)
                self.get_logger().info(f'Request to start motor')
                future.add_done_callback(self.cmd_callback)
                

            elif msg.data[0] == STOP_MOTOR:
                self.cmd_req.mode = self.cmd_req.STOP_MOTOR
                future = self.cmd_client.call_async(self.cmd_req)
                self.get_logger().info(f'Request to stop motor')
                future.add_done_callback(self.cmd_callback)

            elif msg.data[0] == SET_ZERO:
                self.cmd_req.mode = self.cmd_req.SET_ZERO
                future = self.cmd_client.call_async(self.cmd_req)
                self.get_logger().info(f'Request to set zero')
                future.add_done_callback(self.cmd_callback)

            elif msg.data[0] == DEBUG:
                self.debug_req.cmd = msg.data
                future = self.cmd_client.call_async(self.cmd_req)
                self.get_logger().info(f'Sending driver cmd: {msg.data}')
                future.add_done_callback(self.debug_callback)


        elif role == "key": # key command
            self.teleop_msg.joint_vel = [0.0] * 6
            for key in msg.data:
                if key in self.key_bindings:
                    for i in range(6):
                        self.teleop_msg.joint_vel[i] += \
                            self.key_bindings[key][i]*self.vel[key][i]

            self.teleop_msg.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_msg)

    def cmd_callback(self, future):
        self.pending_mode_call = False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if not res.success:
            self.get_logger().warn(f'Request failed: {res.message}')
        else:
            self.get_logger().info(f'Request success: {res.message}')

    def debug_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if not res.success:
            self.get_logger().warn(f'Motor driver timed out')
        else:
            self.get_logger().info(f'Motor driver response: {res.message}')
            self.feedback_string_msg.data = res.message
            self.feedback_string_pub.publish(self.feedback_string_msg)


def main():
    rclpy.init()
    node = TeleopSlicer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
