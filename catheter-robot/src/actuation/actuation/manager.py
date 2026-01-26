import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from control_interface.msg import TeleopIntents
from control_interface.msg import Bytes
from control_interface.msg import ControllerFeedback
from control_interface.srv import SetControlMode
from collections import defaultdict
import time

class ActuationManager(Node):

    def __init__(self):
        super().__init__('actuation_manager')

        self.control_mode = None

        self.deadman = False
        self.scale = 0.3
        self.estop = False

        self.active_source = None
        self.last_input_time = defaultdict(lambda: 0.0)

        self.DEADMAN_TIMEOUT = 0.2
        self.SOURCE_TIMEOUT = 0.2

        # subscriptions
        self.teleop_sub = self.create_subscription(
            TeleopIntents, 'teleop', self.teleop_callback, 10)
        self.teleop_sub

        # self.auto_sub = self.create_subscription(
        #     AutoIntents, 'auto', self.auto_callback, 10)
        # self.auto_sub

        self.feedback_sub = self.create_subscription(
            Bytes, 'hardware_feedback', self.feedback_callback, 10)
        self.feedback_sub

        # self.sensor_sub = self.create_subscription(
        #     AutoIntents, 'sensor', self.sensor_callback, 10)
        # self.sensor_sub

        # publishers
        self.cmd_pub = self.create_publisher(
            Bytes, 'hardware_cmd', 10)
        
        self.feedback_pub = self.create_publisher(
            ControllerFeedback, 'controller_feedback', 10)

        # services
        self.create_service(SetControlMode, 'set_control_mode', self.control_mode_callback)
        # self.create_service(Trigger, 'e_stop', self.e_stop_callback)
        # self.create_service(Trigger, 'set_zero', self.set_zero_callback)

        # self.timer = self.create_timer(0.01, self.update)

    def deadman_cb(self, msg):
        self.deadman = msg.data

    def control_mode_callback(self, req, res):
        self.active_source = req.source
        self.control_mode = req.mode
        res.success = True
        res.message = "Control mode switched"

    # def e_stop_callback(self, req, res):
    #     self.estop = True
    #     out = StringStamped()
    #     out.header.stamp = self.get_clock().now().to_msg()

    #     out.cmd = 
    #     self.pub.publish(out)
    #     res.success = True
    #     res.message = "E-stop engaged"
    #     return res

    # def set_zero_callback(self, req, res):
    #     out = StringStamped()
    #     out.header.stamp = self.get_clock().now().to_msg()

    #     out.cmd = 
    #     self.pub.publish(out)
    #     res.success = True
    #     res.message = "Teleop reset"
    #     return res

    def teleop_callback(self, msg):
        self.last_input_time[msg.source] = time.time()

        if not self.deadman or \
            self.estop or \
            self.control_mode == SetControlMode.Request.NONE:
            return

        if self.active_source is None:
            self.active_source = msg.source

        if msg.source != self.active_source:
            return

        out = Bytes()
        out.header.stamp = self.get_clock().now().to_msg()

        if self.control_mode == SetControlMode.Request.JOINT_VEL:
            prefix = Bytes.VEL.encode("ascii")
            data = struct.pack("<6f", *msg.joint_vel)
            out.byte_msg = prefix + data
        elif self.control_mode == SetControlMode.Request.JOINT_POS:
            prefix = Bytes.POS.encode("ascii")
            data = struct.pack("<6f", *msg.joint_pos)
            out.byte_msg = prefix + data
        
        self.cmd_pub.publish(out)

    def feedback_callback(self, msg):
        # TODO
        out = ControllerFeedback()
        self.feedback_pub.publish(out)

    def update(self):
        now = time.time()

        if not self.deadman or self.estop:
            self.publish_zero()
            self.active_source = None
            return

        if self.active_source:
            if now - self.last_input_time[self.active_source] > self.SOURCE_TIMEOUT:
                self.publish_zero()
                self.active_source = None

    def publish_zero(self):
        out = Bytes()
        out.header.stamp = self.get_clock().now().to_msg()
        prefix = Bytes.POS.encode("ascii")
        data = struct.pack("<6f", *([0.0]*6))
        out.byte_msg = prefix + data
        self.cmd_pub.publish(out)

def main():
    rclpy.init()
    node = ActuationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
