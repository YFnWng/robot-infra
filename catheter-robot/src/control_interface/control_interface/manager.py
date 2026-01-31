import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from control_interface.msg import TeleopIntents, DeviceMsg, ControllerFeedback
from control_interface.srv import ManagerCmd, DeviceCmd, Debug

from collections import defaultdict
import time

class ControlManager(Node):

    def __init__(self):
        super().__init__('actuation_manager')

        # self.manager_req = ManagerCmd.Request()
        self.control_mode = ManagerCmd.Request.NONE

        self.deadman = False
        self.scale = 0.3
        self.estop = False

        self.active_source = None
        self.last_input_time = defaultdict(lambda: 0.0)

        self.DEADMAN_TIMEOUT = 0.2
        self.SOURCE_TIMEOUT = 0.2

        # subscriptions
        self.teleop_sub = self.create_subscription(
            TeleopIntents, '/teleop', self.teleop_callback, 10)
        self.teleop_sub

        # self.auto_sub = self.create_subscription(
        #     AutoIntents, 'auto', self.auto_callback, 10)
        # self.auto_sub

        self.feedback_sub = self.create_subscription(
            DeviceMsg, '/device/feedback', self.feedback_callback, 10)
        self.feedback_sub

        # self.sensor_sub = self.create_subscription(
        #     AutoIntents, 'sensor', self.sensor_callback, 10)
        # self.sensor_sub

        # publishers
        self.cmd_pub = self.create_publisher(
            DeviceMsg, '/manager/cmd', 10)
        
        self.feedback_pub = self.create_publisher(
            ControllerFeedback, '/manager/feedback', 10)

        # services
        self.create_service(ManagerCmd, '/manager/cmd', self.cmd_callback)
        self.create_service(Debug, '/manager/debug', self.debug_callback)
        # self.create_service(Trigger, 'e_stop', self.e_stop_callback)
        # self.create_service(Trigger, 'set_zero', self.set_zero_callback)
        self.device_client = self.create_client(DeviceCmd, '/device/cmd')
        while not self.device_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.device_req = DeviceCmd.Request()

        # self.timer = self.create_timer(0.01, self.update)

    def deadman_cb(self, msg):
        self.deadman = msg.data

    def cmd_callback(self, req, res):
        self.active_source = req.source

        if req.cmd <= req.DEBUG:
            self.control_mode = req.cmd
            res.success = True
            res.message = "Control mode switched"

        else:
            if req.cmd == req.START_MOTOR:
                self.device_req.cmd = self.device_client.START
                self.device_req.data = []
            elif req.cmd == req.STOP_MOTOR:
                self.device_req.cmd = self.device_client.STOP
                self.device_req.data = []
            elif req.cmd == req.SET_ZERO:
                self.device_req.cmd = self.device_client.ZERO
                self.device_req.data = []

            future = self.device_client.call_async(self.device_req)
            rclpy.spin_until_future_complete(self, future)
            device_res = future.result()
            res.success = device_res.success
            res.message = device_res.response

    def debug_callback(self, req, res):
        if self.control_mode == ManagerCmd.Request.DEBUG:
            self.device_req.cmd = req.cmd
            future = self.device_client.call_async(self.device_req)
            self.get_logger().info(f'Send raw cmd: {req.cmd}')
            rclpy.spin_until_future_complete(self, future)
            device_res = future.result()
            res.success = device_res.success
            res.message = device_res.response
            

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

        if self.deadman or \
            self.estop or \
            self.control_mode == ManagerCmd.Request.NONE:
            return

        if self.active_source is None:
            self.active_source = msg.source

        if msg.source != self.active_source:
            return

        out = DeviceMsg()
        out.header.stamp = self.get_clock().now().to_msg()

        if self.control_mode == ManagerCmd.Request.JOINT_VEL:
            # prefix = DeviceMsg.VEL.encode("ascii")
            # data = struct.pack("<6f", *msg.joint_vel)
            # out.byte_msg = prefix + data
            out.predicate = DeviceMsg.VEL
            out.data = msg.joint_vel
        elif self.control_mode == ManagerCmd.Request.JOINT_POS:
            # prefix = DeviceMsg.POS.encode("ascii")
            # data = struct.pack("<6f", *msg.joint_pos)
            # out.byte_msg = prefix + data
            out.predicate = DeviceMsg.POS
            out.data = msg.joint_pos
        
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
        out = DeviceMsg()
        out.header.stamp = self.get_clock().now().to_msg()
        prefix = DeviceMsg.POS.encode("ascii")
        data = struct.pack("<6f", *([0.0]*6))
        out.byte_msg = prefix + data
        self.cmd_pub.publish(out)

def main():
    rclpy.init()
    node = ControlManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
