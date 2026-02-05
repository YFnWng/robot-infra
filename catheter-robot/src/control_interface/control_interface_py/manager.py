#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# from std_msgs.msg import Bool, Float64
# from std_srvs.srv import Trigger
from control_interface.msg import ControlStream, \
    DeviceStream, ManagerStream, DeviceEvent, ManagerEvent
from control_interface.srv import DeviceCmd

from collections import defaultdict
import time

class ControlManager(Node):

    def __init__(self):
        super().__init__('control_manager')

        self.control_mode = ManagerEvent.NONE

        self.deadman = False
        self.scale = 0.3
        self.estop = False

        self.active_source = None
        self.last_input_time = defaultdict(lambda: 0.0)

        self.DEADMAN_TIMEOUT = 0.2
        self.SOURCE_TIMEOUT = 0.2

        # subscriptions
        self.teleop_sub = self.create_subscription(
            ControlStream, '/teleop/control', self.teleop_callback, 10)
        self.teleop_sub

        self.teleop_event_sub = self.create_subscription(
            ManagerEvent, '/teleop/event', self.teleop_event_callback, 10)
        self.teleop_event_sub

        self.device_event_sub = self.create_subscription(
            DeviceEvent, '/device/event', self.device_event_callback, 10)
        self.device_event_sub

        self.state_sub = self.create_subscription(
            DeviceStream, '/device/state', self.device_state_callback, 10)
        self.state_sub

        # publishers
        self.control_pub = self.create_publisher(
            DeviceStream, '/manager/control', 10)
        
        self.state_pub = self.create_publisher(
            ManagerStream, '/manager/state', 10)
        
        self.event_pub = self.create_publisher(
            ManagerEvent, '/manager/event', 10)

        # services
        # self.create_service(DeviceCmd, '/device/command', self.cmd_callback)
        # self.create_service(Debug, '/device/debug', self.debug_callback)
        self.device_client = self.create_client(DeviceCmd, '/device/command')
        while not self.device_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.device_req = DeviceCmd.Request()

        # self.timer = self.create_timer(0.01, self.update)

    # def deadman_cb(self, msg):
    #     self.deadman = msg.data

    def teleop_callback(self, msg: ControlStream):
        self.last_input_time[msg.header.frame_id] = time.time()

        if self.deadman or \
            self.estop or \
            self.control_mode == ManagerEvent.NONE:
            return

        if self.active_source is None:
            self.active_source = msg.header.frame_id

        if msg.header.frame_id != self.active_source:
            return

        out = DeviceStream()
        out.header.stamp = self.get_clock().now().to_msg()
        # out.header.frame_id = msg.header.frame_id

        if self.control_mode == ManagerEvent.JOINT_VEL and msg.joint_vel:
            out.predicate = DeviceStream.VEL
            out.data = msg.joint_vel
        elif self.control_mode == ManagerEvent.JOINT_POS and msg.joint_pos:
            out.predicate = DeviceStream.POS
            out.data = msg.joint_pos
        
        self.control_pub.publish(out)

    def teleop_event_callback(self, msg: ManagerEvent):
        self.active_source = msg.header.frame_id

        if msg.predicate == ManagerEvent.MODE:
            if len(msg.text) != 1:
                self.get_logger().info(f'Invalid control mode request')
            self.control_mode = ord(msg.text)
            self.get_logger().info(f'Control mode switched to {msg.text}')

        else:
            self.device_req.predicate = msg.predicate
            self.device_req.cmd = msg.text
            self.device_req.data = msg.data

            
            future = self.device_client.call_async(self.device_req)
            
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Command {msg.predicate}")
            device_res = future.result()
            if device_res.success:
                out = ManagerEvent()
                out.header.stamp = self.get_clock().now().to_msg()
                out.predicate = self.device_req.predicate
                # out.state = msg.state
                self.event_pub.publish(out)
                # res.message = device_res.response

    def device_state_callback(self, msg: DeviceStream):
        out = ManagerStream()
        out.header.stamp = self.get_clock().now().to_msg()
        if msg.predicate == DeviceStream.VEL:
            out.joint_vel = msg.data
        elif msg.predicate == DeviceStream.POS:
            out.joint_pos = msg.data
        self.state_pub.publish(out)

    def device_event_callback(self, msg: DeviceEvent):
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = msg.predicate
        out.state = msg.state
        self.event_pub.publish(out)

    # def update(self):
    #     now = time.time()

    #     if not self.deadman or self.estop:
    #         self.publish_zero()
    #         self.active_source = None
    #         return

    #     if self.active_source:
    #         if now - self.last_input_time[self.active_source] > self.SOURCE_TIMEOUT:
    #             self.publish_zero()
    #             self.active_source = None

    # def publish_zero(self):
    #     out = DeviceStream()
    #     out.header.stamp = self.get_clock().now().to_msg()
    #     prefix = DeviceStream.POS.encode("ascii")
    #     data = struct.pack("<6f", *([0.0]*6))
    #     out.byte_msg = prefix + data
    #     self.cmd_pub.publish(out)

def main():
    rclpy.init()
    node = ControlManager()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
