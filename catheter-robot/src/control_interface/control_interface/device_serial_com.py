#!/usr/bin/env python3
import struct
import time
import threading
import uuid
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

import serial

from control_interface.msg import DeviceStream, DeviceEvent, ManagerEvent
from control_interface.srv import DeviceCmd

stream_prefix = [DeviceStream.POS, DeviceStream.VEL, DeviceStream.ENC]
event_prefix = [ManagerEvent.LIMIT]
response_prefix = [ManagerEvent.CONNECT,
                    ManagerEvent.MODE, 
                    ManagerEvent.START_MOTOR,
                    ManagerEvent.STOP_MOTOR,
                    ManagerEvent.SET_ZERO,
                    ManagerEvent.SET_TARGET_VEL]

class SerialCommunication(Node):
    def __init__(self):
        super().__init__(
            'device_serial_com',
            automatically_declare_parameters_from_overrides=True
            )
 
        # ---- Parameters ----
        self.startMarker = b'<'
        self.endMarker = b'>'
        self.timeout_sec = self.declare_parameter('timeout_sec', 1.0).value

        # ---- Serial ----
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False

        # ---- ROS interfaces ----
        self.control_sub = self.create_subscription(
            DeviceStream, '/manager/control', self.on_manager_control, 10
        )

        self.state_pub = self.create_publisher(
            DeviceStream, '/device/state', 10
        )

        self.event_pub = self.create_publisher(
            DeviceEvent, '/device/event', 10
        )

        self.cmd_srv = self.create_service(
            DeviceCmd, '/device/command', self.handle_device_command
        )

        # ---- Pending service requests ----
        self.pending = {}  # req_id -> dict(future, deadline)
        self.pending_lock = threading.Lock()

        # ---- RX thread ----
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # ---- Timeout watchdog ----
        self.create_timer(0.01, self.check_timeouts)


    # ============================================================
    # Serial connection
    # ============================================================

    def connect(self, port: str, baudrate: int = 115200, timeout: int = 1000):
        """Connect to the specified serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        # the frequency of data receiving should be higher than the device sending
        self.serial_port = serial.Serial(port, baudrate, timeout=timeout) 
        self.is_connected = self.serial_port.is_open

        if self.is_connected:
            self.get_logger().info(f"Connected to {port} @ {baudrate}")
        else:
            self.get_logger().error("Failed to open serial port")

    def close(self):
        """Close the serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            self.get_logger().info(f"Serial port {self.serial_port.name} closed.")
    
    # ============================================================
    # TX
    # ============================================================

    def send_bytes(self, payload: bytes) -> Optional[bytes]:
        """Send a command to the motor controller"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        try:
            self.serial_port.write(
                self.startMarker + payload + self.endMarker
                )
        except serial.SerialException as e:
            self.get_logger().error(f"Serial TX error: {e}")
        
    def on_manager_control(self, msg: DeviceStream):
        prefix = bytes([msg.predicate])
        data = struct.pack("<" + "d" * len(msg.data), *msg.data)
        self.send_bytes(prefix + data)

    # ============================================================
    # RX LOOP
    # ============================================================

    def rx_loop(self):
        while rclpy.ok():
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.01)
                continue

            try:
                raw = self.serial_port.read_until(self.endMarker)
                if not raw.startswith(self.startMarker):
                    continue

                payload = raw[1:-1]
                if not payload:
                    continue

                prefix = payload[0] # int 0–255

                if prefix in stream_prefix:
                    self.handle_device_stream(prefix, payload[1:])
                elif prefix in event_prefix:
                    self.handle_device_event(prefix, payload[1:])
                elif prefix in response_prefix:
                    self.handle_device_response(prefix, payload[1:])
                else:
                    self.get_logger().warn(f"Unknown prefix: {prefix}")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial RX error: {e}")
    
    # ============================================================
    # Handlers
    # ============================================================

    def handle_device_stream(self, prefix: int, body: bytes):
        n = len(body) // 8
        values = struct.unpack("<" + "d" * n, body) # tuple of doubles

        msg = DeviceStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.predicate = prefix
        msg.data = list(values)
        self.state_pub.publish(msg)

    def handle_device_event(self, prefix: int, body: bytes):
        msg = DeviceEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.predicate = prefix
        msg.state = list(body)
        self.event_pub.publish(msg)

    def handle_device_response(self, prefix: int, body: bytes):
        """
        Expected: body = req_id(16 bytes) + response_payload
        """
        if len(body) < 16:
            self.get_logger().warn("Malformed response")
            return

        req_id = body[:16]
        payload = body[16:]

        with self.pending_lock:
            entry = self.pending.pop(req_id, None)

        if not entry:
            self.get_logger().warn("Unmatched response id={req_id}")
            return

        entry["future"].set_result({
            "success": True,
            "response": payload.decode("utf-8", errors="ignore")
        })

    # ============================================================
    # Service
    # ============================================================

    def handle_device_command(self, request, response):
        req_id = uuid.uuid4().bytes  # 16 bytes

        payload = bytes([request.predicate]) + \
              req_id + request.cmd.encode('utf-8')
        self.send_bytes(payload)

        future = Future()
        with self.pending_lock:
            self.pending[req_id] = {
                "future": future,
                "deadline": time.time() + self.timeout_sec
            }

        # BLOCK here safely if using MultiThreadedExecutor
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        response.success = result["success"]
        response.response = result["response"]

        return response

    # ============================================================
    # Timeout watchdog
    # ============================================================

    def check_timeouts(self):
        now = time.time()
        expired = []

        with self.pending_lock:
            for req_id, entry in self.pending.items():
                if now > entry["deadline"]:
                    entry["future"].set_result({
                        "success": False,
                        "response": "Timeout"
                    })
                    expired.append(req_id)

            for req_id in expired:
                del self.pending[req_id]

def main():
    rclpy.init()

    node = SerialCommunication()

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
