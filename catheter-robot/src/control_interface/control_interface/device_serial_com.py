import struct
import rclpy
from rclpy.node import Node
import serial
import time
from typing import Optional
from control_interface.msg import DeviceMsg
from control_interface.srv import DeviceCmd

class SerialCommunication(Node):
    def __init__(self):
        super().__init__(
            'device_serial_com',
            automatically_declare_parameters_from_overrides=True
            )
 
        self.serial_port = None
        self.is_connected = False
        self.check_buffer = self.declare_parameter('check_buffer', True).value  # Flag to check if the buffer is empty before reading
        self.command_buffer = None
        self.response_buffer = None
        self.startMarker = b'<' # '<'
        self.endMarker = b'>' # '>'
        self.timeout_ms = self.declare_parameter('timeout_ms', 1000).value

        self.sub = self.create_subscription(
            DeviceMsg,
            '/manager/cmd',
            self.send_command,
            10
        )
        self.sub

        self.pub = self.create_publisher(
            DeviceMsg, '/device/feedback', 10)

        timer_period = self.declare_parameter('timer_period', 0.01).value # 10 ms
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.srv = self.create_service(
            DeviceCmd,
            '/device/cmd',
            self.handle_device_command
        )

    def connect(self, port: str, baudrate: int = 115200, timeout: int = 1000):
        """Connect to the specified serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        # the frequency of data receiving should be higher than the device sending
        self.serial_port = serial.Serial(port, baudrate, timeout=timeout) 
        if self.serial_port.is_open:
            self.is_connected = True
            self.serial_port.set_buffer_size(rx_size=100, tx_size=100)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud.")
        else:
            self.is_connected = False
            self.get_logger().warn(f"Failed to connect to {port}.")

    def close(self):
        """Close the serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            self.get_logger().info(f"Serial port {self.serial_port.name} closed.")

    # def send_str(self, command: str, expect_response: bool = False) -> Optional[str]:
    #     """Send a command to the motor controller"""
    #     try:
    #         if self.serial_port and self.serial_port.is_open:
    #             # print(f"Sending command: {command}")
    #             self.serial_port.reset_input_buffer()
    #             self.serial_port.write((command).encode())
    #             if expect_response:
    #                 return self.read_response()
    #         else:
    #             raise Exception("Serial port is not open.")
    #     except serial.SerialException as e:
    #         print(f"Serial communication error: {e}")
    #         return None

    # def read_str(self) -> Optional[str]:
    #     if self.check_buffer:
    #         condition = self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0
    #     else:
    #         condition = self.serial_port and self.serial_port.is_open
    #     condition = self.serial_port and self.serial_port.is_open
    #     if condition:
    #         # t1 = time.time()
    #         # response = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
    #         response = self.serial_port.read_until(b'\r').decode("utf-8", errors="ignore").strip()
    #         while self.serial_port.in_waiting > 0:
    #             # response = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
    #             response = self.serial_port.read_until(b'\r').decode("utf-8", errors="ignore").strip()
    #         self.response_buffer = response
    #         # current_time = time.time()
    #         # elapsed_time = (current_time - t1) * 1000  # Convert to milliseconds
    #         # print(f"read_response elapsed time: {elapsed_time:.2f} ms")
    #         # print(f"Received response: {response}","byte waiting:",self.serial_port.in_waiting)
    #         if response  == self.serial_port_node.GetParameter("Data"):
    #             pass
    #         else:
    #             self.serial_port_node.SetParameter("Data", response )
    #         return self.response_buffer
    #     return None
    
    def send_bytes(self, binaries: bytes) -> Optional[bytes]:
        """Send a command to the motor controller"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")

        try:
            self.serial_port.write(
                self.startMarker + binaries + self.endMarker
                )
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return None


    def read_bytes(self, timeout: float = 1.0) -> Optional[bytes]:
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return None

        start_time = time.time()
        response = None

        try:
            while True:
                # check overall timeout
                if time.time() - start_time > timeout:
                    self.get_logger().warn("Serial read timed out.")
                    return None

                if self.serial_port.in_waiting > 0:
                    # read up to start marker
                    self.serial_port.read_until(self.startMarker)

                    # read until end marker
                    response = self.serial_port.read_until(self.endMarker)
                    
                    # got a response, return immediately
                    return response[:-1]

                # small sleep to avoid busy-waiting
                time.sleep(0.001)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            return None
        # try:
        #     if self.serial_port and self.serial_port.is_open:
        #         response = None
        #         while self.serial_port.in_waiting > 0:
        #             self.serial_port.read_until(self.startMarker)
        #             response = self.serial_port.read_until(self.endMarker)
        #         return response
        #     else:
        #         self.get_logger().error("Serial port is not open.")
        #         return None
        # except serial.SerialException as e:
        #     self.get_logger().error(f"Serial communication error: {e}")
        #     return None
        
    
    def send_command(self, msg):
        prefix = msg.predicate.encode("ascii")
        data = struct.pack("<6d", *msg.data)
        self.send_bytes(prefix + data)

    def timer_callback(self):
        response = self.read_bytes()
        if response is not None:
            out = DeviceMsg()
            out.header.stamp = self.get_clock().now().to_msg()
            out.predicate = response[:1].decode('utf-8')
            n_floats = len(response[1:]) // 8
            out.data = list(struct.unpack('<' + 'd'*n_floats, response[1:]))
            self.pub.publish(out)
    
    # def set_timer_interval(self, interval: int):
    #     """Set the timer interval for reading responses"""
    #     self.timer.setInterval(interval)

    def handle_device_command(self, request, response):
        self.get_logger().info(f"Received serial command: {request.command}")

        try:
            # ---- SEND ----
            self.send_bytes(request.cmd.encode("ascii"))
            # time.sleep(0.01)  # simulate transmission latency

            # ---- WAIT ----
            self.read_bytes()
            start = time.time()
            while time.time() - start < self.timeout_ms:
                if self.serial.in_waiting:
                    reply = self.serial.readline().decode()
                    break
                # time.sleep(0.005)
                # reply = "OK"  # simulate reply
                break
            else:
                response.success = False
                response.response = "Timeout waiting for device"
                return response

            response.success = True
            response.response = reply

        except Exception as e:
            response.success = False
            response.response = str(e)

        return response

def main():
    rclpy.init()
    node = SerialCommunication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
