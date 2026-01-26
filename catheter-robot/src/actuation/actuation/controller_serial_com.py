import rclpy
from rclpy.node import Node
import serial
import time
from typing import Optional
from control_interface.msg import Bytes

class SerialCommunication(Node):
    def __init__(self):
        super().__init__(
            'serial_com',
            automatically_declare_parameters_from_overrides=True
            )
 
        self.serial_port = None
        self.is_connected = False
        self.check_buffer = self.declare_parameter('check_buffer', True).value  # Flag to check if the buffer is empty before reading
        self.command_buffer = None
        self.response_buffer = None
        self.startMarker = b'<' # '<'
        self.endMarker = b'>' # '>'

        self.sub = self.create_subscription(
            Bytes,
            'hardware_cmd',
            self.input_callback,
            10
        )
        self.sub

        self.pub = self.create_publisher(
            Bytes, 'hardware_feedback', 10)

        timer_period = self.declare_parameter('timer_period', 0.01).value # 10 ms
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def connect_serial_port(self, port: str, baudrate: int = 9600, timeout: int = 1000):
        """Connect to the specified serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        # the frequency of data receiving should be higher than the device sending
        self.serial_port = serial.Serial(port, baudrate, timeout=timeout) 
        if self.serial_port.is_open:
            self.is_connected = True
            self.serial_port.set_buffer_size(rx_size=100, tx_size=100)
            print(f"Connected to {port} at {baudrate} baud.")
        else:
            self.is_connected = False
            print(f"Failed to connect to {port}.")

    def close_serial_port(self):
        """Close the serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            print("Serial port closed.")

    def send_str_command(self, command: str, expect_response: bool = False) -> Optional[str]:
        """Send a command to the motor controller"""
        try:
            if self.serial_port and self.serial_port.is_open:
                # print(f"Sending command: {command}")
                self.serial_port.reset_input_buffer()
                self.serial_port.write((command).encode())
                if expect_response:
                    return self.read_response()
            else:
                raise Exception("Serial port is not open.")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None


    def read_str_response(self) -> Optional[str]:
        if self.check_buffer:
            condition = self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0
        else:
            condition = self.serial_port and self.serial_port.is_open
        condition = self.serial_port and self.serial_port.is_open
        if condition:
            # t1 = time.time()
            # response = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
            response = self.serial_port.read_until(b'\r').decode("utf-8", errors="ignore").strip()
            while self.serial_port.in_waiting > 0:
                # response = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                response = self.serial_port.read_until(b'\r').decode("utf-8", errors="ignore").strip()
            self.response_buffer = response
            # current_time = time.time()
            # elapsed_time = (current_time - t1) * 1000  # Convert to milliseconds
            # print(f"read_response elapsed time: {elapsed_time:.2f} ms")
            # print(f"Received response: {response}","byte waiting:",self.serial_port.in_waiting)
            if response  == self.serial_port_node.GetParameter("Data"):
                pass
            else:
                self.serial_port_node.SetParameter("Data", response )
            return self.response_buffer
        return None
    
    def send_command(self, command: bytes, expect_response: bool = False) -> Optional[bytes]:
        """Send a command to the motor controller"""
        try:
            if self.serial_port and self.serial_port.is_open:
                # print(f"Sending command: {command}")
                if expect_response:
                    self.serial_port.reset_input_buffer()
                self.serial_port.write(self.startMarker+command+self.endMarker)
                if expect_response:
                    return self.read_response()
            else:
                raise Exception("Serial port is not open.")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None


    def read_response(self) -> Optional[bytes]:
        if self.check_buffer:
            condition = self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0
        else:
            condition = self.serial_port and self.serial_port.is_open
        # condition = self.serial_port and self.serial_port.is_open
        if condition:
            self.serial_port.read_until(self.startMarker)
            response = self.serial_port.read_until(self.endMarker)
            while self.serial_port.in_waiting > 0:
                self.serial_port.read_until(self.startMarker)
                response = self.serial_port.read_until(self.endMarker)
            self.response_buffer = response[:-1]
            if response  == self.serial_port_node.GetParameter("Data"):
                pass
            else:
                self.serial_port_node.SetParameter("Data", response )
            return self.response_buffer
        # elif self.serial_port.in_waiting <= 0:
        #     print("Serial input buffer is empty.")
        return None
    
    def input_callback(self, msg):
        self.command_buffer = msg.byte_msg

    def timer_callback(self):
        self.send_command()
        response = self.read_response()
        if response is not None:
            out = Bytes()
            out.header.stamp = self.get_clock().now().to_msg()
            out.byte_msg = response
            self.pub.publish(out)
    
    # def set_timer_interval(self, interval: int):
    #     """Set the timer interval for reading responses"""
    #     self.timer.setInterval(interval)

def main():
    rclpy.init()
    node = SerialCommunication()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
