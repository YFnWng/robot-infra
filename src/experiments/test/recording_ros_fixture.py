"""Synthetic telemetry only; no command publishers or device interfaces."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import PointCloud
from control_interface.msg import DeviceStream


def main():
    rclpy.init()
    node = Node("recording_test_telemetry")
    markers = node.create_publisher(PointCloud, "/shape_tracking/markers", 10)
    device = node.create_publisher(DeviceStream, "/device/state", 10)
    status = node.create_publisher(DiagnosticArray, "/catheter_mppi/status", 10)

    def publish():
        markers.publish(PointCloud())
        for predicate in (DeviceStream.POS, DeviceStream.ENC):
            device.publish(DeviceStream(predicate=predicate, data=[0.0] * 6))
        status.publish(DiagnosticArray(status=[DiagnosticStatus(message="DISARMED")]))

    node.create_timer(0.05, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
