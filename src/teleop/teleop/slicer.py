
from control_interface.msg import ControlStream, ManagerStream, ManagerEvent
from geometry_msgs.msg import Point
import ros2_igtl_bridge.msg
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import math
import time


msg = """
This node takes commands from slicer through OpenIGTLink and publishes them
as ControlStream messages. 

CTRL-C to quit
"""

DOMAIN = "robot"
QUALIFY_DRIVER_POWER = ord('U')

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
        self.declare_parameter('input_timeout_s', 0.25)
        # must declare default value with the correct type, otherwise loading from yaml will fail silently.

        self.joints = self.get_parameter('joints').value
        self.keys = self.get_parameter('keys').value
        key_joint_idx = self.get_parameter('key_joint_idx').value
        directions = self.get_parameter('directions').value
        self.vel = list(self.get_parameter('joint_vels').value)
        self.input_timeout_s = float(
            self.get_parameter('input_timeout_s').value)

        if len(self.joints) != 6 or len(self.vel) != 6:
            raise ValueError('joints and joint_vels must each contain 6 values')
        if not all(math.isfinite(value) and value >= 0.0 for value in self.vel):
            raise ValueError('joint_vels must contain finite non-negative values')
        if not (
                len(self.keys) == len(key_joint_idx) == len(directions)):
            raise ValueError(
                'keys, key_joint_idx, and directions must have equal length')
        if len(set(self.keys)) != len(self.keys) or not all(
                isinstance(key, str) and len(key) == 1 for key in self.keys):
            raise ValueError('keys must contain unique single characters')
        if not all(
                0 <= int(joint) < 6 for joint in key_joint_idx):
            raise ValueError('key_joint_idx entries must be in [0, 5]')
        if not all(
                math.isfinite(direction) and direction in (-1, 1)
                for direction in directions):
            raise ValueError('directions entries must be -1 or +1')
        if self.input_timeout_s <= 0.0:
            raise ValueError('input_timeout_s must be positive')

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
        # ros2 bridge to openigtlink name limit: 20 bytes
        self.teleop_stream = ControlStream()
        self.teleop_stream.header.frame_id = "slicer"
        self.teleop_event = ManagerEvent()
        self.teleop_event.header.frame_id = "slicer"
        self.joint_pos_stream = ros2_igtl_bridge.msg.PointArray()
        self.joint_pos_stream.name = DOMAIN + "/joint_pos"
        self.joint_pos_stream.pointdata = [Point(x=0.0, y=0.0, z=0.0),
                                           Point(x=0.0, y=0.0, z=0.0)]
        self.manager_event = ros2_igtl_bridge.msg.String()
        self.manager_event.name = DOMAIN + "/event"
        self.latest_safety_payload = None

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

        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.manager_safety_sub = self.create_subscription(
            ManagerEvent,
            '/manager/safety_status',
            self.manager_safety_callback,
            safety_qos,
        )

        self.manager_stream_sub = self.create_subscription(
            ManagerStream,
            '/manager/state',
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
        self.qualify_client = self.create_client(
            Trigger, '/manager/qualify_driver_power')


        self.last_key_time = None
        self.key_motion_active = False
        self.input_watchdog = self.create_timer(
            0.05, self.input_watchdog_callback)
        self.safety_heartbeat = self.create_timer(
            1.0, self.safety_heartbeat_callback)
        self.get_logger().info(
            'Slicer bridge ready; input watchdog '
            f'{self.input_timeout_s:.2f}s')

    def _parse_igtl_name(self, name):
        parts = name.split('/', 1)
        if len(parts) != 2 or not parts[0] or not parts[1]:
            self.get_logger().warn(
                f'ignored malformed OpenIGTLink name: {name!r}')
            return None
        if parts[0] != DOMAIN:
            return None
        return parts[1]

    @staticmethod
    def _point_vector(msg):
        if len(msg.pointdata) < 2:
            raise ValueError('point array requires at least two points')
        values = [
            msg.pointdata[0].x, msg.pointdata[0].y, msg.pointdata[0].z,
            msg.pointdata[1].x, msg.pointdata[1].y, msg.pointdata[1].z,
        ]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('point array contains non-finite values')
        return values

    def _publish_zero_velocity(self):
        self.teleop_stream.joint_vel = [0.0] * 6
        self.teleop_stream.header.stamp = self.get_clock().now().to_msg()
        self.intent_pub.publish(self.teleop_stream)

    # Set targets
    def point_callback(self, msg):
        role = self._parse_igtl_name(msg.name)
        if role is None:
            return

        if role.startswith("joint_vel_target"): # openigtlink -> ros2 bridge adds number after node name
            try:
                values = self._point_vector(msg)
            except ValueError as exc:
                self.get_logger().warn(
                    f'ignored invalid velocity target: {exc}')
                return
            if any(value < 0.0 for value in values):
                self.get_logger().warn(
                    'ignored velocity target containing a negative speed')
                return
            self.vel = values
            self.get_logger().info(f"Set velocity: {self.vel}")
            # catheter lm motor doesn't move when velocity > 10.6mm/s

        elif role.startswith("joint_pos_target"):
            try:
                self.teleop_stream.joint_pos = self._point_vector(msg)
            except ValueError as exc:
                self.get_logger().warn(
                    f'ignored invalid position target: {exc}')
                return
            # The manager requires an atomic target + speed transaction.
            self.teleop_stream.joint_vel = list(self.vel)
            
            self.teleop_stream.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_stream)
            self.get_logger().info(f"Control position: {self.teleop_stream.joint_pos}")

    # Compute current motion based on pressed keys and publish
    def string_callback(self, msg):
        role = self._parse_igtl_name(msg.name)
        if role is None:
            return
        
        if role == "cmd": 

            n = len(msg.data)
            if n <= 0:
                return
            
            predicate = ord(msg.data[0])
            if predicate > 255:
                self.get_logger().warn(
                    'ignored command with a non-byte predicate')
                return
            if predicate == QUALIFY_DRIVER_POWER:
                self._request_driver_power_qualification()
                return
            self.teleop_event.predicate = predicate
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
            # Slicer appends '#<sequence>' so identical held-key states remain
            # observable as a live deadman heartbeat through OpenIGTLink.
            key_state = msg.data.split("#", 1)[0]
            self.last_key_time = time.monotonic()
            for key in key_state:
                # self.get_logger().info(f"key pressed: {key}")
                if key in self.keys:
                    # later key overrides
                    joint, dir = self.key_bindings[key]
                    self.teleop_stream.joint_vel[joint] = self.vel[joint]*dir

            self.teleop_stream.header.stamp = \
                self.get_clock().now().to_msg()
            self.intent_pub.publish(self.teleop_stream)
            self.key_motion_active = any(
                value != 0.0 for value in self.teleop_stream.joint_vel)
            self.get_logger().debug(
                f"Control joint vel: {self.teleop_stream.joint_vel}")

    def input_watchdog_callback(self):
        if (not self.key_motion_active or self.last_key_time is None
                or time.monotonic() - self.last_key_time
                <= self.input_timeout_s):
            return
        self._publish_zero_velocity()
        self.key_motion_active = False
        self.get_logger().warn(
            'Slicer key heartbeat stale; published zero velocity')

    def _publish_operator_event(self, predicate, subject):
        event = ros2_igtl_bridge.msg.String()
        event.name = DOMAIN + '/event'
        event.data = chr(predicate) + subject
        self.manager_event_pub.publish(event)

    def _request_driver_power_qualification(self):
        if not self.qualify_client.service_is_ready():
            self._publish_operator_event(
                QUALIFY_DRIVER_POWER,
                'FAILED:qualification service unavailable')
            self.get_logger().warn(
                'driver-power qualification service is unavailable')
            return
        future = self.qualify_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_driver_power_qualification)
        self._publish_operator_event(
            QUALIFY_DRIVER_POWER, 'IN_PROGRESS')

    def _on_driver_power_qualification(self, future):
        try:
            result = future.result()
            prefix = 'OK:' if result.success else 'FAILED:'
            message = prefix + result.message
        except Exception as exc:
            message = f'FAILED:{exc}'
        self._publish_operator_event(
            QUALIFY_DRIVER_POWER, message)

    def manager_event_callback(self, msg: ManagerEvent):
        # Safety state has a dedicated retained subscription below. Suppress
        # its volatile mirror here to avoid duplicate OpenIGTLink events.
        if (msg.predicate == ManagerEvent.FAULT_STATUS
                and msg.text.startswith('MANAGER_')):
            return
        self._forward_manager_event(msg)

    def manager_safety_callback(self, msg: ManagerEvent):
        self.latest_safety_payload = self._event_payload(msg)
        self._forward_manager_event(msg)

    @staticmethod
    def _event_payload(msg: ManagerEvent):
        if msg.text != "":
            subject = msg.text
        elif msg.state != []:
            subject = "".join(map(chr, msg.state))
        else:
            subject = ""
        return chr(msg.predicate) + subject

    def _forward_manager_event(self, msg: ManagerEvent):
        self.manager_event.data = self._event_payload(msg)
        self.manager_event_pub.publish(self.manager_event)

    def safety_heartbeat_callback(self):
        if self.latest_safety_payload is None:
            return
        self.manager_event.data = self.latest_safety_payload
        self.manager_event_pub.publish(self.manager_event)

    def manager_stream_callback(self, msg: ManagerStream):
        if len(msg.joint_pos) < 6:      # e.g. a joint_vel-only frame; nothing to show
            return
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
