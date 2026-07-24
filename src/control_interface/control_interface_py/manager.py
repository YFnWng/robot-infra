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
import math
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
        self.declare_parameter("source_timeout_s", 0.5)
        self.SOURCE_TIMEOUT = float(
            self.get_parameter("source_timeout_s").value)
        if self.SOURCE_TIMEOUT <= 0.0:
            raise ValueError("source_timeout_s must be positive")

        # Exclusive-control arbitration: a "priority" source (the automation
        # pipeline) claims exclusive control so manual teleop can't override its
        # velocity / joint limits during a data-collection run. It holds the lock
        # from its first event until it releases with MODE=NONE, or until it goes
        # silent longer than LOCK_TIMEOUT (crash safety). A STOP_MOTOR from any
        # source is always honoured.
        self.declare_parameter("priority_sources", ["autonomy"])
        self.priority_sources = set(self.get_parameter("priority_sources").value)
        self.locked_source = None
        self.LOCK_TIMEOUT = 1.0

        # Hard joint-limit clamp (safety net for BOTH automation and manual
        # teleop): every forwarded command is bounded to the active catheter
        # profile. VEL commands are clamped to +/- vel_max per joint and zeroed
        # for any joint already at a position limit and being pushed further out
        # (using the latest reported pose). POS targets are clamped into range.
        # No limits_file => no clamp.
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "default")
        self.declare_parameter("position_guard_horizon_s", 0.05)
        (self._pos_lower, self._pos_upper, self._vel_min,
         self._vel_max) = self._load_limits()
        self._position_guard_horizon_s = float(
            self.get_parameter("position_guard_horizon_s").value)
        if self._position_guard_horizon_s <= 0.0:
            raise ValueError("position_guard_horizon_s must be positive")
        self._last_pos = None

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

        # Source-level deadman. Command origins must publish live state
        # periodically; if Slicer/OpenIGTLink or automation disappears, send an
        # explicit zero before the Teensy's independent watchdog is needed.
        self.source_watchdog_timer = self.create_timer(
            0.05, self._source_watchdog_tick)

    # def deadman_cb(self, msg):
    #     self.deadman = msg.data

    def teleop_callback(self, msg: ControlStream):
        src = msg.header.frame_id
        self.last_input_time[src] = time.time()

        if self.deadman or \
            self.estop or \
            self.control_mode == ManagerEvent.NONE:
            return

        # Exclusive control: while a priority source holds the lock, only it may
        # command (a stale lock is auto-released inside _lock_blocks).
        if self._lock_blocks(src):
            return

        if self.active_source is None:
            self.active_source = src

        if src != self.active_source:
            return

        out = DeviceStream()
        out.header.stamp = self.get_clock().now().to_msg()
        # out.header.frame_id = msg.header.frame_id

        if self.control_mode == ManagerEvent.JOINT_VEL and msg.joint_vel:
            out.predicate = DeviceStream.VEL
            out.data = self._clamp_command(DeviceStream.VEL, msg.joint_vel)
        elif self.control_mode == ManagerEvent.JOINT_POS and msg.joint_pos:
            out.predicate = DeviceStream.POS
            out.data = self._clamp_command(DeviceStream.POS, msg.joint_pos)

        self.control_pub.publish(out)

    def _source_watchdog_tick(self):
        src = self.active_source
        if src is None:
            return
        age = time.time() - self.last_input_time[src]
        if age <= self.SOURCE_TIMEOUT:
            return

        out = DeviceStream()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = DeviceStream.VEL
        out.data = [0.0] * 6
        self.control_pub.publish(out)
        self.active_source = None
        self.get_logger().warn(
            f"control source '{src}' stale for {age:.3f}s — commanded zero velocity")

    def teleop_event_callback(self, msg: ManagerEvent):
        src = msg.header.frame_id
        self.last_input_time[src] = time.time()

        # While another source holds exclusive control, honour only a safety
        # STOP_MOTOR (without disturbing the lock or active source) and ignore
        # everything else.
        if self._lock_blocks(src):
            if msg.predicate == ManagerEvent.STOP_MOTOR:
                self._dispatch_device_command(msg)
            else:
                self.get_logger().warn(
                    f"event from '{src}' ignored — '{self.locked_source}' holds control")
            return

        # A priority source claims the lock on any event and releases it with
        # MODE=NONE; ordinary sources never lock.
        if src in self.priority_sources:
            releasing = (msg.predicate == ManagerEvent.MODE
                         and msg.text == chr(ManagerEvent.NONE))
            self.locked_source = None if releasing else src

        self.active_source = src

        if msg.predicate == ManagerEvent.MODE:
            if len(msg.text) != 1:
                self.get_logger().warn('Invalid control mode request')
                return

            self.control_mode = ord(msg.text)
            self.get_logger().info(f'Control mode switched to {msg.text}')
            return

        self._dispatch_device_command(msg)

    def _lock_blocks(self, src: str) -> bool:
        """True if `src` is locked out by another source's exclusive control.
        Auto-releases a stale lock (holder silent > LOCK_TIMEOUT) so a crashed
        holder can't shut teleop out forever."""
        if self.locked_source is None or src == self.locked_source:
            return False
        if time.time() - self.last_input_time[self.locked_source] > self.LOCK_TIMEOUT:
            self.get_logger().warn(
                f"exclusive-control holder '{self.locked_source}' went silent — releasing")
            self.locked_source = None
            return False
        return True

    def _dispatch_device_command(self, msg: ManagerEvent) -> None:
        req = DeviceCmd.Request()
        req.predicate = msg.predicate
        req.cmd = msg.text
        req.data = msg.data

        future = self.device_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_device_response(f, req)
        )

    def _on_device_response(self, future, req: DeviceCmd.Request):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Device service failed: {e}')
            return

        if not res.success:
            self.get_logger().warn(
                f'Device command failed: predicate={req.predicate}'
            )
            return

        # Publish manager event
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = req.predicate
        self.event_pub.publish(out)

        self.get_logger().info(f'Manager Command {req.predicate} completed')

    def device_state_callback(self, msg: DeviceStream):
        out = ManagerStream()
        out.header.stamp = self.get_clock().now().to_msg()
        if msg.predicate == DeviceStream.VEL:
            out.joint_vel = msg.data
        elif msg.predicate == DeviceStream.POS:
            out.joint_pos = msg.data
            self._last_pos = list(msg.data)          # cache for limit gating
        else:
            return  # e.g. ENC raw counts: recorded on /device/state, not forwarded
        self.state_pub.publish(out)

    def _load_limits(self):
        """Load position and reliable velocity bounds for all six joints."""
        path = self.get_parameter("limits_file").value
        if not path:
            return None, None, None, None
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
        name = self.get_parameter("catheter").value
        profiles = cfg.get("catheters", {})
        if name not in profiles:
            raise ValueError(f"catheter '{name}' not in {path}; have {list(profiles)}")
        p = profiles[name]
        out = []
        for k, raw in (
                ("pos_lower", p["pos_lower"]),
                ("pos_upper", p["pos_upper"]),
                ("vel_min", p.get("vel_min", [0.0] * 6)),
                ("vel_max", p["vel_max"])):
            vals = [float(x) for x in raw]
            if len(vals) != 6:
                raise ValueError(f"'{k}' for catheter '{name}' must have 6 values")
            out.append(vals)
        if any(value < 0.0 for value in out[2]):
            raise ValueError("vel_min values must be non-negative")
        if any(vmax <= 0.0 for vmax in out[3]):
            raise ValueError("vel_max values must be positive")
        if any(vmin > vmax for vmin, vmax in zip(out[2], out[3])):
            raise ValueError("vel_min values cannot exceed vel_max")
        self.get_logger().info(
            f"joint-limit clamp ACTIVE: catheter '{name}' from {path}")
        return tuple(out)

    def _clamp_command(self, predicate, data):
        """Bound a forwarded device command to the active catheter limits.

        VEL: preserve exact zero; otherwise enforce
        ``vel_min[j] <= |v[j]| <= vel_max[j]``. Position limits retain
        authority: if their safe velocity is below ``vel_min``, command zero.
        POS: target clamped into [pos_lower, pos_upper].
        """
        if self._vel_max is None:                    # no profile => no clamp
            return list(data)
        d = list(data)
        n = min(6, len(d))
        if predicate == DeviceStream.VEL:
            for j in range(n):
                vmax = self._vel_max[j]
                d[j] = max(-vmax, min(vmax, d[j]))
                vmin = self._vel_min[j]
                if 0.0 < abs(d[j]) < vmin:
                    d[j] = math.copysign(vmin, d[j])
                if self._last_pos is not None and j < len(self._last_pos):
                    p = self._last_pos[j]
                    if not math.isfinite(p):
                        d[j] = 0.0
                    elif p < self._pos_lower[j]:
                        d[j] = max(0.0, d[j])
                    elif p > self._pos_upper[j]:
                        d[j] = min(0.0, d[j])
                    else:
                        horizon = self._position_guard_horizon_s
                        min_velocity = (self._pos_lower[j] - p) / horizon
                        max_velocity = (self._pos_upper[j] - p) / horizon
                        d[j] = max(min_velocity, min(max_velocity, d[j]))
                # Never send a low-speed remnant created by the position guard.
                # Stopping is safer than crossing a hard limit at vel_min.
                if 0.0 < abs(d[j]) < vmin:
                    d[j] = 0.0
        elif predicate == DeviceStream.POS:
            for j in range(n):
                d[j] = max(self._pos_lower[j], min(self._pos_upper[j], d[j]))
        return d

    def device_event_callback(self, msg: DeviceEvent):
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = msg.predicate
        out.state = msg.state
        out.text = msg.text
        out.data = msg.data
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
