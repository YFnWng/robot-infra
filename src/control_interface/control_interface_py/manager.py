#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_interface.msg import ControlStream, \
    DeviceStream, ManagerStream, DeviceEvent, ManagerEvent
from control_interface.srv import DeviceCmd

from collections import defaultdict, deque
import math
import threading
import time


def parse_fault_status(response: str) -> dict:
    """Parse firmware Q response: V1,L=mask,E=mask,Q=seq,F=f0,..,f5."""
    fields = response.strip().split(',')
    if not fields or fields[0] != 'V1':
        raise ValueError(f'unsupported fault-status response: {response!r}')
    values = {}
    fault_start = None
    for index, field in enumerate(fields[1:], start=1):
        if field.startswith('F='):
            fault_start = index
            break
        if '=' not in field:
            raise ValueError(f'malformed fault-status field: {field!r}')
        key, value = field.split('=', 1)
        values[key] = value
    if fault_start is None:
        raise ValueError('fault-status response has no F field')
    faults = [int(fields[fault_start].split('=', 1)[1])]
    faults.extend(int(value) for value in fields[fault_start + 1:])
    if len(faults) != 6:
        raise ValueError(
            f'fault-status response has {len(faults)} faults, expected 6')
    return {
        'version': 1,
        'latched_mask': int(values['L'], 16),
        'enabled_mask': int(values['E'], 16),
        'sequence': int(values['Q']),
        'faults': faults,
        'raw': response,
    }


def parse_driver_diagnostic(response: str, expected_axis=None) -> dict:
    """Parse a successful firmware H response and validate its driver ACK."""
    fields = response.strip().split(',')
    if len(fields) != 7 or fields[0] != 'OK_DRIVER_UART' or fields[1] != 'V1':
        raise ValueError(f'malformed driver diagnostic response: {response!r}')
    values = {}
    for field in fields[2:]:
        if '=' not in field:
            raise ValueError(f'malformed driver diagnostic field: {field!r}')
        key, value = field.split('=', 1)
        values[key] = value
    if set(values) != {'A', 'S', 'F', 'N', 'R'}:
        raise ValueError(f'incomplete driver diagnostic response: {response!r}')
    axis = int(values['A'])
    failure = int(values['F'])
    attempts = int(values['N'])
    response_hex = values['R']
    if expected_axis is not None and axis != expected_axis:
        raise ValueError(
            f'driver diagnostic returned axis {axis}, expected {expected_axis}')
    if values['S'] != 'K' or failure != 0 or attempts < 1:
        raise ValueError(
            f'driver diagnostic did not complete cleanly: {response!r}')
    if (not response_hex or len(response_hex) % 2
            or any(character not in '0123456789abcdefABCDEF'
                   for character in response_hex)):
        raise ValueError(
            f'driver diagnostic has invalid acknowledgement: {response!r}')
    return {
        'version': 1,
        'axis': axis,
        'stage': values['S'],
        'failure': failure,
        'attempts': attempts,
        'response': bytes.fromhex(response_hex),
        'raw': response,
    }


class ControlManager(Node):

    def __init__(self):
        super().__init__('control_manager')

        self.control_mode = ManagerEvent.NONE

        self.deadman = False
        self.scale = 0.3
        self.estop = False

        self.active_source = None
        self.last_input_time = defaultdict(lambda: 0.0)

        self.declare_parameter("allowed_sources", ["slicer", "autonomy"])
        self.allowed_sources = set(
            self.get_parameter("allowed_sources").value)
        self.declare_parameter('allow_debug_commands', False)
        self.allow_debug_commands = bool(
            self.get_parameter('allow_debug_commands').value)

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
        self.declare_parameter("catheter", "imricor_test")
        self.declare_parameter("require_limits", True)
        self.declare_parameter("position_guard_horizon_s", 0.05)
        (self._pos_lower, self._pos_upper, self._vel_min,
         self._vel_max) = self._load_limits()
        self._position_guard_horizon_s = float(
            self.get_parameter("position_guard_horizon_s").value)
        if self._position_guard_horizon_s <= 0.0:
            raise ValueError("position_guard_horizon_s must be positive")
        self._last_pos = None
        self.declare_parameter("feedback_timeout_s", 0.25)
        self.FEEDBACK_TIMEOUT = float(
            self.get_parameter("feedback_timeout_s").value)
        if self.FEEDBACK_TIMEOUT <= 0.0:
            raise ValueError("feedback_timeout_s must be positive")
        self._last_pos_time = None
        self._last_enc_time = None
        self._fault_latched = False
        self._fault_reason = ""
        self._last_safety_status = None
        self._transport_ready = False
        self._driver_power_qualified = False
        self._shutdown_started = False
        self._qualification_lock = threading.Lock()
        self._pos_history = deque(maxlen=2000)
        self._enc_history = deque(maxlen=2000)
        self.declare_parameter('qualification_window_s', 0.5)
        self.declare_parameter('qualification_timeout_s', 4.0)
        self.declare_parameter('qualification_pos_tolerance', 0.05)
        self.declare_parameter('qualification_enc_tolerance_counts', 64.0)
        self.QUALIFICATION_WINDOW = float(
            self.get_parameter('qualification_window_s').value)
        self.QUALIFICATION_TIMEOUT = float(
            self.get_parameter('qualification_timeout_s').value)
        self.QUALIFICATION_POS_TOLERANCE = float(
            self.get_parameter('qualification_pos_tolerance').value)
        self.QUALIFICATION_ENC_TOLERANCE = float(
            self.get_parameter('qualification_enc_tolerance_counts').value)
        if min(
                self.QUALIFICATION_WINDOW,
                self.QUALIFICATION_TIMEOUT,
                self.QUALIFICATION_POS_TOLERANCE,
                self.QUALIFICATION_ENC_TOLERANCE) <= 0.0:
            raise ValueError('driver-power qualification parameters must be positive')
        self.qualification_group = ReentrantCallbackGroup()
        self.device_client_group = ReentrantCallbackGroup()

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

        transport_qos = QoSProfile(depth=1)
        transport_qos.reliability = ReliabilityPolicy.RELIABLE
        transport_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.transport_status_sub = self.create_subscription(
            String, '/device/transport_status',
            self.transport_status_callback, transport_qos)

        # publishers
        self.control_pub = self.create_publisher(
            DeviceStream, '/manager/control', 10)
        
        self.state_pub = self.create_publisher(
            ManagerStream, '/manager/state', 10)
        
        self.event_pub = self.create_publisher(
            ManagerEvent, '/manager/event', 10)
        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.safety_status_pub = self.create_publisher(
            ManagerEvent, '/manager/safety_status', safety_qos)

        self.qualification_service = self.create_service(
            Trigger, '/manager/qualify_driver_power',
            self.qualify_driver_power,
            callback_group=self.qualification_group)
        self.device_client = self.create_client(
            DeviceCmd, '/device/command',
            callback_group=self.device_client_group)
        while not self.device_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Source-level deadman. Command origins must publish live state
        # periodically; if Slicer/OpenIGTLink or automation disappears, send an
        # explicit zero before the Teensy's independent watchdog is needed.
        self.source_watchdog_timer = self.create_timer(
            0.05, self._source_watchdog_tick)
        self.feedback_watchdog_timer = self.create_timer(
            0.05, self._feedback_watchdog_tick)
        self._publish_safety_status(force=True)

    # def deadman_cb(self, msg):
    #     self.deadman = msg.data

    def teleop_callback(self, msg: ControlStream):
        src = msg.header.frame_id

        if src not in self.allowed_sources:
            self.get_logger().warn(
                f"ignored command from unapproved source {src!r}")
            return

        if self.control_mode == ManagerEvent.JOINT_VEL:
            if len(msg.joint_vel) != 6 or not all(
                    math.isfinite(value) for value in msg.joint_vel):
                self.get_logger().warn(
                    'ignored JOINT_VEL command: expected 6 finite values')
                return
        elif self.control_mode == ManagerEvent.JOINT_POS:
            if (len(msg.joint_pos) != 6 or len(msg.joint_vel) != 6
                    or not all(math.isfinite(value) for value in (
                        list(msg.joint_pos) + list(msg.joint_vel)))):
                self.get_logger().warn(
                    'ignored JOINT_POS command: expected 6 finite targets '
                    'and 6 finite speeds')
                return
        else:
            return

        if self._motion_inhibit_reason():
            self._publish_safety_status()
            return

        self.last_input_time[src] = time.monotonic()

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

        if self.control_mode == ManagerEvent.JOINT_VEL:
            out.predicate = DeviceStream.VEL
            out.data = self._clamp_command(DeviceStream.VEL, msg.joint_vel)
        elif self.control_mode == ManagerEvent.JOINT_POS:
            out.predicate = DeviceStream.POS
            target = self._clamp_command(DeviceStream.POS, msg.joint_pos)
            speed = self._clamp_position_speeds(msg.joint_vel)
            out.data = target + speed
        else:
            return

        self.control_pub.publish(out)

    def _source_watchdog_tick(self):
        src = self.active_source
        if src is None:
            return
        age = time.monotonic() - self.last_input_time[src]
        if age <= self.SOURCE_TIMEOUT:
            return

        if self.control_mode == ManagerEvent.JOINT_POS:
            stop = ManagerEvent()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = src
            stop.predicate = ManagerEvent.STOP_MOTOR
            self._dispatch_device_command(stop)
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self.get_logger().warn(
                f'control source {src!r} stale for {age:.3f}s - '
                'stopped position transaction')
            return

        out = DeviceStream()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = DeviceStream.VEL
        out.data = [0.0] * 6
        self.control_pub.publish(out)
        self.active_source = None
        self.control_mode = ManagerEvent.NONE
        self.get_logger().warn(
            f"control source '{src}' stale for {age:.3f}s — commanded zero velocity")

    def teleop_event_callback(self, msg: ManagerEvent):
        src = msg.header.frame_id
        if src not in self.allowed_sources:
            self.get_logger().warn(
                f"ignored event from unapproved source {src!r}")
            return

        if msg.predicate == ManagerEvent.MODE:
            if len(msg.text) != 1 or ord(msg.text) not in (
                    ManagerEvent.NONE, ManagerEvent.JOINT_VEL,
                    ManagerEvent.JOINT_POS, ManagerEvent.DEBUG):
                self.get_logger().warn('Invalid control mode request')
                return
            if (ord(msg.text) == ManagerEvent.DEBUG
                    and not self.allow_debug_commands):
                self._reject_event(msg, 'DEBUG_DISABLED')
                return

        self.last_input_time[src] = time.monotonic()

        # STOP is always accepted, even while another source holds the lock or
        # a device fault is latched.
        if msg.predicate == ManagerEvent.STOP_MOTOR:
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self._dispatch_device_command(msg)
            return

        # START_MOTOR is a legacy command with no reliable firmware transaction.
        # Driver readiness is established only through the guarded qualification
        # service, never by optimistically enabling motors.
        if msg.predicate == ManagerEvent.START_MOTOR:
            self._reject_event(msg, 'START_MOTOR_UNSUPPORTED')
            return

        if (msg.predicate in (
                ManagerEvent.DEBUG, ManagerEvent.DRIVER_DIAGNOSTIC)
                and not self.allow_debug_commands):
            self._reject_event(msg, 'DEBUG_DISABLED')
            return

        # Raw resets can erase evidence of a real runtime fault. Startup encoder
        # faults must use /manager/qualify_driver_power; diagnostic users can
        # still call the low-level device service deliberately.
        if msg.predicate == ManagerEvent.RESET_FAULT:
            self._reject_event(msg, 'USE_DRIVER_POWER_QUALIFICATION')
            return

        if msg.predicate == ManagerEvent.CONNECTION:
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self._driver_power_qualified = False
            self._dispatch_device_command(msg)
            self._publish_safety_status(force=True)
            return

        if msg.predicate == ManagerEvent.SET_ZERO:
            reason = self._motion_inhibit_reason()
            if reason:
                self._reject_event(msg, f'SET_ZERO_INHIBITED:{reason}')
                return
            if (self.control_mode != ManagerEvent.NONE
                    or self.active_source is not None):
                self._reject_event(msg, 'SET_ZERO_REQUIRES_MODE_NONE')
                return
            if not self._feedback_is_stable():
                self._reject_event(
                    msg, 'SET_ZERO_REQUIRES_STATIONARY_FEEDBACK')
                return

        if (self._fault_latched and msg.predicate not in (
                ManagerEvent.RESET_FAULT, ManagerEvent.FAULT_STATUS,
                ManagerEvent.CONNECTION, ManagerEvent.DRIVER_DIAGNOSTIC)):
            self.get_logger().warn(
                f"ignored predicate={msg.predicate}: manager fault is latched")
            self._publish_safety_status()
            return

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

        if msg.predicate == ManagerEvent.MODE:
            requested_mode = ord(msg.text)
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            if requested_mode != ManagerEvent.NONE \
                    and self._motion_inhibit_reason():
                if self.locked_source == src:
                    self.locked_source = None
                self._publish_safety_status()
                return
            self.control_mode = requested_mode
            if requested_mode != ManagerEvent.NONE:
                self.active_source = src
            self.get_logger().info(f'Control mode switched to {msg.text}')
            return

        self._dispatch_device_command(msg)

    def _reject_event(self, msg, reason):
        self.get_logger().warn(
            f'rejected predicate={msg.predicate}: {reason}')
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'manager'
        out.predicate = msg.predicate
        out.text = f'COMMAND_REJECTED:{reason}'
        self.event_pub.publish(out)

    def _lock_blocks(self, src: str) -> bool:
        """True if `src` is locked out by another source's exclusive control.
        Auto-releases a stale lock (holder silent > LOCK_TIMEOUT) so a crashed
        holder can't shut teleop out forever."""
        if self.locked_source is None or src == self.locked_source:
            return False
        lock_age = time.monotonic() - self.last_input_time[self.locked_source]
        if lock_age > self.LOCK_TIMEOUT:
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
                f'Device command failed: predicate={req.predicate}: '
                f'{res.response}'
            )
            self._fault_latched = True
            self._fault_reason = (
                f'DEVICE_COMMAND_FAILED:{req.predicate}:{res.response}')
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self._publish_safety_status(force=True)
            return

        if req.predicate == ManagerEvent.RESET_FAULT:
            self._fault_latched = False
            self._fault_reason = ""
            self._publish_safety_status(force=True)

        # Publish manager event
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = req.predicate
        self.event_pub.publish(out)

        self.get_logger().info(f'Manager Command {req.predicate} completed')

    def device_state_callback(self, msg: DeviceStream):
        now = time.monotonic()
        out = ManagerStream()
        out.header.stamp = self.get_clock().now().to_msg()
        if msg.predicate == DeviceStream.VEL:
            if len(msg.data) != 6 or not all(
                    math.isfinite(value) for value in msg.data):
                self.get_logger().warn('ignored malformed VEL feedback')
                return
            out.joint_vel = msg.data
        elif msg.predicate == DeviceStream.POS:
            if len(msg.data) != 6 or not all(
                    math.isfinite(value) for value in msg.data):
                self.get_logger().warn('ignored malformed POS feedback')
                return
            out.joint_pos = msg.data
            self._last_pos = list(msg.data)          # cache for limit gating
            self._last_pos_time = now
            self._pos_history.append((now, tuple(msg.data)))
        elif msg.predicate == DeviceStream.ENC:
            if len(msg.data) != 6 or not all(
                    math.isfinite(value) for value in msg.data):
                self.get_logger().warn('ignored malformed ENC feedback')
                return
            self._last_enc_time = now
            self._enc_history.append((now, tuple(msg.data)))
            self._publish_safety_status()
            return
        else:
            return
        self.state_pub.publish(out)
        self._publish_safety_status()

    def _feedback_is_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        stamps = (self._last_pos_time, self._last_enc_time)
        return all(stamp is not None and now - stamp <= self.FEEDBACK_TIMEOUT
                   for stamp in stamps)

    def _motion_inhibit_reason(self):
        if self._fault_latched:
            return self._fault_reason or 'DEVICE_FAULT'
        if not self._transport_ready:
            return 'TRANSPORT_NOT_READY'
        if not self._feedback_is_fresh():
            return 'FEEDBACK_NOT_QUALIFIED'
        if not self._driver_power_qualified:
            return 'DRIVER_POWER_NOT_QUALIFIED'
        return ''

    @staticmethod
    def _history_is_stable(history, now, window_s, tolerance):
        """Require complete, recent coverage with bounded per-axis range."""
        samples = list(history)
        cutoff = now - window_s
        if (len(samples) < 2 or samples[0][0] > cutoff
                or now - samples[-1][0] > 0.25):
            return False
        window = [values for stamp, values in samples if stamp >= cutoff]
        if len(window) < 2:
            return False
        return all(
            max(values[axis] for values in window)
            - min(values[axis] for values in window) <= tolerance
            for axis in range(6))

    def _feedback_is_stable(self):
        now = time.monotonic()
        return (
            self._feedback_is_fresh(now)
            and self._history_is_stable(
                self._pos_history, now, self.QUALIFICATION_WINDOW,
                self.QUALIFICATION_POS_TOLERANCE)
            and self._history_is_stable(
                self._enc_history, now, self.QUALIFICATION_WINDOW,
                self.QUALIFICATION_ENC_TOLERANCE))

    def _wait_for_stable_feedback(self):
        deadline = time.monotonic() + self.QUALIFICATION_TIMEOUT
        while time.monotonic() < deadline:
            if not self._transport_ready:
                return False, 'serial transport became unavailable'
            if self._feedback_is_stable():
                return True, ''
            time.sleep(0.02)
        return False, (
            f'POS/ENC feedback was not stationary for '
            f'{self.QUALIFICATION_WINDOW:.2f}s')

    def _call_device_sync(self, predicate, timeout=1.5, cmd=''):
        request = DeviceCmd.Request()
        request.predicate = predicate
        request.cmd = cmd
        future = self.device_client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            return False, f'predicate {predicate} timed out'
        try:
            result = future.result()
        except Exception as exc:
            return False, f'predicate {predicate} failed: {exc}'
        return bool(result.success), result.response

    def _probe_all_drivers(self):
        for axis in range(6):
            ok, message = self._call_device_sync(
                ManagerEvent.DRIVER_DIAGNOSTIC,
                timeout=2.0,
                cmd=str(axis))
            if not ok:
                return False, (
                    f'driver UART probe failed on axis {axis}: {message}')
            try:
                parse_driver_diagnostic(message, expected_axis=axis)
            except (KeyError, TypeError, ValueError) as exc:
                return False, (
                    f'driver UART probe invalid on axis {axis}: {exc}')
        return True, 'all 6 driver UART probes passed'

    def qualify_driver_power(self, request, response):
        """Explicitly qualify a driver power-up before motion is permitted."""
        del request
        if not self._qualification_lock.acquire(blocking=False):
            response.success = False
            response.message = 'driver-power qualification already in progress'
            return response
        try:
            self._driver_power_qualified = False
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self._publish_safety_status(
                detail='DRIVER_POWER_QUALIFICATION_IN_PROGRESS', force=True)

            if not self._transport_ready:
                # CONNECT sent while driver power was absent can be discarded
                # during Teensy/driver initialization. Retry it now that the
                # operator has explicitly requested power qualification.
                ok, message = self._call_device_sync(
                    ManagerEvent.CONNECTION, timeout=2.0)
                if not ok:
                    response.success = False
                    response.message = (
                        f'serial reconnect attempt failed: {message}')
                    return response
                ready_deadline = time.monotonic() + 2.0
                while (not self._transport_ready
                       and time.monotonic() < ready_deadline):
                    time.sleep(0.02)
                if not self._transport_ready:
                    response.success = False
                    response.message = (
                        'serial opened but no bidirectional POS/ENC telemetry '
                        'was established')
                    return response

            ok, message = self._call_device_sync(ManagerEvent.STOP_MOTOR)
            if not ok:
                response.success = False
                response.message = f'firmware STOP failed: {message}'
                return response

            stable, reason = self._wait_for_stable_feedback()
            if not stable:
                response.success = False
                response.message = reason
                return response

            ok, message = self._call_device_sync(ManagerEvent.FAULT_STATUS)
            if not ok:
                response.success = False
                response.message = f'fault-status query failed: {message}'
                return response
            status = parse_fault_status(message)
            if status['enabled_mask']:
                response.success = False
                response.message = (
                    f"firmware motors remain enabled: E="
                    f"{status['enabled_mask']:02X}")
                return response

            nonstartup = [
                axis for axis, fault in enumerate(status['faults'])
                if fault not in (
                    0, ManagerEvent.FAULT_ENCODER_INTEGRITY)]
            if nonstartup:
                response.success = False
                response.message = (
                    'refusing startup reset; non-encoder faults on axes '
                    f'{nonstartup}: {status["raw"]}')
                return response

            if status['latched_mask']:
                ok, message = self._call_device_sync(
                    ManagerEvent.RESET_FAULT)
                if not ok:
                    response.success = False
                    response.message = f'encoder-latch reset failed: {message}'
                    return response

                stable, reason = self._wait_for_stable_feedback()
                if not stable:
                    response.success = False
                    response.message = (
                        f'post-reset feedback qualification failed: {reason}')
                    return response

            probes_ok, probe_message = self._probe_all_drivers()
            if not probes_ok:
                response.success = False
                response.message = probe_message
                return response

            ok, message = self._call_device_sync(ManagerEvent.FAULT_STATUS)
            if not ok:
                response.success = False
                response.message = (
                    f'post-reset fault-status query failed: {message}')
                return response
            final_status = parse_fault_status(message)
            if (final_status['latched_mask']
                    or final_status['enabled_mask']
                    or any(final_status['faults'])):
                response.success = False
                response.message = (
                    f'firmware did not qualify cleanly: {final_status["raw"]}')
                return response

            self._fault_latched = False
            self._fault_reason = ''
            self._driver_power_qualified = True
            self._publish_safety_status(force=True)
            response.success = True
            response.message = (
                'driver power qualified: stable POS/ENC, all 6 driver UARTs '
                'responsive, motors disabled, firmware fault status clean')
            return response
        except (KeyError, TypeError, ValueError) as exc:
            response.success = False
            response.message = f'driver-power qualification failed: {exc}'
            return response
        finally:
            if not self._driver_power_qualified:
                self._publish_safety_status(force=True)
            self._qualification_lock.release()

    def initiate_shutdown(self):
        """Best-effort zero and firmware STOP before ROS teardown."""
        if self._shutdown_started:
            return None
        self._shutdown_started = True
        self._driver_power_qualified = False
        self.active_source = None
        self.locked_source = None
        self.control_mode = ManagerEvent.NONE
        # The default ROS SIGINT handler may invalidate the context before
        # executor.spin() returns. ROS publishers and clients are unusable in
        # that case; the serial node still owns the direct firmware STOP barrier.
        if not rclpy.ok():
            return None
        try:
            self._command_zero_velocity()
        except Exception as exc:
            self.get_logger().warn(
                f'could not publish shutdown zero velocity: {exc}')
        if not self.device_client.service_is_ready():
            self.get_logger().warn(
                'device service unavailable during graceful shutdown; '
                'serial-node and firmware watchdog stops remain active')
            return None
        request = DeviceCmd.Request()
        request.predicate = ManagerEvent.STOP_MOTOR
        try:
            future = self.device_client.call_async(request)
        except Exception as exc:
            self.get_logger().warn(
                f'failed to request firmware STOP during shutdown: {exc}')
            return None
        self.get_logger().info(
            'graceful shutdown: zero velocity and firmware STOP requested')
        return future

    def _command_zero_velocity(self):
        out = DeviceStream()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = DeviceStream.VEL
        out.data = [0.0] * 6
        self.control_pub.publish(out)

    def _publish_safety_status(self, detail=None, force=False):
        reason = detail or self._motion_inhibit_reason()
        status = f'MANAGER_INHIBITED:{reason}' if reason else 'MANAGER_READY'
        if not force and status == self._last_safety_status:
            return
        self._last_safety_status = status
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = ManagerEvent.FAULT_STATUS
        out.text = status
        self.safety_status_pub.publish(out)
        self.event_pub.publish(out)

    def _feedback_watchdog_tick(self):
        if self.active_source is None or self._feedback_is_fresh():
            return
        self._command_zero_velocity()
        self.active_source = None
        self.control_mode = ManagerEvent.NONE
        self._publish_safety_status(force=True)
        self.get_logger().error(
            'device feedback became stale during motion; manager inhibited')

    def transport_status_callback(self, msg: String):
        ready = msg.data.startswith('SERIAL_READY:')
        was_ready = self._transport_ready
        was_active = self.active_source is not None
        self._transport_ready = ready
        if not ready or not was_ready:
            self._driver_power_qualified = False
        if not ready and was_active:
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
            self.get_logger().error(
                f'serial transport lost during motion: {msg.data}')
        self._publish_safety_status(force=not ready and was_active)

    def _load_limits(self):
        """Load position and reliable velocity bounds for all six joints."""
        path = self.get_parameter("limits_file").value
        if not path:
            if self.get_parameter("require_limits").value:
                raise ValueError(
                    'limits_file is required; refusing to start without limits')
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

    def _clamp_position_speeds(self, data):
        '''Clamp physical motor-axis speed magnitudes for a POS transaction.'''
        values = [abs(float(value)) for value in data]
        if len(values) != 6:
            raise ValueError('position speed vector must contain 6 values')
        if self._vel_max is None:
            return values
        for joint in range(6):
            values[joint] = min(values[joint], self._vel_max[joint])
            if 0.0 < values[joint] < self._vel_min[joint]:
                values[joint] = self._vel_min[joint]
        return values

    def device_event_callback(self, msg: DeviceEvent):
        confirmed_transition = False
        if len(msg.data) >= 2 and math.isfinite(msg.data[1]):
            confirmed_transition = (
                int(msg.data[1]) == ManagerEvent.MOTION_CONFIRMED)
        confirmed = (msg.text.startswith('MOTION_CONFIRMED:')
                     or confirmed_transition)
        if confirmed:
            self._fault_latched = True
            self._fault_reason = msg.text or 'CONFIRMED_DEVICE_FAULT'
            self._command_zero_velocity()
            self.active_source = None
            self.control_mode = ManagerEvent.NONE
        out = ManagerEvent()
        out.header.stamp = self.get_clock().now().to_msg()
        out.predicate = msg.predicate
        out.state = msg.state
        out.text = msg.text
        out.data = msg.data
        self.event_pub.publish(out)
        if confirmed:
            self._publish_safety_status(force=True)

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

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.initiate_shutdown()
        # Let the service request leave this process before its executor and DDS
        # entities are destroyed. The serial node also emits a direct STOP frame
        # during its own teardown.
        time.sleep(0.05)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
