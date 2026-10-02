"""Lightweight non-serial device process for model-in-the-loop simulation."""
from __future__ import annotations

import math
import json
import threading
import time

from control_interface.msg import ControlStream, DeviceEvent, DeviceStream, ManagerEvent
from control_interface.srv import DeviceCmd
from control_interface_py.command_freshness import validate_command_stamp
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from catheter_control.safety.hardware_contract import load_hardware_contract
from .sim_plant import SimulatedActuatorPlant, position_servo_velocity
from .sim_perturbations import ActuatorConfig, ActuatorPerturbation


class SimulatedDeviceNode(Node):
    def __init__(self):
        super().__init__("catheter_sim_device")
        self.declare_parameter("simulation_only", True)
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "imricor_test")
        self.declare_parameter("plant_rate_hz", 100.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("command_max_age_s", 0.10)
        self.declare_parameter("command_future_tolerance_s", 0.05)
        self.declare_parameter(
            "position_tolerance", [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
        self.declare_parameter("position_settle_s", 0.1)
        self.declare_parameter(
            "initial_encoder_counts", [0, 0, 0, 0, 0, 0])
        self.declare_parameter("actuator_gain", [1.0]*6)
        self.declare_parameter("actuator_deadband_rad_s", [0.0]*6)
        self.declare_parameter("actuator_time_constant_s", [0.0]*6)
        self.declare_parameter("actuator_command_delay_s", 0.0)
        self.declare_parameter("actuator_reversal_backlash_rad", [0.0]*6)
        self.declare_parameter(
            "actuator_reversal_backlash_positive_rad", [0.0]*6)
        self.declare_parameter(
            "actuator_reversal_backlash_negative_rad", [0.0]*6)
        self.declare_parameter("actuator_initial_backlash_unengaged", False)
        if not bool(self.get_parameter("simulation_only").value):
            raise ValueError("simulation_only must be true")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate_hz = float(self.get_parameter("plant_rate_hz").value)
        self.timeout_s = float(self.get_parameter("command_timeout_s").value)
        if self.rate_hz <= 0.0 or self.timeout_s <= 0.0:
            raise ValueError("simulation rate/timeouts must be positive")
        contract = load_hardware_contract(
            str(self.get_parameter("limits_file").value),
            str(self.get_parameter("catheter").value))
        self.contract = contract
        self.position_tolerance = np.asarray(
            self.get_parameter("position_tolerance").value,
            dtype=np.float64)
        self.position_settle_s = float(
            self.get_parameter("position_settle_s").value)
        if (self.position_tolerance.shape != (6,)
                or not np.all(np.isfinite(self.position_tolerance))
                or np.any(self.position_tolerance <= 0.0)):
            raise ValueError(
                "position_tolerance must contain six positive finite values")
        if (not math.isfinite(self.position_settle_s)
                or self.position_settle_s <= 0.0):
            raise ValueError("position_settle_s must be finite and positive")
        self.actuator_config = ActuatorConfig(
            gain=tuple(self.get_parameter("actuator_gain").value),
            deadband_rad_s=tuple(self.get_parameter(
                "actuator_deadband_rad_s").value),
            time_constant_s=tuple(self.get_parameter(
                "actuator_time_constant_s").value),
            command_delay_s=float(self.get_parameter(
                "actuator_command_delay_s").value),
            reversal_backlash_rad=tuple(self.get_parameter(
                "actuator_reversal_backlash_rad").value),
            reversal_backlash_positive_rad=tuple(self.get_parameter(
                "actuator_reversal_backlash_positive_rad").value),
            reversal_backlash_negative_rad=tuple(self.get_parameter(
                "actuator_reversal_backlash_negative_rad").value),
            initial_backlash_unengaged=bool(self.get_parameter(
                "actuator_initial_backlash_unengaged").value))
        self.actuator = ActuatorPerturbation(
            self.actuator_config, 1.0/self.rate_hz)
        self.plant = SimulatedActuatorPlant(
            contract, timestamp_ns=max(
                1, int(self.get_clock().now().nanoseconds)),
            step_s=1.0/self.rate_hz,
            initial_encoder_counts=list(
                self.get_parameter("initial_encoder_counts").value),
            actuator_perturbation=self.actuator)
        self._lock = threading.RLock()
        self._last_arrival = None
        self._last_stamp_ns = 0
        self._watchdog_reported = False
        self._position_target = None
        self._position_speed = None
        self._position_active = False
        self._position_terminal = False
        self._position_within_since = None
        self._position_mask = 0
        self._plant_reset_count = 0

        motion_qos = QoSProfile(depth=1)
        motion_qos.reliability = ReliabilityPolicy.RELIABLE
        motion_qos.durability = DurabilityPolicy.VOLATILE
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        callbacks = ReentrantCallbackGroup()
        self.create_subscription(
            DeviceStream, "/sim/manager/control", self._control_cb,
            motion_qos, callback_group=callbacks)
        self.state_pub = self.create_publisher(
            DeviceStream, "/sim/device/state", 10)
        self.transmitted_state_pub = self.create_publisher(
            DeviceStream, "/sim/catheter_sim/transmitted_state", 10)
        self.event_pub = self.create_publisher(
            DeviceEvent, "/sim/device/event", 10)
        self.transport_pub = self.create_publisher(
            String, "/sim/device/transport_status", transient_qos)
        self.command_service = self.create_service(
            DeviceCmd, "/sim/device/command", self._device_command_cb,
            callback_group=callbacks)
        self.reset_service = self.create_service(
            Trigger, "/sim/catheter_sim/reset_plant",
            self._reset_plant_cb, callback_group=callbacks)
        self.joint_pub = self.create_publisher(
            JointState, "/sim/catheter_sim/joint_states", 10)
        self.realized_pub = self.create_publisher(
            ControlStream, "/sim/catheter_sim/realized_control", 10)
        self.projected_pub = self.create_publisher(
            ControlStream, "/sim/catheter_sim/projected_control", 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/sim/catheter_sim/device_status", 10)
        self.timer = self.create_timer(1.0/self.rate_hz, self._tick)
        self.status_timer = self.create_timer(0.2, self._status)
        self._transport_ready()
        self.get_logger().warn(
            "SIMULATION ONLY device active under /sim; no serial imports or "
            "physical transports are present")

    def _transport_ready(self):
        message = String()
        message.data = "SERIAL_READY:SIMULATED_V171"
        self.transport_pub.publish(message)

    def _control_cb(self, message):
        expected = {DeviceStream.VEL: 6, DeviceStream.POS: 12}
        if (message.predicate not in expected
                or len(message.data) != expected[message.predicate]
                or not all(math.isfinite(value) for value in message.data)):
            with self._lock:
                self._cancel_position()
                self.plant.stop()
            self.get_logger().error(
                "rejected malformed simulated command: "
                f"predicate={message.predicate} length={len(message.data)}")
            return
        zero = (message.predicate == DeviceStream.VEL
                and all(value == 0.0 for value in message.data))
        accepted, reason, stamp_ns = validate_command_stamp(
            message, now_ns=self.get_clock().now().nanoseconds,
            last_stamp_ns=self._last_stamp_ns,
            maximum_age_s=float(
                self.get_parameter("command_max_age_s").value),
            future_tolerance_s=float(
                self.get_parameter("command_future_tolerance_s").value))
        if not accepted and not zero:
            self.get_logger().warn(f"rejected simulated command: {reason}")
            return
        with self._lock:
            if message.predicate == DeviceStream.VEL:
                self._cancel_position()
                if zero:
                    self.plant.stop()
                else:
                    self.plant.set_command(message.data)
                    self._last_arrival = time.monotonic()
                    self._watchdog_reported = False
            else:
                target = np.asarray(message.data[:6], dtype=np.float64)
                speed = np.abs(np.asarray(message.data[6:], dtype=np.float64))
                if not self.contract.position_is_valid(target):
                    self._cancel_position()
                    self.plant.stop()
                    self._publish_position_status(
                        ManagerEvent.POSITION_REJECTED, target)
                    self.get_logger().error(
                        "rejected simulated position target outside hard limits")
                    return
                same_target = (
                    self._position_target is not None
                    and np.allclose(target, self._position_target,
                                    rtol=0.0, atol=1e-6))
                try:
                    # Validate zero-speed axes before accepting the atomic
                    # transaction. The actual command is recomputed in _tick.
                    position_servo_velocity(
                        self.plant.joint_position, target, speed,
                        self.position_tolerance, self.plant.step_s)
                except ValueError as error:
                    self._cancel_position()
                    self.plant.stop()
                    self._publish_position_status(
                        ManagerEvent.POSITION_REJECTED, target)
                    self.get_logger().error(
                        f"rejected simulated position transaction: {error}")
                    return
                if not same_target:
                    self._position_target = target.copy()
                    self._position_speed = speed.copy()
                    self._position_active = True
                    self._position_terminal = False
                    self._position_within_since = None
                    moving = (np.abs(target-self.plant.joint_position)
                              > self.position_tolerance)
                    self._position_mask = sum(
                        (1 << int(axis)) for axis in np.flatnonzero(moving))
                self._last_arrival = time.monotonic()
                self._watchdog_reported = False
            if accepted:
                self._last_stamp_ns = stamp_ns

    def _cancel_position(self):
        self._position_target = None
        self._position_speed = None
        self._position_active = False
        self._position_terminal = False
        self._position_within_since = None
        self._position_mask = 0

    def _publish_position_status(self, status, target=None):
        message = DeviceEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.predicate = ManagerEvent.POSITION_STATUS
        names = {
            ManagerEvent.POSITION_COMPLETE: "POSITION_COMPLETE",
            ManagerEvent.POSITION_TIMED_OUT: "POSITION_TIMED_OUT",
            ManagerEvent.POSITION_REJECTED: "POSITION_REJECTED",
        }
        message.text = names.get(status, "POSITION_UNKNOWN")
        error = (np.zeros(6, dtype=np.float64) if target is None
                 else np.asarray(target)-self.plant.joint_position)
        message.data = [
            1.0, float(status), float(self._position_mask),
            *[float(value) for value in error]]
        self.event_pub.publish(message)

    def _device_command_cb(self, request, response):
        predicate = int(request.predicate)
        if predicate == ManagerEvent.SET_ZERO:
            response.success = False
            response.response = "SET_ZERO_FORBIDDEN"
        elif predicate == ManagerEvent.CONNECTION:
            self._transport_ready()
            response.success = True
            response.response = "SIMULATED_V171_CONNECTED"
        elif predicate == ManagerEvent.STOP_MOTOR:
            with self._lock:
                self._cancel_position()
                self.plant.stop()
            response.success = True
            response.response = "OK"
        elif predicate == ManagerEvent.FAULT_STATUS:
            response.success = True
            response.response = "V1,L=00,E=00,Q=0,F=0,0,0,0,0,0"
        elif predicate == ManagerEvent.RESET_FAULT:
            with self._lock:
                self._cancel_position()
                self.plant.stop()
            response.success = True
            response.response = "OK"
        elif predicate == ManagerEvent.DRIVER_DIAGNOSTIC:
            try:
                axis = int(request.cmd)
            except ValueError:
                axis = -1
            response.success = 0 <= axis < 6
            response.response = (
                f"OK_DRIVER_UART,V1,A={axis},S=K,F=0,N=1,R=4F4B"
                if response.success else "ERR_AXIS")
        else:
            response.success = False
            response.response = f"UNSUPPORTED_SIM_COMMAND:{predicate}"
        return response

    def _reset_plant_cb(self, _request, response):
        """Simulation-only reset of hidden actuator/transmission state."""
        with self._lock:
            if self._position_active:
                response.success = False
                response.message = "position_transaction_active"
                return response
            self._cancel_position()
            self.plant.reset_transmission()
            self._plant_reset_count += 1
            self._last_arrival = None
            self._watchdog_reported = False
        response.success = True
        response.message = "simulation plant transmission reset"
        self.get_logger().info(response.message)
        return response

    def _tick(self):
        completed_target = None
        with self._lock:
            timestamp_ns = max(
                self.plant.timestamp_ns+1,
                int(self.get_clock().now().nanoseconds))
            integration_step_s = (
                timestamp_ns-self.plant.timestamp_ns) * 1e-9
            if (self._last_arrival is not None
                    and time.monotonic()-self._last_arrival > self.timeout_s
                    and np.any(self.plant.requested_velocity != 0.0)):
                self._cancel_position()
                self.plant.stop()
                if not self._watchdog_reported:
                    self._watchdog_reported = True
                    self.get_logger().warn(
                        "simulated firmware watchdog stopped stale command")
            if self._position_active:
                try:
                    command = position_servo_velocity(
                        self.plant.joint_position, self._position_target,
                        self._position_speed, self.position_tolerance,
                        integration_step_s)
                    self.plant.set_command(command)
                except ValueError as error:
                    target = self._position_target.copy()
                    self._cancel_position()
                    self.plant.stop()
                    self.get_logger().error(
                        f"simulated position transaction failed: {error}")
                    self._publish_position_status(
                        ManagerEvent.POSITION_REJECTED, target)
            try:
                snapshot = self.plant.step(timestamp_ns)
            except (ValueError, RuntimeError) as error:
                self.plant.stop()
                self.get_logger().error(
                    f"simulated device fault: {type(error).__name__}: {error}")
                return
            if self._position_active:
                within = np.all(
                    np.abs(self._position_target-snapshot.joint_position)
                    <= self.position_tolerance)
                now_monotonic = time.monotonic()
                if within:
                    if self._position_within_since is None:
                        self._position_within_since = now_monotonic
                    elif (now_monotonic-self._position_within_since
                          >= self.position_settle_s):
                        completed_target = self._position_target.copy()
                        self._position_active = False
                        self._position_terminal = True
                        self.plant.stop()
                else:
                    self._position_within_since = None
        if completed_target is not None:
            self._publish_position_status(
                ManagerEvent.POSITION_COMPLETE, completed_target)
            self.get_logger().info(
                "simulated position transaction complete")
        stamp = rclpy.time.Time(nanoseconds=timestamp_ns).to_msg()
        for predicate, values in (
                (DeviceStream.POS, snapshot.joint_position),
                (DeviceStream.ENC, snapshot.encoder_counts)):
            message = DeviceStream()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.predicate = predicate
            message.data = [float(value) for value in values]
            self.state_pub.publish(message)
        transmitted = DeviceStream()
        transmitted.header.stamp = stamp
        transmitted.header.frame_id = self.frame_id
        transmitted.predicate = DeviceStream.ENC
        transmitted.data = [
            float(value) for value in snapshot.transmitted_encoder_counts]
        self.transmitted_state_pub.publish(transmitted)
        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = self.frame_id
        joints.name = [
            "catheter_linear", "catheter_rotation", "catheter_bend",
            "sheath_linear", "sheath_rotation", "sheath_bend"]
        joints.position = [float(value) for value in snapshot.joint_position]
        joints.velocity = [
            float(value) for value in snapshot.realized_logical_velocity]
        self.joint_pub.publish(joints)
        realized = ControlStream()
        realized.header = joints.header
        realized.joint_vel = joints.velocity
        self.realized_pub.publish(realized)
        projected = ControlStream()
        projected.header = joints.header
        projected.joint_vel = [
            float(value) for value in snapshot.projected_logical_velocity]
        self.projected_pub.publish(projected)

    def _status(self):
        self._transport_ready()
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "catheter_control/simulated_device"
        status.hardware_id = "no_serial"
        status.message = "RUNNING"
        status.values = [
            KeyValue(key="simulation_only", value="True"),
            KeyValue(key="serial_present", value="False"),
            KeyValue(key="actuator_gain", value=json.dumps(
                list(self.actuator_config.gain))),
            KeyValue(key="actuator_deadband_rad_s", value=json.dumps(
                list(self.actuator_config.deadband_rad_s))),
            KeyValue(key="actuator_time_constant_s", value=json.dumps(
                list(self.actuator_config.time_constant_s))),
            KeyValue(key="actuator_command_delay_s", value=str(
                self.actuator_config.command_delay_s)),
            KeyValue(key="actuator_reversal_backlash_rad", value=json.dumps(
                list(self.actuator_config.reversal_backlash_rad))),
            KeyValue(key="plant_reset_count", value=str(
                self._plant_reset_count))]
        report.status = [status]
        self.status_pub.publish(report)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDeviceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            node.plant.stop()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
