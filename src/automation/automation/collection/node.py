"""Automated data-collection node: drive the robot with velocity commands.

Publishes ``control_interface/ControlStream`` velocity commands on
``/teleop/control`` (same path as manual teleop) after selecting ``JOINT_VEL``
mode via ``/teleop/event``. Emits run markers on ``/collection/events``.

This first stage supports ``mode:=constant`` (a single joint at a fixed,
speed-clamped velocity) for safe bring-up. ``mode:=sinusoidal`` (reusing the SOFA
``SinusoidalGenerator``) is added next.

Safety
------
* Velocities are clamped to ``joint_max_speeds`` per joint.
* Motors are NOT started unless ``start_motor:=true`` (default false) — so the
  node's output can be verified with ``ros2 topic echo`` before touching hardware.
* On stop / shutdown the node commands zero velocity.
"""
from __future__ import annotations

import json

from control_interface.msg import (
    ControlStream, DeviceEvent, DeviceStream, ManagerEvent)
from control_interface.srv import DeviceCmd
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Joint order matches teleop/config/params.yaml
JOINTS = [
    "catheter_lin", "catheter_rot", "catheter_bend",
    "sheath_lin", "sheath_rot", "sheath_bend",
]
TARGET_JOINTS = {"catheter": [0, 1, 2], "sheath": [3, 4, 5]}
ROT_JOINTS = {1, 4}   # rotation joints (deg, wrap-around): catheter_rot, sheath_rot
DEFAULT_MAX_SPEEDS = [5.0, 30.0, 1.0, 5.0, 30.0, 30.0]
DEFAULT_MIN_SPEEDS = [0.0] * 6


def parse_fault_status(response: str) -> dict:
    """Parse firmware ``Q`` response: V1,L=mask,E=mask,Q=seq,F=f0,..,f5."""
    fields = response.strip().split(",")
    if not fields or fields[0] != "V1":
        raise ValueError(f"unsupported fault-status response: {response!r}")
    values = {}
    fault_start = None
    for index, field in enumerate(fields[1:], start=1):
        if field.startswith("F="):
            fault_start = index
            break
        if "=" not in field:
            raise ValueError(f"malformed fault-status field: {field!r}")
        key, value = field.split("=", 1)
        values[key] = value
    if fault_start is None:
        raise ValueError("fault-status response has no F field")
    faults = [int(fields[fault_start].split("=", 1)[1])]
    faults.extend(int(value) for value in fields[fault_start + 1:])
    if len(faults) != 6:
        raise ValueError(f"fault-status response has {len(faults)} faults, expected 6")
    return {
        "version": 1,
        "latched_mask": int(values["L"], 16),
        "enabled_mask": int(values["E"], 16),
        "sequence": int(values["Q"]),
        "faults": faults,
        "raw": response,
    }


class CollectionNode(Node):
    """Velocity-command source for automated data collection."""

    def __init__(self) -> None:
        super().__init__("collection")

        self.declare_parameter("source_name", "autonomy")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("mode", "constant")          # constant | sinusoidal
        self.declare_parameter("target", "catheter")        # catheter | sheath
        self.declare_parameter("joint_min_speeds", DEFAULT_MIN_SPEEDS)
        self.declare_parameter("joint_max_speeds", DEFAULT_MAX_SPEEDS)
        self.declare_parameter("auto_enable", True)          # send MODE=JOINT_VEL
        self.declare_parameter("start_motor", False)         # send START_MOTOR
        self.declare_parameter("shutdown_on_done", True)
        # constant-mode params
        self.declare_parameter("test_joint", 0)              # absolute index 0..5
        self.declare_parameter("test_velocity", 0.0)         # units per joint
        # sinusoidal-mode params (target = the 3 driven joints)
        self.declare_parameter("sofa_sim_path", "/home/wangyf/sofa-cosserat-sim")
        self.declare_parameter("seed", -1)                   # <0 => random
        self.declare_parameter("joint_lower", [0.0, -180.0, 0.0])   # target joints
        self.declare_parameter("joint_upper", [0.1, 180.0, 0.01])   # PLACEHOLDER: set real limits
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("freq_change_interval", 10.0)
        self.declare_parameter("amp_range", [0.2, 1.0])
        # per-catheter pos + vel limits from YAML (overrides the joint_lower/upper
        # and joint_max_speeds params when limits_file is set)
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "default")
        self.declare_parameter("expect_enc", True)      # warn if no raw-ENC frames
        # return-to-start: after the trajectory, servo the target joints back to
        # the pose captured at run_start (proportional velocity, saturated to
        # vel_max, shortest angle for rotation joints).
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("return_kp", 1.0)          # gain, 1/s
        self.declare_parameter("return_tol", 0.5)         # mm or deg, per joint
        self.declare_parameter("return_timeout_s", 30.0)

        self._source = self.get_parameter("source_name").value
        self._rate = float(self.get_parameter("rate_hz").value)
        self._duration = float(self.get_parameter("duration_s").value)
        self._mode = self.get_parameter("mode").value
        self._target = self.get_parameter("target").value
        self._min_speeds = np.asarray(
            self.get_parameter("joint_min_speeds").value, dtype=float)
        self._max_speeds = np.asarray(
            self.get_parameter("joint_max_speeds").value, dtype=float)
        self._start_motor = bool(self.get_parameter("start_motor").value)
        self._shutdown_on_done = bool(self.get_parameter("shutdown_on_done").value)
        self._test_joint = int(self.get_parameter("test_joint").value)
        self._test_velocity = float(self.get_parameter("test_velocity").value)

        # per-catheter limits (6-joint pos_lower/upper + vel_max) override the
        # joint_lower/upper and joint_max_speeds params if a limits_file is given.
        self._pos_lower6, self._pos_upper6, _vel_min6, _vel_max6 = (
            self._load_limits())
        if _vel_min6 is not None:
            self._min_speeds = _vel_min6
        if _vel_max6 is not None:
            self._max_speeds = _vel_max6
        self._validate_velocity_bounds()

        if self._target not in TARGET_JOINTS:
            raise ValueError(f"target must be catheter|sheath, got {self._target}")
        self._target_idx = TARGET_JOINTS[self._target]

        self._gen = None
        self._shortest_delta = None
        self._h = 1.0 / self._rate
        if self._mode == "sinusoidal":
            self._build_generator()

        self._control_pub = self.create_publisher(ControlStream, "/teleop/control", 10)
        self._event_pub = self.create_publisher(ManagerEvent, "/teleop/event", 10)
        self._marker_pub = self.create_publisher(String, "/collection/events", 10)
        self._device_client = self.create_client(DeviceCmd, "/device/command")

        # Latest device state (firmware reports predicate 'P'/'V'/'E' + 6 values),
        # captured so the run_start marker records the initial joint configuration.
        self._last_state = None
        self._last_enc = None          # latest raw-ENC frame (start encoder counts)
        self._last_pos = None          # latest reported POS 6-vector (return feedback)
        self._enc_seen = False
        self.create_subscription(DeviceStream, "/device/state", self._state_cb, 10)
        self.create_subscription(
            DeviceEvent, "/device/event", self._device_event_cb, 10)

        self._t0 = None
        self._done = False
        self._returning = False
        self._return_t0 = None
        self._return_status = "not_started"
        self._return_elapsed_s = None
        self._start_pos = None         # pose captured at run_start (return target)
        self._run_status = "not_started"
        self._hardware_fault = None
        self._fault_status = None
        self._finish_return_result = None
        self._finish_status_timer = None
        self.should_exit = False

        # Enable after a short delay so publishers finish discovery.
        self._start_timer = self.create_timer(0.5, self._start)

    # -- lifecycle -------------------------------------------------------- #
    def _start(self) -> None:
        self._start_timer.cancel()
        if not self._device_client.wait_for_service(timeout_sec=1.0):
            self._abort_preflight("/device/command service unavailable")
            return
        request = DeviceCmd.Request()
        request.predicate = ManagerEvent.FAULT_STATUS
        future = self._device_client.call_async(request)
        future.add_done_callback(self._on_fault_status)

    def _on_fault_status(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.response or "fault-status query failed")
            self._fault_status = parse_fault_status(response.response)
        except Exception as exc:
            self._abort_preflight(f"fault-status query failed: {exc}")
            return
        if self._fault_status["latched_mask"]:
            self._abort_preflight(
                "firmware has latched motor faults", self._fault_status)
            return
        if self._fault_status["enabled_mask"]:
            self._abort_preflight(
                "firmware reports enabled motors before run", self._fault_status)
            return
        self._begin_run()

    def _abort_preflight(self, reason: str, status: dict | None = None) -> None:
        self._done = True
        self._run_status = "preflight_failed"
        self._marker(
            "run_end", status=self._run_status, preflight_error=reason,
            fault_status=status, enc_seen=self._enc_seen)
        self.get_logger().error(f"collection preflight failed: {reason}")
        if status is not None:
            self.get_logger().error(f"firmware fault status: {status}")
        if self._shutdown_on_done:
            self.should_exit = True

    def _begin_run(self) -> None:
        if self.get_parameter("auto_enable").value:
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_VEL))
            self.get_logger().info("requested JOINT_VEL mode")
        if self._start_motor:
            self._send_event(ManagerEvent.START_MOTOR)
            self.get_logger().warn("START_MOTOR sent — hardware may move")
        self._marker("run_start", mode=self._mode, target=self._target,
                     seed=int(self.get_parameter("seed").value),
                     start_state=self._state_snapshot())
        self._start_pos = list(self._last_pos) if self._last_pos is not None else None
        self._run_status = "running"
        if self.get_parameter("return_to_start").value and self._start_pos is None:
            self.get_logger().warn("no POS at run_start — return-to-start disabled")
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        if self.get_parameter("expect_enc").value:       # early ENC warning
            self._enc_timer = self.create_timer(3.0, self._check_enc_early)
        self.get_logger().info(
            f"collection running: mode={self._mode} target={self._target} "
            f"rate={self._rate}Hz duration={self._duration}s source={self._source}")

    def _check_enc_early(self) -> None:
        self._enc_timer.cancel()
        if not self._enc_seen:
            self.get_logger().error(
                "NO raw-ENC frames on /device/state after 3s — is the Teensy flashed "
                "with the ENC firmware? Recorded data will lack raw encoders.")

    def _tick(self) -> None:
        if self._done:
            return
        if self._returning:
            self._return_tick()
            return
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        done = t >= self._duration or (self._gen is not None and self._gen.is_done(t))
        if done:
            self._begin_return()
            return
        vel = self._apply_velocity_bounds(self._velocity(t))
        self._publish_velocity(vel)

    # -- return to start -------------------------------------------------- #
    def _begin_return(self) -> None:
        """Trajectory finished: settle at zero, then either servo the target
        joints back to the start pose or finish immediately."""
        for _ in range(5):
            self._publish_velocity(np.zeros(6))
        if (self.get_parameter("return_to_start").value
                and self._start_pos is not None and self._last_pos is not None):
            self._returning = True
            self._return_t0 = self.get_clock().now()
            self._return_status = "in_progress"
            tgt = [round(self._start_pos[j], 2) for j in self._target_idx]
            self._marker("return_start", target_joints=list(self._target_idx),
                         start_pose=tgt)
            self.get_logger().info(f"returning joints {self._target_idx} to start {tgt}")
        else:
            if self.get_parameter("return_to_start").value:
                self._return_status = "skipped_no_feedback"
                self.get_logger().warn("return-to-start skipped (no POS feedback)")
            else:
                self._return_status = "disabled"
            self._finish()

    def _return_error(self, joint: int) -> float:
        """Signed target-minus-current error for one physical joint."""
        error = self._start_pos[joint] - self._last_pos[joint]
        if joint in ROT_JOINTS:
            error = (error + 180.0) % 360.0 - 180.0
        return float(error)

    def _return_step(self):
        """Proportional velocity to drive target joints toward the start pose.
        Returns (vel6, done); rotation joints use the shortest signed angle."""
        kp = float(self.get_parameter("return_kp").value)
        tol = float(self.get_parameter("return_tol").value)
        v = np.zeros(6)
        done = True
        for j in self._target_idx:
            err = self._return_error(j)
            if abs(err) > tol:
                done = False
                v[j] = float(np.clip(kp * err,
                                     -self._max_speeds[j], self._max_speeds[j]))
        return self._apply_velocity_bounds(v), done

    def _return_tick(self) -> None:
        elapsed = (self.get_clock().now() - self._return_t0).nanoseconds * 1e-9
        v, done = self._return_step()
        if done or elapsed > float(self.get_parameter("return_timeout_s").value):
            for _ in range(5):
                self._publish_velocity(np.zeros(6))
            self._returning = False
            self._return_status = "succeeded" if done else "timed_out"
            self._return_elapsed_s = float(elapsed)
            self.get_logger().info(
                "returned to start" if done else "return-to-start timed out")
            self._finish()
            return
        self._publish_velocity(v)

    def _finish(self) -> None:
        if self._done:
            return
        if self._run_status == "running":
            self._run_status = "completed"
        self._done = True
        self._timer.cancel()
        if hasattr(self, "_enc_timer"):
            self._enc_timer.cancel()
        for _ in range(5):                       # settle at zero
            self._publish_velocity(np.zeros(6))
        self._finish_return_result = self._return_result()
        if self._start_motor or self._hardware_fault is not None:
            self._send_event(ManagerEvent.STOP_MOTOR)
        if self.get_parameter("auto_enable").value:
            # release the manager's exclusive-control lock and return it to idle
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.NONE))
        if self.get_parameter("expect_enc").value and not self._enc_seen:
            self.get_logger().error(
                "run finished with NO raw-ENC frames — recorded data lacks raw "
                "encoders (flash the ENC firmware).")

        # Let zero-velocity/STOP messages reach the firmware before querying
        # final safety state. The old run_end marker reused the clean preflight
        # response and therefore hid faults latched during the run.
        self._finish_status_timer = self.create_timer(
            0.1, self._request_final_fault_status)

    def _request_final_fault_status(self) -> None:
        self._finish_status_timer.cancel()
        request = DeviceCmd.Request()
        request.predicate = ManagerEvent.FAULT_STATUS
        future = self._device_client.call_async(request)
        future.add_done_callback(self._on_final_fault_status)

    def _on_final_fault_status(self, future) -> None:
        final_status = None
        status_error = None
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.response or "fault-status query failed")
            final_status = parse_fault_status(response.response)
        except Exception as exc:
            status_error = str(exc)
            self.get_logger().error(
                f"final fault-status query failed: {status_error}")

        return_result = self._finish_return_result
        self._marker(
            "run_end", status=self._run_status,
            hardware_fault=self._hardware_fault,
            preflight_fault_status=self._fault_status,
            fault_status=final_status,
            fault_status_error=status_error,
            enc_seen=self._enc_seen, return_result=return_result)
        if return_result["error_target_minus_final"] is not None:
            self.get_logger().info(
                "final return error (target-final) "
                f"{return_result['error_target_minus_final']}; "
                f"max_abs={return_result['max_abs_error']:.4f}; "
                f"status={return_result['status']}")
        self.get_logger().info("collection complete")
        if self._shutdown_on_done:
            self.should_exit = True

    def _return_result(self) -> dict:
        """Return-to-start outcome stored in the run_end event."""
        requested = bool(self.get_parameter("return_to_start").value)
        tolerance = float(self.get_parameter("return_tol").value)
        result = {
            "requested": requested,
            "status": self._return_status,
            "target_joint_indices": list(self._target_idx),
            "target_joint_names": [JOINTS[j] for j in self._target_idx],
            "units": [
                "deg" if j in ROT_JOINTS else "mm" for j in self._target_idx],
            "tolerance": tolerance,
            "elapsed_s": self._return_elapsed_s,
            "start_position": None,
            "final_position": None,
            "error_target_minus_final": None,
            "max_abs_error": None,
            "within_tolerance": None,
        }
        if self._start_pos is None or self._last_pos is None:
            return result
        errors = [self._return_error(j) for j in self._target_idx]
        result.update({
            "start_position": [
                float(self._start_pos[j]) for j in self._target_idx],
            "final_position": [
                float(self._last_pos[j]) for j in self._target_idx],
            "error_target_minus_final": errors,
            "max_abs_error": max(abs(error) for error in errors),
            "within_tolerance": all(
                abs(error) <= tolerance for error in errors),
        })
        return result

    # -- command generation ---------------------------------------------- #
    def _build_generator(self) -> None:
        import os
        import sys
        sofa_path = self.get_parameter("sofa_sim_path").value or os.environ.get(
            "SOFA_SIM_PATH", "")
        if sofa_path and sofa_path not in sys.path:
            sys.path.insert(0, sofa_path)
        try:
            from data_collection.generators.base import InputGenerator
            from data_collection.generators.sinusoidal import SinusoidalGenerator
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                f"cannot import SOFA generator from '{sofa_path}'; set sofa_sim_path "
                f"or SOFA_SIM_PATH: {exc}")

        self._shortest_delta = InputGenerator.shortest_rotation_delta
        idx = self._target_idx
        if self._pos_lower6 is not None:                 # from limits_file
            lower = self._pos_lower6[idx]
            upper = self._pos_upper6[idx]
        else:                                            # from direct params
            lower = np.asarray(self.get_parameter("joint_lower").value, dtype=float)
            upper = np.asarray(self.get_parameter("joint_upper").value, dtype=float)
        seed = int(self.get_parameter("seed").value)
        self._gen = SinusoidalGenerator(
            joint_lower_limits=lower,
            joint_upper_limits=upper,
            dt=self._h,
            joint_max_speeds=self._max_speeds[idx],
            speed_factor=float(self.get_parameter("speed_factor").value),
            duration=self._duration,
            freq_change_interval=float(self.get_parameter("freq_change_interval").value),
            amp_range=tuple(self.get_parameter("amp_range").value),
            seed=None if seed < 0 else seed,
        )
        self.get_logger().info(
            f"sinusoidal generator: target={self._target} joints={idx} "
            f"lower={lower} upper={upper} "
            f"min_speeds={self._min_speeds[idx]} "
            f"max_speeds={self._max_speeds[idx]} "
            f"seed={seed} auto_ramp_s={self._gen.ramp_duration:.3f}")

    def _velocity(self, t: float) -> np.ndarray:
        """Return a length-6 velocity vector for time t."""
        v = np.zeros(6)
        if self._mode == "constant":
            if 0 <= self._test_joint < 6:
                v[self._test_joint] = self._test_velocity
        elif self._mode == "sinusoidal":
            # velocity feed-forward = finite difference of the generator's
            # position, with the rotation joint (local index 1) unwrapped.
            q0 = self._gen.step(t)
            q1 = self._gen.step(t + self._h)
            v[self._target_idx] = self._shortest_delta(q0, q1) / self._h
        else:
            raise ValueError(f"unknown mode '{self._mode}'")
        return v

    def _validate_velocity_bounds(self) -> None:
        """Validate the six-joint reliable-speed interval."""
        if self._min_speeds.shape != (6,) or self._max_speeds.shape != (6,):
            raise ValueError("joint_min_speeds and joint_max_speeds must have 6 values")
        if (not np.all(np.isfinite(self._min_speeds))
                or not np.all(np.isfinite(self._max_speeds))):
            raise ValueError("joint velocity bounds must be finite")
        if np.any(self._min_speeds < 0.0) or np.any(self._max_speeds <= 0.0):
            raise ValueError("joint velocity bounds require 0 <= min and 0 < max")
        if np.any(self._min_speeds > self._max_speeds):
            raise ValueError("joint_min_speeds cannot exceed joint_max_speeds")

    def _apply_velocity_bounds(self, velocity) -> np.ndarray:
        """Clamp max speed and lift nonzero commands into the reliable band."""
        bounded = np.clip(
            np.asarray(velocity, dtype=float),
            -self._max_speeds, self._max_speeds)
        below_floor = (
            (np.abs(bounded) > 0.0)
            & (np.abs(bounded) < self._min_speeds))
        bounded[below_floor] = np.copysign(
            self._min_speeds[below_floor], bounded[below_floor])
        return bounded

    # -- state ----------------------------------------------------------- #
    def _state_cb(self, msg: DeviceStream) -> None:
        snap = {
            "predicate": chr(msg.predicate),          # 'P' pos, 'V' vel, 'E' enc
            "data": [float(x) for x in msg.data],
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self._last_state = snap
        if msg.predicate == DeviceStream.POS:
            self._last_pos = snap["data"]             # feedback for return-to-start
        elif msg.predicate == DeviceStream.ENC:
            self._enc_seen = True
            self._last_enc = snap                     # raw encoder counts

    def _device_event_cb(self, msg: DeviceEvent) -> None:
        if msg.predicate != ManagerEvent.STALL or len(msg.data) < 11:
            return
        data = list(msg.data)
        fault = {
            "text": msg.text,
            "protocol_version": int(data[0]),
            "transition": int(data[1]),
            "fault_type": int(data[2]),
            "axis": int(data[3]),
            "coupled_axis": int(data[4]),
            "sequence": int(data[5]),
            "commanded_velocity": float(data[6]),
            "measured_velocity": float(data[7]),
            "window_displacement": float(data[8]),
            "target_rpm": int(data[9]),
            "window_ms": int(data[10]),
            "detail": int(data[11]) if len(data) > 11 else 0,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        if fault["detail"]:
            fault["driver_stage"] = chr(fault["detail"])
        if fault["transition"] != ManagerEvent.MOTION_CONFIRMED:
            return
        if self._t0 is None or self._done:
            self._hardware_fault = fault
            return
        self._hardware_fault = fault
        self._run_status = "hardware_fault"
        self._returning = False
        self._return_status = "aborted_hardware_fault"
        self.get_logger().error(
            f"confirmed device fault — terminating collection: {fault}")
        self._finish()

    def _load_limits(self):
        """Load per-catheter position and velocity bounds for all six joints.

        ``vel_min`` is optional for compatibility and defaults to zero.
        """
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
        keys = ("pos_lower", "pos_upper", "vel_min", "vel_max")
        values = (
            p["pos_lower"], p["pos_upper"],
            p.get("vel_min", [0.0] * 6), p["vel_max"])
        out = tuple(np.asarray(value, dtype=float) for value in values)
        for arr, k in zip(out, keys):
            if arr.shape != (6,):
                raise ValueError(f"'{k}' for catheter '{name}' must have 6 values")
        self.get_logger().info(f"limits: catheter '{name}' from {path}")
        return out

    def _state_snapshot(self):
        """run_start initial condition: the latest device frame plus the latest
        raw-ENC frame (starting encoder counts, the model input)."""
        if self._last_state is None:
            self.get_logger().warn("no /device/state yet — run_start has no start_state")
        return {"last": self._last_state, "enc": self._last_enc}

    # -- publishing helpers ---------------------------------------------- #
    def _publish_velocity(self, vel: np.ndarray) -> None:
        msg = ControlStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._source
        msg.joint_vel = [float(x) for x in vel]
        self._control_pub.publish(msg)

    def _send_event(self, predicate: int, text: str = "") -> None:
        ev = ManagerEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.header.frame_id = self._source
        ev.predicate = predicate
        ev.text = text
        self._event_pub.publish(ev)

    def _marker(self, event: str, **fields) -> None:
        payload = {"event": event, "stamp_ns": self.get_clock().now().nanoseconds}
        payload.update(fields)
        self._marker_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollectionNode()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
