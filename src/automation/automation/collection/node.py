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
* Motion starts only after fresh POS and ENC feedback remains stationary and
  the target joints are inside the selected catheter limits.
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
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String

from .identification import IdentificationConfig, IdentificationGenerator

# Joint order matches teleop/config/params.yaml
JOINTS = [
    "catheter_lin", "catheter_rot", "catheter_bend",
    "sheath_lin", "sheath_rot", "sheath_bend",
]
TARGET_JOINTS = {"catheter": [0, 1, 2], "sheath": [3, 4, 5]}
ROT_JOINTS = {1, 4}   # rotation joints (deg, wrap-around): catheter_rot, sheath_rot
DEFAULT_MAX_SPEEDS = [5.0, 30.0, 4.9, 5.0, 30.0, 30.0]
DEFAULT_MIN_SPEEDS = [0.0] * 6
DEFAULT_PREFLIGHT_POSITION_DRIFT = [0.1, 1.0, 0.1, 0.1, 1.0, 1.0]


def collection_marker_qos() -> QoSProfile:
    """Retain complete run markers for late-discovering recorders.

    rosbag2 requests transient-local durability for this event topic. A volatile
    publisher is incompatible with that request and can lose ``run_start`` while
    DDS discovery is still settling. Retaining all markers from one run also
    lets a recorder reconnect before the node exits without losing chronology.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=128,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


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
        self.declare_parameter("mode", "constant")  # constant | sinusoidal | identification
        self.declare_parameter("target", "catheter")        # catheter | sheath
        self.declare_parameter("joint_min_speeds", DEFAULT_MIN_SPEEDS)
        self.declare_parameter("joint_max_speeds", DEFAULT_MAX_SPEEDS)
        # A nonzero reliable-speed floor cannot be applied directly to a smooth
        # feed-forward velocity without changing the integrated trajectory.
        # Track the generator position with zero/minimum-speed hysteresis instead.
        self.declare_parameter("floor_tracking_enabled", True)
        self.declare_parameter("floor_tracking_kp", 2.0)
        self.declare_parameter("floor_tracking_enter_time_s", 0.10)
        self.declare_parameter("floor_tracking_exit_time_s", 0.05)
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
        # Identification-mode parameters, in physical target-joint units.
        self.declare_parameter(
            "identification_amplitudes", [20.0, 100.0, 6.0])
        self.declare_parameter(
            "identification_margins", [0.5, 5.0, 0.25])
        self.declare_parameter(
            "identification_minimum_amplitudes", [2.0, 20.0, 1.0])
        self.declare_parameter("identification_settle_s", 2.0)
        self.declare_parameter("identification_dwell_s", 1.0)
        self.declare_parameter("identification_hold_s", 2.0)
        self.declare_parameter("identification_slow_fraction", 0.30)
        self.declare_parameter("identification_medium_fraction", 0.70)
        self.declare_parameter("identification_max_duration_s", 900.0)
        # per-catheter pos + vel limits from YAML (overrides the joint_lower/upper
        # and joint_max_speeds params when limits_file is set)
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "imricor_test")
        self.declare_parameter("expect_enc", True)      # warn if no raw-ENC frames
        # Feedback qualification before MODE/START or any velocity publication.
        self.declare_parameter("preflight_feedback_timeout_s", 3.0)
        self.declare_parameter("preflight_stability_s", 0.5)
        self.declare_parameter("preflight_max_feedback_age_s", 0.25)
        self.declare_parameter("preflight_position_limit_tolerance", 0.1)
        self.declare_parameter(
            "preflight_position_drift", DEFAULT_PREFLIGHT_POSITION_DRIFT)
        self.declare_parameter("preflight_encoder_drift_counts", 100.0)
        self.declare_parameter("preflight_require_enc", True)
        # Return after the trajectory. Position mode is encoder-qualified and
        # can target either run-start or encoder zero; velocity mode remains a
        # selectable fallback.
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
        self._floor_tracking_enabled = bool(
            self.get_parameter("floor_tracking_enabled").value)
        self._floor_tracking_kp = float(
            self.get_parameter("floor_tracking_kp").value)
        self._floor_tracking_enter_time_s = float(
            self.get_parameter("floor_tracking_enter_time_s").value)
        self._floor_tracking_exit_time_s = float(
            self.get_parameter("floor_tracking_exit_time_s").value)
        self._start_motor = bool(self.get_parameter("start_motor").value)
        self._shutdown_on_done = bool(self.get_parameter("shutdown_on_done").value)
        self._test_joint = int(self.get_parameter("test_joint").value)
        self._test_velocity = float(self.get_parameter("test_velocity").value)
        self._preflight_timeout_s = float(
            self.get_parameter("preflight_feedback_timeout_s").value)
        self._preflight_stability_s = float(
            self.get_parameter("preflight_stability_s").value)
        self._preflight_max_age_s = float(
            self.get_parameter("preflight_max_feedback_age_s").value)
        self._preflight_limit_tolerance = float(
            self.get_parameter("preflight_position_limit_tolerance").value)
        self._preflight_position_drift = np.asarray(
            self.get_parameter("preflight_position_drift").value, dtype=float)
        self._preflight_encoder_drift = float(
            self.get_parameter("preflight_encoder_drift_counts").value)
        self._preflight_require_enc = bool(
            self.get_parameter("preflight_require_enc").value)
        self._validate_preflight_parameters()

        # Position return is the default; the legacy velocity servo remains
        # selectable for comparison and fallback.
        self.declare_parameter('return_control_mode', 'position')
        self.declare_parameter('return_to_zero', False)
        self.declare_parameter('return_position_speed_factor', 0.5)
        self.declare_parameter(
            'return_position_tolerance', [0.1, 0.5, 0.05, 0.1, 0.5, 0.5])
        self.declare_parameter('return_position_settle_s', 0.2)
        self.declare_parameter('return_position_mode_delay_s', 0.1)
        self._return_control_mode = str(
            self.get_parameter('return_control_mode').value).lower()
        self._return_to_zero = bool(
            self.get_parameter('return_to_zero').value)
        self._return_position_speed_factor = float(
            self.get_parameter('return_position_speed_factor').value)
        self._return_tolerances = np.asarray(
            self.get_parameter('return_position_tolerance').value, dtype=float)
        self._return_position_settle_s = float(
            self.get_parameter('return_position_settle_s').value)
        self._return_position_mode_delay_s = float(
            self.get_parameter('return_position_mode_delay_s').value)
        self._validate_position_return_parameters()

        # per-catheter limits (6-joint pos_lower/upper + vel_max) override the
        # joint_lower/upper and joint_max_speeds params if a limits_file is given.
        self._pos_lower6, self._pos_upper6, _vel_min6, _vel_max6 = (
            self._load_limits())
        if _vel_min6 is not None:
            self._min_speeds = _vel_min6
        if _vel_max6 is not None:
            self._max_speeds = _vel_max6
        self._validate_velocity_bounds()
        self._validate_floor_tracking_parameters()

        if self._target not in TARGET_JOINTS:
            raise ValueError(f"target must be catheter|sheath, got {self._target}")
        self._target_idx = TARGET_JOINTS[self._target]
        if self._mode not in ("constant", "sinusoidal", "identification"):
            raise ValueError(f"unknown mode '{self._mode}'")
        if self._mode == "identification" and self._target != "catheter":
            raise ValueError("identification mode currently supports target=catheter")

        self._gen = None
        self._shortest_delta = None
        self._floor_tracking_direction = np.zeros(6, dtype=np.int8)
        self._floor_reference_offset = np.zeros(6)
        self._h = 1.0 / self._rate
        self._episode_index = -1
        if self._mode == "sinusoidal":
            self._build_generator()

        self._control_pub = self.create_publisher(ControlStream, "/teleop/control", 10)
        self._event_pub = self.create_publisher(ManagerEvent, "/teleop/event", 10)
        self._marker_pub = self.create_publisher(
            String, "/collection/events", collection_marker_qos())
        self._device_client = self.create_client(DeviceCmd, "/device/command")

        # Latest device state (firmware reports predicate 'P'/'V'/'E' + 6 values),
        # captured so the run_start marker records the initial joint configuration.
        self._last_state = None
        self._last_enc = None          # latest raw-ENC frame (start encoder counts)
        self._last_pos = None          # latest reported POS 6-vector (return feedback)
        self._pos_history = []         # (receipt stamp_ns, length-6 data)
        self._enc_history = []
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
        self._return_target_pos = None
        self._return_position_speeds = None
        self._return_within_since_ns = None
        self._position_mode_ready_ns = None
        self._position_complete_seen = False
        self._position_status = None
        self._run_status = "not_started"
        self._hardware_fault = None
        self._fault_status = None
        self._finish_return_result = None
        self._finish_status_timer = None
        self._preflight_timer = None
        self._preflight_deadline_ns = None
        self._preflight_last_error = "feedback qualification has not started"
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
        now_ns = self.get_clock().now().nanoseconds
        self._preflight_deadline_ns = int(
            now_ns + self._preflight_timeout_s * 1e9)
        self._preflight_timer = self.create_timer(
            0.05, self._preflight_feedback_tick)
        self._preflight_feedback_tick()

    def _preflight_feedback_tick(self) -> None:
        """Wait for a fresh, in-range, stationary POS/ENC window."""
        if self._done:
            return
        now_ns = self.get_clock().now().nanoseconds
        error = self._feedback_preflight_error(now_ns)
        if error is None:
            if self._preflight_timer is not None:
                self._preflight_timer.cancel()
            self.get_logger().info(
                f"feedback preflight passed: {self._preflight_stability_s:.2f}s "
                "stationary POS/ENC window")
            self._begin_run()
            return
        self._preflight_last_error = error
        if now_ns >= self._preflight_deadline_ns:
            self._abort_preflight(
                f"feedback qualification timed out: {error}", self._fault_status)

    def _feedback_preflight_error(self, now_ns: int) -> str | None:
        """Return why feedback is not motion-safe yet, otherwise ``None``."""
        if self._last_pos is None:
            return "missing POS feedback"
        if self._preflight_require_enc and self._last_enc is None:
            return "missing ENC feedback"
        if len(self._last_pos) != 6:
            return f"POS feedback has {len(self._last_pos)} values, expected 6"
        if self._preflight_require_enc and len(self._last_enc["data"]) != 6:
            return (
                f"ENC feedback has {len(self._last_enc['data'])} values, expected 6")

        pos = np.asarray(self._last_pos, dtype=float)
        if not np.all(np.isfinite(pos)):
            return "POS feedback contains non-finite values"
        if self._preflight_require_enc:
            enc = np.asarray(self._last_enc["data"], dtype=float)
            if not np.all(np.isfinite(enc)):
                return "ENC feedback contains non-finite values"

        max_age_ns = int(self._preflight_max_age_s * 1e9)
        pos_age_ns = now_ns - self._pos_history[-1][0] if self._pos_history else None
        if pos_age_ns is None or pos_age_ns < 0 or pos_age_ns > max_age_ns:
            age = "unknown" if pos_age_ns is None else f"{pos_age_ns * 1e-9:.3f}s"
            return f"stale POS feedback ({age})"
        if self._preflight_require_enc:
            enc_age_ns = now_ns - self._enc_history[-1][0] if self._enc_history else None
            if enc_age_ns is None or enc_age_ns < 0 or enc_age_ns > max_age_ns:
                age = (
                    "unknown" if enc_age_ns is None
                    else f"{enc_age_ns * 1e-9:.3f}s")
                return f"stale ENC feedback ({age})"

        if self._pos_lower6 is not None:
            for joint in self._target_idx:
                lower = self._pos_lower6[joint]
                upper = self._pos_upper6[joint]
                if (pos[joint] < lower - self._preflight_limit_tolerance
                        or pos[joint] > upper + self._preflight_limit_tolerance):
                    return (
                        f"{JOINTS[joint]} position {pos[joint]:.6g} outside "
                        f"[{lower:.6g}, {upper:.6g}]")

        stability_ns = int(self._preflight_stability_s * 1e9)
        pos_window = CollectionNode._recent_stability_window(
            self._pos_history, stability_ns)
        if pos_window is None:
            return "POS stability window is incomplete"
        pos_samples = np.asarray([data for _, data in pos_window], dtype=float)
        pos_span = np.ptp(pos_samples, axis=0)
        for joint in self._target_idx:
            allowed = self._preflight_position_drift[joint]
            if pos_span[joint] > allowed:
                return (
                    f"{JOINTS[joint]} moved {pos_span[joint]:.6g} during "
                    f"preflight (allowed {allowed:.6g})")

        if self._preflight_require_enc:
            enc_window = CollectionNode._recent_stability_window(
                self._enc_history, stability_ns)
            if enc_window is None:
                return "ENC stability window is incomplete"
            enc_samples = np.asarray(
                [data for _, data in enc_window], dtype=float)
            enc_span = np.ptp(enc_samples, axis=0)
            unstable = np.flatnonzero(
                enc_span > self._preflight_encoder_drift)
            if unstable.size:
                joint = int(unstable[0])
                return (
                    f"{JOINTS[joint]} encoder moved {enc_span[joint]:.6g} counts "
                    f"during preflight (allowed "
                    f"{self._preflight_encoder_drift:.6g})")
        return None

    @staticmethod
    def _recent_stability_window(history, duration_ns: int):
        """Return the latest fully covered window, including its left sample."""
        if not history:
            return None
        cutoff = history[-1][0] - duration_ns
        start = None
        for index, (stamp_ns, _) in enumerate(history):
            if stamp_ns <= cutoff:
                start = index
            else:
                break
        if start is None:
            return None
        return history[start:]

    def _abort_preflight(self, reason: str, status: dict | None = None) -> None:
        if self._preflight_timer is not None:
            self._preflight_timer.cancel()
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
        if self._preflight_timer is not None:
            self._preflight_timer.cancel()
        self._start_pos = list(self._last_pos) if self._last_pos is not None else None
        if self._mode == "identification":
            try:
                self._build_identification_generator()
            except Exception as exc:
                self._abort_preflight(
                    f"identification trajectory validation failed: {exc}",
                    self._fault_status)
                return
        if self.get_parameter("auto_enable").value:
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_VEL))
            self.get_logger().info("requested JOINT_VEL mode")
        if self._start_motor:
            self._send_event(ManagerEvent.START_MOTOR)
            self.get_logger().warn("START_MOTOR sent — hardware may move")
        generator_metadata = (
            self._gen.metadata if self._mode == "identification" else None)
        self._marker("run_start", mode=self._mode, target=self._target,
                     seed=int(self.get_parameter("seed").value),
                     floor_tracking={
                         "enabled": self._floor_tracking_enabled,
                         "kp": self._floor_tracking_kp,
                         "enter_time_s": self._floor_tracking_enter_time_s,
                         "exit_time_s": self._floor_tracking_exit_time_s,
                     },
                     start_state=self._state_snapshot(),
                     generator=generator_metadata)
        self._initialize_floor_tracking()
        self._run_status = "running"
        if self.get_parameter("return_to_start").value and self._start_pos is None:
            self.get_logger().warn("no POS at run_start — return-to-start disabled")
        self._t0 = self.get_clock().now()
        self._update_episode_markers(0.0)
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
            self._update_episode_markers(self._duration, finishing=True)
            self._begin_return()
            return
        self._update_episode_markers(t)
        vel = self._trajectory_velocity(t)
        self._publish_velocity(vel)

    # -- return to start -------------------------------------------------- #
    def _velocity_return_begin_legacy(self) -> None:
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

    def _velocity_return_error_legacy(self, joint: int) -> float:
        """Signed target-minus-current error for one physical joint."""
        error = self._start_pos[joint] - self._last_pos[joint]
        if joint in ROT_JOINTS:
            error = (error + 180.0) % 360.0 - 180.0
        return float(error)

    def _velocity_return_step_legacy(self):
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

    def _velocity_return_tick_legacy(self) -> None:
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

    def _position_return_target(self) -> np.ndarray:
        '''Build a full logical target while leaving non-target joints fixed.'''
        target = np.asarray(self._last_pos, dtype=float).copy()
        source = (
            np.zeros(6) if self._return_to_zero
            else np.asarray(self._start_pos, dtype=float))
        target[self._target_idx] = source[self._target_idx]
        if self._pos_lower6 is not None:
            target = np.clip(target, self._pos_lower6, self._pos_upper6)
        return target

    def _begin_return(self) -> None:
        '''Stop the trajectory and start the selected return controller.'''
        for _ in range(5):
            self._publish_velocity(np.zeros(6))
        enabled = bool(self.get_parameter('return_to_start').value)
        if not enabled or self._start_pos is None or self._last_pos is None:
            self._return_status = (
                'disabled' if not enabled else 'skipped_no_feedback')
            if enabled:
                self.get_logger().warn(
                    'return skipped because POS feedback is unavailable')
            self._finish()
            return

        self._return_target_pos = self._position_return_target()
        self._returning = True
        self._return_t0 = self.get_clock().now()
        self._return_status = 'in_progress'
        self._return_within_since_ns = None
        self._position_complete_seen = False
        self._position_status = None
        target_values = [
            round(float(self._return_target_pos[j]), 3)
            for j in self._target_idx]
        self._marker(
            'return_start', target_joints=list(self._target_idx),
            target_pose=target_values,
            target_kind='zero' if self._return_to_zero else 'start',
            control_mode=self._return_control_mode)
        self.get_logger().info(
            f'returning joints {self._target_idx} to {target_values} '
            f'with {self._return_control_mode} control')

        if self._return_control_mode == 'position':
            # Stop the velocity transaction explicitly before changing modes;
            # cross-topic delivery order is not a safety boundary.
            self._send_event(ManagerEvent.STOP_MOTOR)
            self._return_position_speeds = np.zeros(6)
            speeds = np.maximum(
                self._min_speeds,
                self._return_position_speed_factor * self._max_speeds)
            self._return_position_speeds[self._target_idx] = speeds[
                self._target_idx]
            self._send_event(
                ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_POS))
            self._position_mode_ready_ns = int(
                self.get_clock().now().nanoseconds
                + self._return_position_mode_delay_s * 1e9)

    def _return_error(self, joint: int) -> float:
        '''Signed target-minus-current error for a return joint.'''
        error = self._return_target_pos[joint] - self._last_pos[joint]
        if joint in ROT_JOINTS:
            error = (error + 180.0) % 360.0 - 180.0
        return float(error)

    def _return_step(self):
        '''Legacy proportional velocity return, retained as a fallback.'''
        kp = float(self.get_parameter('return_kp').value)
        tolerance = float(self.get_parameter('return_tol').value)
        velocity = np.zeros(6)
        done = True
        for joint in self._target_idx:
            error = self._return_error(joint)
            if abs(error) > tolerance:
                done = False
                velocity[joint] = float(np.clip(
                    kp * error,
                    -self._max_speeds[joint], self._max_speeds[joint]))
        return self._apply_velocity_bounds(velocity), done

    def _position_return_done(self, now_ns: int) -> bool:
        if self._last_pos is None:
            self._return_within_since_ns = None
            return False
        within = all(
            abs(self._return_error(joint)) <= self._return_tolerances[joint]
            for joint in self._target_idx)
        if not within:
            self._return_within_since_ns = None
            return False
        if self._return_within_since_ns is None:
            self._return_within_since_ns = now_ns
            return False
        return (
            now_ns - self._return_within_since_ns
            >= int(self._return_position_settle_s * 1e9))

    def _complete_return(self, status: str, elapsed: float) -> None:
        if self._return_control_mode == 'position':
            self._send_event(ManagerEvent.STOP_MOTOR)
        else:
            for _ in range(5):
                self._publish_velocity(np.zeros(6))
        self._returning = False
        self._return_status = status
        self._return_elapsed_s = float(elapsed)
        self.get_logger().info(f'return finished with status={status}')
        self._finish()

    def _return_tick(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._return_t0).nanoseconds * 1e-9
        timeout = float(self.get_parameter('return_timeout_s').value)
        if self._return_control_mode == 'position':
            if self._position_status in (
                    ManagerEvent.POSITION_TIMED_OUT,
                    ManagerEvent.POSITION_REJECTED):
                status = (
                    'firmware_timed_out'
                    if self._position_status == ManagerEvent.POSITION_TIMED_OUT
                    else 'firmware_rejected')
                self._complete_return(status, elapsed)
                return
            if self._position_return_done(now.nanoseconds):
                self._complete_return('succeeded', elapsed)
                return
            if elapsed > timeout:
                self._complete_return('timed_out', elapsed)
                return
            if now.nanoseconds >= self._position_mode_ready_ns:
                self._publish_position(
                    self._return_target_pos, self._return_position_speeds)
            return

        velocity, done = self._return_step()
        if done or elapsed > timeout:
            self._complete_return(
                'succeeded' if done else 'timed_out', elapsed)
            return
        self._publish_velocity(velocity)

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
        if (self._start_motor or self._hardware_fault is not None
                or self._return_control_mode == 'position'):
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
        result['target_kind'] = 'zero' if self._return_to_zero else 'start'
        result['control_mode'] = self._return_control_mode
        result['target_position'] = None
        if self._return_target_pos is None or self._last_pos is None:
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
        result['target_position'] = [
            float(self._return_target_pos[j]) for j in self._target_idx]
        if self._return_control_mode == 'position':
            tolerances = [
                float(self._return_tolerances[j]) for j in self._target_idx]
            result['tolerance'] = tolerances
            result['within_tolerance'] = all(
                abs(error) <= allowed
                for error, allowed in zip(errors, tolerances))
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

    def _build_identification_generator(self) -> None:
        """Resolve and validate the full plan from measured run-start POS."""
        if self._start_pos is None:
            raise ValueError("fresh POS feedback is required")
        idx = self._target_idx
        if self._pos_lower6 is not None:
            lower = self._pos_lower6[idx]
            upper = self._pos_upper6[idx]
        else:
            lower = np.asarray(self.get_parameter("joint_lower").value, dtype=float)
            upper = np.asarray(self.get_parameter("joint_upper").value, dtype=float)
        seed = int(self.get_parameter("seed").value)
        config = IdentificationConfig(
            amplitudes=tuple(self.get_parameter(
                "identification_amplitudes").value),
            margins=tuple(self.get_parameter("identification_margins").value),
            minimum_amplitudes=tuple(self.get_parameter(
                "identification_minimum_amplitudes").value),
            settle_s=float(self.get_parameter("identification_settle_s").value),
            dwell_s=float(self.get_parameter("identification_dwell_s").value),
            hold_s=float(self.get_parameter("identification_hold_s").value),
            slow_fraction=float(self.get_parameter(
                "identification_slow_fraction").value),
            medium_fraction=float(self.get_parameter(
                "identification_medium_fraction").value),
            seed=1 if seed < 0 else seed,
            max_duration_s=float(self.get_parameter(
                "identification_max_duration_s").value),
        )
        self._gen = IdentificationGenerator(
            start_position=np.asarray(self._start_pos, dtype=float)[idx],
            lower_limits=lower,
            upper_limits=upper,
            minimum_speeds=self._min_speeds[idx],
            maximum_speeds=self._max_speeds[idx],
            dt=self._h,
            config=config,
        )
        self._duration = self._gen.duration
        self._episode_index = -1
        self.get_logger().info(
            f"identification generator: duration={self._duration:.3f}s "
            f"requested={list(config.amplitudes)} "
            f"resolved={self._gen.amplitudes.tolist()} "
            f"episodes={len(self._gen.episodes)} seed={config.seed}")

    def _update_episode_markers(self, t: float, finishing: bool = False) -> None:
        """Publish every crossed episode boundary exactly once."""
        if self._mode != "identification" or self._gen is None:
            return
        target_index = len(self._gen.episodes) if finishing else (
            self._gen.episode_index(t) + 1)
        while self._episode_index + 1 < target_index:
            if self._episode_index >= 0:
                previous = self._gen.episodes[self._episode_index]
                self._marker(
                    "episode_end", index=self._episode_index,
                    name=previous.name,
                    planned_end_s=previous.start_s + previous.duration_s)
            self._episode_index += 1
            if self._episode_index < len(self._gen.episodes):
                current = self._gen.episodes[self._episode_index]
                self._marker(
                    "episode_start", index=self._episode_index,
                    name=current.name, planned_start_s=current.start_s,
                    planned_duration_s=current.duration_s,
                    command_speed_limits=(
                        current.maximum_command_speed_limits.tolist()))
        if finishing and self._episode_index == len(self._gen.episodes) - 1:
            previous = self._gen.episodes[self._episode_index]
            self._marker(
                "episode_end", index=self._episode_index, name=previous.name,
                planned_end_s=previous.start_s + previous.duration_s)
            self._episode_index += 1

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
        elif self._mode == "identification":
            v[self._target_idx] = self._gen.relative_velocity(t)
        else:
            raise ValueError(f"unknown mode '{self._mode}'")
        return v

    def _initialize_floor_tracking(self) -> None:
        """Reset tracking and rebase the generator at the measured start pose."""
        self._floor_tracking_direction[:] = 0
        self._floor_reference_offset[:] = 0.0
        if (self._mode not in ("sinusoidal", "identification") or self._gen is None
                or self._last_pos is None):
            return
        q0 = np.asarray(self._gen.step(0.0), dtype=float)
        measured = np.asarray(self._last_pos, dtype=float)[self._target_idx]
        # Feed-forward previously applied generator displacements relative to
        # the run start. Preserve that behavior to avoid an initial catch-up.
        self._floor_reference_offset[self._target_idx] = measured - q0

    def _trajectory_reference(self, t: float) -> np.ndarray:
        """Return the rebased, position-limited six-joint reference."""
        reference = np.asarray(self._last_pos, dtype=float).copy()
        generated = np.asarray(self._gen.step(t), dtype=float)
        if self._mode == "identification":
            reference[self._target_idx] = (
                np.asarray(self._start_pos, dtype=float)[self._target_idx]
                + generated)
        else:
            reference[self._target_idx] = (
                generated + self._floor_reference_offset[self._target_idx])
        if self._pos_lower6 is not None and self._mode != "identification":
            reference = np.clip(reference, self._pos_lower6, self._pos_upper6)
        return reference

    def _trajectory_velocity(self, t: float) -> np.ndarray:
        """Generate a bounded command, using floor-aware tracking when needed."""
        feedforward = self._velocity(t)
        if (self._mode not in ("sinusoidal", "identification")
                or not self._floor_tracking_enabled
                or not np.any(self._min_speeds[self._target_idx] > 0.0)):
            bounded = self._apply_velocity_bounds(feedforward)
            return self._apply_identification_speed_ceiling(t, bounded)
        if self._last_pos is None or len(self._last_pos) != 6:
            # Feedback is required to decide whether a minimum-speed pulse is
            # needed. Stop floored joints rather than integrating blindly.
            safe = feedforward.copy()
            safe[self._min_speeds > 0.0] = 0.0
            bounded = self._apply_velocity_bounds(safe)
            return self._apply_identification_speed_ceiling(t, bounded)
        measured = np.asarray(self._last_pos, dtype=float)
        if not np.all(np.isfinite(measured)):
            safe = feedforward.copy()
            safe[self._min_speeds > 0.0] = 0.0
            bounded = self._apply_velocity_bounds(safe)
            return self._apply_identification_speed_ceiling(t, bounded)
        reference = self._trajectory_reference(t)
        tracked = self._floor_aware_velocity(feedforward, reference, measured)
        return self._apply_identification_speed_ceiling(t, tracked)

    def _apply_identification_speed_ceiling(
            self, t: float, velocity) -> np.ndarray:
        """Keep feedback correction inside the active experiment's budget."""
        bounded = np.asarray(velocity, dtype=float).copy()
        if self._mode != "identification" or self._gen is None:
            return bounded
        limits = np.asarray(self._gen.command_speed_limits(t), dtype=float)
        if limits.shape != (3,) or np.any(limits < 0.0):
            raise ValueError("identification command speed limits are invalid")
        for local_index, joint in enumerate(self._target_idx):
            limit = limits[local_index]
            bounded[joint] = float(np.clip(bounded[joint], -limit, limit))
            if limit == 0.0:
                # A dwell is an explicit zero-input experiment. Do not carry
                # the floor tracker's Schmitt-trigger state into the next move.
                self._floor_tracking_direction[joint] = 0
        return bounded

    def _floor_aware_velocity(
            self, feedforward, reference, measured) -> np.ndarray:
        """Realize a smooth position reference using zero or reliable speed.

        Normal feed-forward plus proportional position correction is used in
        the reliable band. Inside the forbidden band, a Schmitt trigger emits
        either zero or exactly ``vel_min``. This prevents a small continuous
        command from accumulating into large unintended travel.
        """
        velocity = np.clip(
            np.asarray(feedforward, dtype=float),
            -self._max_speeds, self._max_speeds)
        reference = np.asarray(reference, dtype=float)
        measured = np.asarray(measured, dtype=float)
        for joint in self._target_idx:
            vmin = self._min_speeds[joint]
            if vmin <= 0.0:
                continue
            error = reference[joint] - measured[joint]
            if joint in ROT_JOINTS:
                error = (error + 180.0) % 360.0 - 180.0
            candidate = float(np.clip(
                velocity[joint] + self._floor_tracking_kp * error,
                -self._max_speeds[joint], self._max_speeds[joint]))

            if abs(candidate) >= vmin:
                velocity[joint] = candidate
                self._floor_tracking_direction[joint] = (
                    1 if candidate > 0.0 else -1)
                continue

            enter_error = vmin * self._floor_tracking_enter_time_s
            exit_error = vmin * self._floor_tracking_exit_time_s
            direction = int(self._floor_tracking_direction[joint])
            if direction and direction * error <= exit_error:
                # Require at least one explicit stop command before a possible
                # reversal; never jump directly from +vmin to -vmin.
                self._floor_tracking_direction[joint] = 0
                velocity[joint] = 0.0
                continue
            if direction == 0 and abs(error) >= enter_error:
                direction = 1 if error > 0.0 else -1
            self._floor_tracking_direction[joint] = direction
            velocity[joint] = direction * vmin
        return velocity

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

    def _validate_floor_tracking_parameters(self) -> None:
        """Validate floor-tracker gain and hysteresis timing."""
        if (not np.isfinite(self._floor_tracking_kp)
                or self._floor_tracking_kp < 0.0):
            raise ValueError("floor_tracking_kp must be finite and non-negative")
        enter = self._floor_tracking_enter_time_s
        exit_ = self._floor_tracking_exit_time_s
        if (not np.isfinite(enter) or not np.isfinite(exit_)
                or enter <= 0.0 or exit_ < 0.0 or exit_ >= enter):
            raise ValueError(
                "floor tracking requires 0 <= exit_time_s < enter_time_s")

    def _validate_position_return_parameters(self) -> None:
        '''Validate the encoder-qualified one-shot position return settings.'''
        if self._return_control_mode not in ('velocity', 'position'):
            raise ValueError(
                'return_control_mode must be velocity or position')
        if (not np.isfinite(self._return_position_speed_factor)
                or not 0.0 < self._return_position_speed_factor <= 1.0):
            raise ValueError(
                'return_position_speed_factor must be in (0, 1]')
        if (self._return_tolerances.shape != (6,)
                or not np.all(np.isfinite(self._return_tolerances))
                or np.any(self._return_tolerances <= 0.0)):
            raise ValueError(
                'return_position_tolerance must contain 6 positive values')
        if (not np.isfinite(self._return_position_settle_s)
                or self._return_position_settle_s <= 0.0):
            raise ValueError('return_position_settle_s must be positive')
        if (not np.isfinite(self._return_position_mode_delay_s)
                or self._return_position_mode_delay_s < 0.0):
            raise ValueError(
                'return_position_mode_delay_s must be non-negative')

    def _validate_preflight_parameters(self) -> None:
        """Validate feedback qualification thresholds."""
        if self._preflight_timeout_s <= 0.0:
            raise ValueError("preflight_feedback_timeout_s must be positive")
        if (self._preflight_stability_s <= 0.0
                or self._preflight_stability_s >= self._preflight_timeout_s):
            raise ValueError(
                "preflight_stability_s must be positive and less than timeout")
        if self._preflight_max_age_s <= 0.0:
            raise ValueError("preflight_max_feedback_age_s must be positive")
        if self._preflight_limit_tolerance < 0.0:
            raise ValueError(
                "preflight_position_limit_tolerance must be non-negative")
        if (self._preflight_position_drift.shape != (6,)
                or not np.all(np.isfinite(self._preflight_position_drift))
                or np.any(self._preflight_position_drift < 0.0)):
            raise ValueError(
                "preflight_position_drift must contain 6 finite non-negative values")
        if (not np.isfinite(self._preflight_encoder_drift)
                or self._preflight_encoder_drift < 0.0):
            raise ValueError(
                "preflight_encoder_drift_counts must be finite and non-negative")

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
        now_ns = self.get_clock().now().nanoseconds
        snap = {
            "predicate": chr(msg.predicate),          # 'P' pos, 'V' vel, 'E' enc
            "data": [float(x) for x in msg.data],
            "stamp_ns": now_ns,
        }
        self._last_state = snap
        if msg.predicate == DeviceStream.POS:
            self._last_pos = snap["data"]             # feedback for return-to-start
            self._pos_history.append((now_ns, snap["data"]))
            self._prune_preflight_history(now_ns)
        elif msg.predicate == DeviceStream.ENC:
            self._enc_seen = True
            self._last_enc = snap                     # raw encoder counts
            self._enc_history.append((now_ns, snap["data"]))
            self._prune_preflight_history(now_ns)

    def _prune_preflight_history(self, now_ns: int) -> None:
        """Retain enough recent feedback for qualification without growing."""
        keep_ns = int(max(
            self._preflight_timeout_s + 0.5,
            self._preflight_stability_s + 0.5) * 1e9)
        cutoff = now_ns - keep_ns
        while self._pos_history and self._pos_history[0][0] < cutoff:
            self._pos_history.pop(0)
        while self._enc_history and self._enc_history[0][0] < cutoff:
            self._enc_history.pop(0)

    def _device_event_cb(self, msg: DeviceEvent) -> None:
        if (msg.predicate == ManagerEvent.POSITION_STATUS
                and len(msg.data) >= 3):
            self._position_status = int(msg.data[1])
            self._position_complete_seen = (
                self._position_status == ManagerEvent.POSITION_COMPLETE)
            if self._position_status != ManagerEvent.POSITION_COMPLETE:
                self.get_logger().error(
                    f'firmware position event: {msg.text} data={list(msg.data)}')
            return
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
        if (fault["fault_type"] == ManagerEvent.FAULT_DRIVER_COMMUNICATION
                and fault["detail"]):
            fault["driver_stage"] = chr(fault["detail"])
        if fault["protocol_version"] >= 3 and len(data) >= 15:
            failure_names = {
                0: "none",
                1: "timeout",
                2: "partial_frame",
                3: "response_overflow",
                4: "explicit_rejection",
                5: "malformed_response",
            }
            failure = int(data[12])
            response_length = min(int(data[14]), 8, max(0, len(data) - 15))
            response = bytes(
                int(value) & 0xff
                for value in data[15:15 + response_length])
            fault.update({
                "driver_ack_failure": failure_names.get(
                    failure, f"unknown_{failure}"),
                "driver_ack_attempts": int(data[13]),
                "driver_response_hex": response.hex(" "),
                "driver_response_ascii": "".join(
                    chr(value) if 32 <= value < 127 else "."
                    for value in response),
            })
        if fault["transition"] != ManagerEvent.MOTION_CONFIRMED:
            return
        if self._t0 is None and not self._done:
            self._hardware_fault = fault
            self._abort_preflight(
                "confirmed device fault during feedback preflight", fault)
            return
        if self._done:
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
        return {
            "last": self._last_state,
            "pos": None if self._last_pos is None else [
                float(value) for value in self._last_pos],
            "enc": self._last_enc,
        }

    # -- publishing helpers ---------------------------------------------- #
    def _publish_velocity(self, vel: np.ndarray) -> None:
        msg = ControlStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._source
        msg.joint_vel = [float(x) for x in vel]
        self._control_pub.publish(msg)

    def _publish_position(self, target, motor_speeds) -> None:
        '''Publish one atomic absolute target plus physical-axis speed limits.'''
        msg = ControlStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._source
        msg.joint_pos = [float(value) for value in target]
        msg.joint_vel = [float(value) for value in motor_speeds]
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
