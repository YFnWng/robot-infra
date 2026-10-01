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
import math

from control_interface.msg import (
    CausalExperimentTrace, ControlStream, DeviceEvent, DeviceStream,
    ManagerEvent)
from control_interface.srv import DeviceCmd
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .identification import IdentificationConfig, IdentificationGenerator
from .causal_experiment import (
    CausalExperimentConfig, CausalExperimentGenerator,
    resolve_tolerance_qualified_start)

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
        self.declare_parameter("mode", "constant")
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
            "identification_margins", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "identification_minimum_amplitudes", [2.0, 20.0, 1.0])
        self.declare_parameter("identification_full_limits", True)
        self.declare_parameter("identification_settle_s", 2.0)
        self.declare_parameter("identification_dwell_s", 1.0)
        self.declare_parameter("identification_hold_s", 2.0)
        self.declare_parameter("identification_slow_fraction", 0.30)
        self.declare_parameter("identification_medium_fraction", 0.70)
        self.declare_parameter("identification_max_duration_s", 600.0)
        # Causal proximal experiment. These values are intentionally modest
        # and remain subject to the selected catheter's hard limits.
        self.declare_parameter("causal_schedule", "full")
        self.declare_parameter("causal_amplitudes", [6.0, 75.0, 5.5])
        self.declare_parameter(
            "causal_minimum_amplitudes", [5.0, 65.0, 4.75])
        self.declare_parameter("causal_margins", [15.0, 20.0, 1.0])
        self.declare_parameter("causal_slow_speeds", [2.0, 7.0, 2.0])
        self.declare_parameter("causal_fast_speeds", [5.0, 20.0, 4.0])
        self.declare_parameter("causal_repeats", 3)
        self.declare_parameter("causal_static_s", 15.0)
        self.declare_parameter("causal_endpoint_dwell_s", 2.0)
        self.declare_parameter("causal_between_episode_s", 1.0)
        self.declare_parameter("causal_rotation_relax_s", 4.0)
        self.declare_parameter("causal_timing_leads_ms", [20.0, 40.0, 80.0])
        self.declare_parameter("causal_timing_direction", 1)
        self.declare_parameter("causal_bend_bias_position", 7.5)
        self.declare_parameter("causal_insertion_center_position", 20.0)
        self.declare_parameter(
            "causal_insertion_plateaus",
            [0.0, 40.0 / 3.0, 80.0 / 3.0, 40.0])
        self.declare_parameter("causal_insertion_plateau_visits", 2)
        self.declare_parameter("causal_insertion_plateau_dwell_s", 3.0)
        self.declare_parameter("causal_tendon_sweep_limits", [0.0, 15.0])
        self.declare_parameter("causal_allow_insertion_centering", False)
        self.declare_parameter("causal_max_duration_s", 950.0)
        self.declare_parameter("causal_require_estimator", True)
        # Keep the estimator status contract available for the independent
        # controller-disarmed/adaptation-disabled interlocks, while allowing
        # camera/SVO-first experiments to treat online marker/UKF quality as
        # diagnostic only.  The default remains fail-closed for the existing
        # causal schedules.
        self.declare_parameter("causal_require_estimator_tracking", True)
        self.declare_parameter("causal_estimator_status_topic",
                               "/catheter_mppi/status")
        self.declare_parameter("causal_estimator_max_age_s", 0.5)
        self.declare_parameter("causal_minimum_accepted_observations", 8)
        self.declare_parameter("causal_maximum_marker_rejections", 3)
        self.declare_parameter("causal_require_controller_disarmed", True)
        self.declare_parameter("causal_require_adaptation_disabled", True)
        # Isolation runs use a repeatable interior operating point. This is an
        # ordinary manager-mediated position transaction, never an encoder
        # zero operation. The causal launch enables it only for actuating runs.
        self.declare_parameter("causal_initialize_before_run", False)
        self.declare_parameter("causal_initial_position", [20.0, 0.0, 0.0])
        self.declare_parameter("causal_initialization_timeout_s", 30.0)
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

        self._causal_initialize_before_run = bool(
            self.get_parameter("causal_initialize_before_run").value)
        self._causal_initial_position = np.asarray(
            self.get_parameter("causal_initial_position").value, dtype=float)
        self._causal_initialization_timeout_s = float(
            self.get_parameter("causal_initialization_timeout_s").value)
        self._validate_causal_initialization_parameters()

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
        if self._mode not in (
                "constant", "sinusoidal", "identification", "causal"):
            raise ValueError(f"unknown mode '{self._mode}'")
        if self._mode in ("identification", "causal") and self._target != "catheter":
            raise ValueError(
                f"{self._mode} mode currently supports target=catheter")
        self._causal_contract = None
        if self._mode == "causal":
            # The experiment records the exact projection used by MPPI and
            # mirrored from firmware. Import lazily so ordinary automation
            # collection modes do not depend on the learned-control package.
            from catheter_control.hardware_contract import (
                load_hardware_contract)
            self._causal_contract = load_hardware_contract(
                str(self.get_parameter("limits_file").value),
                str(self.get_parameter("catheter").value))

        self._gen = None
        self._shortest_delta = None
        self._floor_tracking_direction = np.zeros(6, dtype=np.int8)
        self._floor_reference_offset = np.zeros(6)
        self._h = 1.0 / self._rate
        self._episode_index = -1
        self._causal_episode_start_timestamp_ns = 0
        self._causal_episode_start_position = [math.nan] * 6
        self._causal_episode_start_encoder = [math.nan] * 6
        if self._mode == "sinusoidal":
            self._build_generator()

        motion_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._control_pub = self.create_publisher(
            ControlStream, "/teleop/control", motion_qos)
        self._event_pub = self.create_publisher(ManagerEvent, "/teleop/event", 10)
        self._marker_pub = self.create_publisher(
            String, "/collection/events", collection_marker_qos())
        trace_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._causal_trace_pub = self.create_publisher(
            CausalExperimentTrace, "/collection/causal_trace", trace_qos)
        self._device_client = self.create_client(DeviceCmd, "/device/command")
        self._abort_service = self.create_service(
            Trigger, "/collection/abort", self._abort_cb)

        # Latest device state (firmware reports predicate 'P'/'V'/'E' + 6 values),
        # captured so the run_start marker records the initial joint configuration.
        self._last_state = None
        self._last_enc = None          # latest raw-ENC frame (start encoder counts)
        self._last_pos = None          # latest reported POS 6-vector (return feedback)
        self._pos_history = []         # (receipt stamp_ns, length-6 data)
        self._enc_history = []
        self._enc_seen = False
        self._causal_estimator_status = None
        self._causal_estimator_status_receipt_ns = None
        self.create_subscription(DeviceStream, "/device/state", self._state_cb, 10)
        self.create_subscription(
            DeviceEvent, "/device/event", self._device_event_cb, 10)
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("causal_estimator_status_topic").value),
            self._causal_estimator_status_cb, 10)

        self._t0 = None
        self._done = False
        self._returning = False
        self._return_t0 = None
        self._return_status = "not_started"
        self._return_elapsed_s = None
        self._start_pos = None         # pose captured at run_start (return target)
        self._causal_plan_start = None  # exact-limit planning origin
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
        self._initializing = False
        self._initialization_complete = False
        self._initialization_timer = None
        self._initialization_t0 = None
        self._initialization_target_pos = None
        self._initialization_position_speeds = None
        self._initialization_within_since_ns = None
        self._initialization_mode_ready_ns = None
        self.should_exit = False

        # Enable after a short delay so publishers finish discovery.
        self._start_timer = self.create_timer(0.5, self._start)

    # -- lifecycle -------------------------------------------------------- #
    def _abort_cb(self, _request, response):
        """Stop without an automatic return move; safe in every phase."""
        if self._done:
            response.success = False
            response.message = "collection is already finished"
            return response
        if self._t0 is None:
            self._abort_preflight("abort requested by operator")
        else:
            self._run_status = "operator_aborted"
            self._returning = False
            self._return_status = "aborted_by_operator"
            self._marker("operator_abort")
            self._finish()
        response.success = True
        response.message = "stop initiated"
        return response

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
            if (self._mode == "causal"
                    and self._causal_initialize_before_run
                    and not self._initialization_complete):
                self._begin_causal_initialization()
            else:
                self._begin_run()
            return
        self._preflight_last_error = error
        if now_ns >= self._preflight_deadline_ns:
            self._abort_preflight(
                f"feedback qualification timed out: {error}", self._fault_status)

    def _feedback_preflight_error(self, now_ns: int) -> str | None:
        """Return why feedback is not motion-safe yet, otherwise ``None``."""
        if getattr(self, "_mode", None) == "causal":
            estimator_error = self._causal_estimator_error(
                now_ns, require_clean=True)
            if estimator_error is not None:
                return estimator_error
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
                tolerance = self._preflight_limit_tolerance
                causal_contract = getattr(self, "_causal_contract", None)
                if causal_contract is not None:
                    tolerance = float(
                        causal_contract.feedback_limit_tolerance[joint])
                if (pos[joint] < lower - tolerance
                        or pos[joint] > upper + tolerance):
                    return (
                        f"{JOINTS[joint]} position {pos[joint]:.6g} outside "
                        f"[{lower:.6g}, {upper:.6g}] with feedback "
                        f"tolerance {tolerance:.6g}")

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
        self._stop_causal_initialization_motion()
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

    def _begin_causal_initialization(self) -> None:
        """Move the selected catheter joints to the reviewed run origin.

        This uses the manager-mediated absolute-position path. Feedback must
        pass a second stationary preflight at the destination before the
        generator is constructed, preventing an isolation run from silently
        inheriting a transient or off-target initialization state.
        """
        if not self._start_motor:
            self._abort_preflight(
                "causal initialization requires start_motor:=true")
            return
        if self._last_pos is None:
            self._abort_preflight(
                "causal initialization has no POS feedback")
            return

        target = np.asarray(self._last_pos, dtype=float).copy()
        target[self._target_idx] = self._causal_initial_position
        if self._pos_lower6 is not None:
            for joint in self._target_idx:
                if not (self._pos_lower6[joint] <= target[joint]
                        <= self._pos_upper6[joint]):
                    self._abort_preflight(
                        f"causal initialization target for {JOINTS[joint]} "
                        f"is outside [{self._pos_lower6[joint]:.6g}, "
                        f"{self._pos_upper6[joint]:.6g}]")
                    return

        self._initializing = True
        self._initialization_t0 = self.get_clock().now()
        self._initialization_target_pos = target
        self._initialization_position_speeds = np.zeros(6)
        speeds = np.maximum(
            self._min_speeds,
            self._return_position_speed_factor * self._max_speeds)
        self._initialization_position_speeds[self._target_idx] = speeds[
            self._target_idx]
        self._initialization_within_since_ns = None
        self._position_status = None
        self._position_complete_seen = False
        self._marker(
            "initialization_start",
            start_position=[float(self._last_pos[j]) for j in self._target_idx],
            target_position=[float(target[j]) for j in self._target_idx],
            tolerance=[float(self._return_tolerances[j])
                       for j in self._target_idx],
            speed_limits=[float(
                self._initialization_position_speeds[j])
                for j in self._target_idx],
            control_mode="position")
        self.get_logger().info(
            "initializing causal experiment joints %s to %s with position "
            "control" % (
                self._target_idx,
                [round(float(target[j]), 3) for j in self._target_idx]))
        self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_POS))
        self._initialization_mode_ready_ns = int(
            self.get_clock().now().nanoseconds
            + self._return_position_mode_delay_s * 1e9)
        self._initialization_timer = self.create_timer(
            1.0 / self._rate, self._causal_initialization_tick)

    def _initialization_position_done(self, now_ns: int) -> bool:
        if self._last_pos is None:
            self._initialization_within_since_ns = None
            return False
        within = all(
            abs(self._initialization_target_pos[joint]
                - self._last_pos[joint]) <= self._return_tolerances[joint]
            for joint in self._target_idx)
        if not within:
            self._initialization_within_since_ns = None
            return False
        if self._initialization_within_since_ns is None:
            self._initialization_within_since_ns = now_ns
            return False
        return (
            now_ns - self._initialization_within_since_ns
            >= int(self._return_position_settle_s * 1e9))

    def _stop_causal_initialization_motion(self) -> None:
        if self._initialization_timer is not None:
            self._initialization_timer.cancel()
            self._initialization_timer = None
        if self._initializing:
            self._send_event(ManagerEvent.STOP_MOTOR)
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.NONE))
        self._initializing = False

    def _causal_initialization_tick(self) -> None:
        if self._done or not self._initializing:
            return
        now = self.get_clock().now()
        elapsed = (now - self._initialization_t0).nanoseconds * 1e-9
        if self._position_status in (
                ManagerEvent.POSITION_TIMED_OUT,
                ManagerEvent.POSITION_REJECTED):
            status = (
                "firmware_timed_out"
                if self._position_status == ManagerEvent.POSITION_TIMED_OUT
                else "firmware_rejected")
            self._marker(
                "initialization_failed", status=status,
                final_position=(None if self._last_pos is None else
                                [float(self._last_pos[j])
                                 for j in self._target_idx]))
            self._abort_preflight(
                f"causal initialization {status}", self._fault_status)
            return
        if self._initialization_position_done(now.nanoseconds):
            final = [float(self._last_pos[j]) for j in self._target_idx]
            error = [
                float(self._initialization_target_pos[j] - self._last_pos[j])
                for j in self._target_idx]
            self._marker(
                "initialization_complete", status="succeeded",
                elapsed_s=float(elapsed), final_position=final,
                error_target_minus_final=error)
            self.get_logger().info(
                f"causal initialization complete: position={final} "
                f"error={error}")
            self._stop_causal_initialization_motion()
            self._initialization_complete = True
            # Re-qualify fresh stationary feedback and estimator health at the
            # initialized configuration before recording run_start.
            self._preflight_deadline_ns = int(
                now.nanoseconds + self._preflight_timeout_s * 1e9)
            self._preflight_timer = self.create_timer(
                0.05, self._preflight_feedback_tick)
            self._preflight_feedback_tick()
            return
        if elapsed > self._causal_initialization_timeout_s:
            final = (None if self._last_pos is None else
                     [float(self._last_pos[j]) for j in self._target_idx])
            self._marker(
                "initialization_failed", status="timed_out",
                elapsed_s=float(elapsed), final_position=final)
            self._abort_preflight(
                "causal initialization timed out", self._fault_status)
            return
        if now.nanoseconds >= self._initialization_mode_ready_ns:
            self._publish_position(
                self._initialization_target_pos,
                self._initialization_position_speeds)

    def _begin_run(self) -> None:
        if self._preflight_timer is not None:
            self._preflight_timer.cancel()
        self._start_pos = list(self._last_pos) if self._last_pos is not None else None
        if self._mode in ("identification", "causal"):
            try:
                if self._mode == "identification":
                    self._build_identification_generator()
                else:
                    self._build_causal_generator()
            except Exception as exc:
                self._abort_preflight(
                    f"{self._mode} trajectory validation failed: {exc}",
                    self._fault_status)
                return
        if self.get_parameter("auto_enable").value:
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_VEL))
            self.get_logger().info("requested JOINT_VEL mode")
        if self._start_motor:
            self._send_event(ManagerEvent.START_MOTOR)
            self.get_logger().warn("START_MOTOR sent — hardware may move")
        generator_metadata = (
            self._gen.metadata
            if self._mode in ("identification", "causal") else None)
        self._marker("run_start", mode=self._mode, target=self._target,
                     seed=int(self.get_parameter("seed").value),
                     floor_tracking={
                         "enabled": self._floor_tracking_enabled,
                         "kp": self._floor_tracking_kp,
                         "enter_time_s": self._floor_tracking_enter_time_s,
                         "exit_time_s": self._floor_tracking_exit_time_s,
                     },
                     start_state=self._state_snapshot(),
                     causal_plan_start=(
                         None if self._causal_plan_start is None else
                         self._causal_plan_start.tolist()),
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
        if self._mode == "causal":
            gate_error = self._causal_estimator_error(
                self.get_clock().now().nanoseconds)
            if gate_error is not None:
                self._run_status = "causal_estimator_abort"
                self._returning = False
                self._return_status = "aborted_estimator_gate"
                self.get_logger().error(
                    f"causal experiment stopped: {gate_error}")
                self._marker("causal_estimator_abort", reason=gate_error)
                self._finish()
                return
            vel[self._target_idx] = self._gen.enforce_basis(
                t, vel[self._target_idx])
            command_error = self._timing_command_error(t, vel)
            if command_error is not None:
                self._run_status = "causal_command_topology_abort"
                self._returning = False
                self._return_status = "aborted_command_topology"
                self.get_logger().error(
                    f"causal experiment stopped: {command_error}")
                self._marker(
                    "causal_command_topology_abort", reason=command_error)
                self._finish()
                return
            self._publish_causal_trace(t, vel)
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
            full_position_limits=bool(self.get_parameter(
                "identification_full_limits").value),
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
            f"full_limits={config.full_position_limits} "
            f"position=[{self._gen.usable_lower.tolist()}, "
            f"{self._gen.usable_upper.tolist()}] "
            f"velocity_tiers=[{self._gen.slow_speeds.tolist()}, "
            f"{self._gen.medium_speeds.tolist()}, "
            f"{self._gen.fast_speeds.tolist()}] "
            f"bend_experiment_amplitude="
            f"{self._gen.bend_experiment_amplitude:.3f} "
            f"requested={list(config.amplitudes)} "
            f"resolved={self._gen.amplitudes.tolist()} "
            f"episodes={len(self._gen.episodes)} seed={config.seed}")

    def _build_causal_generator(self) -> None:
        """Resolve the causal basis experiment from fresh measured POS."""
        if self._start_pos is None:
            raise ValueError("fresh POS feedback is required")
        idx = self._target_idx
        if self._pos_lower6 is not None:
            lower = self._pos_lower6[idx]
            upper = self._pos_upper6[idx]
        else:
            lower = np.asarray(
                self.get_parameter("joint_lower").value, dtype=float)
            upper = np.asarray(
                self.get_parameter("joint_upper").value, dtype=float)
        config = CausalExperimentConfig(
            schedule=str(self.get_parameter("causal_schedule").value),
            amplitudes=tuple(self.get_parameter("causal_amplitudes").value),
            minimum_amplitudes=tuple(self.get_parameter(
                "causal_minimum_amplitudes").value),
            margins=tuple(self.get_parameter("causal_margins").value),
            slow_speeds=tuple(self.get_parameter(
                "causal_slow_speeds").value),
            fast_speeds=tuple(self.get_parameter(
                "causal_fast_speeds").value),
            repeats=int(self.get_parameter("causal_repeats").value),
            static_s=float(self.get_parameter("causal_static_s").value),
            endpoint_dwell_s=float(self.get_parameter(
                "causal_endpoint_dwell_s").value),
            between_episode_s=float(self.get_parameter(
                "causal_between_episode_s").value),
            rotation_relax_s=float(self.get_parameter(
                "causal_rotation_relax_s").value),
            timing_leads_ms=tuple(self.get_parameter(
                "causal_timing_leads_ms").value),
            timing_direction=int(self.get_parameter(
                "causal_timing_direction").value),
            bend_bias_position=float(self.get_parameter(
                "causal_bend_bias_position").value),
            insertion_center_position=float(self.get_parameter(
                "causal_insertion_center_position").value),
            insertion_plateaus=tuple(self.get_parameter(
                "causal_insertion_plateaus").value),
            insertion_plateau_visits=int(self.get_parameter(
                "causal_insertion_plateau_visits").value),
            insertion_plateau_dwell_s=float(self.get_parameter(
                "causal_insertion_plateau_dwell_s").value),
            tendon_sweep_limits=tuple(self.get_parameter(
                "causal_tendon_sweep_limits").value),
            allow_insertion_centering=bool(self.get_parameter(
                "causal_allow_insertion_centering").value),
            max_duration_s=float(self.get_parameter(
                "causal_max_duration_s").value),
        )
        measured_start = np.asarray(self._start_pos, dtype=float)[idx]
        tolerance = self._causal_contract.feedback_limit_tolerance[idx]
        self._causal_plan_start = resolve_tolerance_qualified_start(
            measured_start, lower, upper, tolerance)
        adjustment = self._causal_plan_start - measured_start
        if np.any(adjustment != 0.0):
            self.get_logger().warn(
                "resolved tolerance-qualified measured run start onto exact "
                "command limits: measured=%s plan=%s adjustment=%s" % (
                    measured_start.tolist(),
                    self._causal_plan_start.tolist(), adjustment.tolist()))
        self._gen = CausalExperimentGenerator(
            start_position=self._causal_plan_start,
            lower_limits=lower, upper_limits=upper,
            minimum_speeds=self._min_speeds[idx],
            maximum_speeds=self._max_speeds[idx], dt=self._h,
            config=config)
        self._duration = self._gen.duration
        self._episode_index = -1
        self.get_logger().info(
            "causal experiment generator: schedule=%s duration=%.3fs "
            "half_excursions=%s peak_to_peak=%s center=%s "
            "bend_bias=%.3f speeds=%s/%s episodes=%d" % (
                self._gen.schedule, self._duration,
                self._gen.amplitudes.tolist(),
                (2.0 * self._gen.amplitudes).tolist(),
                self._gen.experiment_center.tolist(),
                config.bend_bias_position,
                self._gen.slow_speeds.tolist(),
                self._gen.fast_speeds.tolist(),
                len(self._gen.episodes)))

    def _update_episode_markers(self, t: float, finishing: bool = False) -> None:
        """Publish every crossed episode boundary exactly once."""
        if self._mode not in ("identification", "causal") or self._gen is None:
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
                self.get_logger().info(
                    "episode %d/%d END %s" % (
                        self._episode_index + 1, len(self._gen.episodes),
                        previous.name))
            self._episode_index += 1
            if self._episode_index < len(self._gen.episodes):
                current = self._gen.episodes[self._episode_index]
                if self._mode == "causal":
                    self._causal_episode_start_timestamp_ns = int(
                        self.get_clock().now().nanoseconds)
                    self._causal_episode_start_position = (
                        [math.nan] * 6 if self._last_pos is None else
                        [float(value) for value in self._last_pos])
                    self._causal_episode_start_encoder = (
                        [math.nan] * 6 if self._last_enc is None else
                        [float(value) for value in self._last_enc["data"]])
                self._marker(
                    "episode_start", index=self._episode_index,
                    name=current.name, planned_start_s=current.start_s,
                    planned_duration_s=current.duration_s,
                    excitation_basis=getattr(
                        current, "excitation_basis", "unspecified"),
                    speed_tier=getattr(current, "speed_tier", "unspecified"),
                    repetition=int(getattr(current, "repetition", 0)),
                    insertion_plateau_mm=getattr(
                        current, "insertion_plateau_mm", None),
                    branch_order=getattr(current, "branch_order", None),
                    command_speed_limits=(
                        current.maximum_command_speed_limits.tolist()),
                    **({} if self._mode != "causal" else {
                        "start_timestamp_ns": (
                            self._causal_episode_start_timestamp_ns),
                        "start_logical_position": (
                            self._causal_episode_start_position),
                        "start_raw_encoder_counts": (
                            self._causal_episode_start_encoder),
                    }))
                self.get_logger().info(
                    "episode %d/%d START %s basis=%s speed=%s "
                    "repetition=%d duration=%.3fs" % (
                        self._episode_index + 1, len(self._gen.episodes),
                        current.name,
                        getattr(current, "excitation_basis", "unspecified"),
                        getattr(current, "speed_tier", "unspecified"),
                        int(getattr(current, "repetition", 0)),
                        current.duration_s))
        if finishing and self._episode_index == len(self._gen.episodes) - 1:
            previous = self._gen.episodes[self._episode_index]
            self._marker(
                "episode_end", index=self._episode_index, name=previous.name,
                planned_end_s=previous.start_s + previous.duration_s)
            self.get_logger().info(
                "episode %d/%d END %s" % (
                    self._episode_index + 1, len(self._gen.episodes),
                    previous.name))
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
        elif self._mode in ("identification", "causal"):
            v[self._target_idx] = self._gen.relative_velocity(t)
        else:
            raise ValueError(f"unknown mode '{self._mode}'")
        return v

    def _initialize_floor_tracking(self) -> None:
        """Reset tracking and rebase the generator at the measured start pose."""
        self._floor_tracking_direction[:] = 0
        self._floor_reference_offset[:] = 0.0
        if (self._mode not in ("sinusoidal", "identification", "causal")
                or self._gen is None
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
        if self._mode == "causal":
            reference[self._target_idx] = (
                self._causal_plan_start + generated)
        elif self._mode == "identification":
            reference[self._target_idx] = (
                np.asarray(self._start_pos, dtype=float)[self._target_idx]
                + generated)
        else:
            reference[self._target_idx] = (
                generated + self._floor_reference_offset[self._target_idx])
        if (self._pos_lower6 is not None
                and self._mode not in ("identification", "causal")):
            reference = np.clip(reference, self._pos_lower6, self._pos_upper6)
        return reference

    def _trajectory_velocity(self, t: float) -> np.ndarray:
        """Generate a bounded command, using floor-aware tracking when needed."""
        feedforward = self._velocity(t)
        if (self._mode == "causal" and self._gen is not None
                and self._gen.active_episode(t).is_timing_episode):
            # Phase 3 is generated as constant reliable-speed pulses in the
            # two physical shafts. Applying a floor per *logical* coordinate
            # here would cancel raw shaft 0 whenever logical insertion and
            # bending receive the same floor. Preserve the raw construction;
            # the per-episode ceiling and the manager-equivalent validation in
            # _timing_command_error retain safety authority.
            return self._apply_identification_speed_ceiling(t, feedforward)
        if (self._mode not in ("sinusoidal", "identification", "causal")
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

    def _timing_command_error(self, t: float, velocity) -> str | None:
        """Fail closed if Phase-3 raw-shaft intent would be transformed.

        The manager still performs its normal logical speed and position
        projection.  This pre-publication check mirrors that projection and
        accepts a timing sample only when it leaves both physical shaft
        commands unchanged.  It does not bypass or weaken any manager gate.
        """
        if self._gen is None:
            return None
        expected = self._gen.timing_raw_velocity(t)
        if expected is None:
            return None
        command = np.asarray(velocity, dtype=float)
        if command.shape != (6,) or not np.all(np.isfinite(command)):
            return "phase-3 command is not a finite six-axis vector"
        actual = np.asarray(
            self._causal_contract.logical_to_motor_axis_velocity(command),
            dtype=float)[[0, 2]]
        if not np.allclose(actual, expected, rtol=0.0, atol=1e-9):
            return (
                "phase-3 raw command differs from generator: "
                f"expected={expected.tolist()} actual={actual.tolist()}")
        if self._last_pos is None or len(self._last_pos) != 6:
            return "phase-3 command projection lacks POS feedback"
        projected_logical = self._causal_contract.project_logical_velocity(
            command, self._last_pos)
        projected = np.asarray(
            self._causal_contract.logical_to_motor_axis_velocity(
                projected_logical), dtype=float)[[0, 2]]
        if not np.allclose(projected, expected, rtol=0.0, atol=1e-9):
            return (
                "manager projection would change phase-3 raw command: "
                f"expected={expected.tolist()} projected={projected.tolist()}")
        return None

    def _apply_identification_speed_ceiling(
            self, t: float, velocity) -> np.ndarray:
        """Keep feedback correction inside the active experiment's budget."""
        bounded = np.asarray(velocity, dtype=float).copy()
        if self._mode not in ("identification", "causal") or self._gen is None:
            return bounded
        limits = np.asarray(self._gen.command_speed_limits(t), dtype=float)
        if limits.shape != (3,) or np.any(limits < 0.0):
            raise ValueError("experiment command speed limits are invalid")
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

    def _validate_causal_initialization_parameters(self) -> None:
        """Validate the fixed operating-point initialization contract."""
        if (self._causal_initial_position.shape != (3,)
                or not np.all(np.isfinite(self._causal_initial_position))):
            raise ValueError(
                "causal_initial_position must contain 3 finite values")
        if (not np.isfinite(self._causal_initialization_timeout_s)
                or self._causal_initialization_timeout_s <= 0.0):
            raise ValueError(
                "causal_initialization_timeout_s must be positive")

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
    def _causal_estimator_status_cb(self, message: DiagnosticArray) -> None:
        """Cache the compact controller/UKF health contract for preflight."""
        selected = next((status for status in message.status
                         if status.name == "catheter_control/mppi"), None)
        if selected is None:
            return
        values = {item.key: item.value for item in selected.values}
        try:
            accepted = int(values.get("accepted_observations", "0"))
            rejected = int(values.get("consecutive_rejections", "0"))
        except ValueError:
            return
        self._causal_estimator_status = {
            "controller_state": selected.message,
            "armed": values.get("armed", "False").lower() == "true",
            "estimator_health": values.get(
                "estimator_health", "UNAVAILABLE"),
            "accepted_observations": accepted,
            "consecutive_rejections": rejected,
            "marker_diagnostic": values.get(
                "marker_diagnostic", "UNAVAILABLE"),
            "marker_update_reason": values.get(
                "marker_update_reason", "none"),
            "adaptation_enabled": values.get(
                "model_adaptation_enabled", "False").lower() == "true",
        }
        self._causal_estimator_status_receipt_ns = (
            self.get_clock().now().nanoseconds)

    def _causal_estimator_error(
            self, now_ns: int, require_clean: bool = False) -> str | None:
        if not bool(self.get_parameter("causal_require_estimator").value):
            return None
        if self._causal_estimator_status is None:
            return "missing catheter estimator status"
        receipt = self._causal_estimator_status_receipt_ns
        maximum_age_s = float(
            self.get_parameter("causal_estimator_max_age_s").value)
        age_s = math.inf if receipt is None else (now_ns - receipt) * 1e-9
        if age_s < 0.0 or age_s > maximum_age_s:
            return f"stale catheter estimator status ({age_s:.3f}s)"
        status = self._causal_estimator_status
        if (bool(self.get_parameter(
                "causal_require_controller_disarmed").value)
                and status["armed"]):
            return "catheter controller is armed"
        if (bool(self.get_parameter(
                "causal_require_adaptation_disabled").value)
                and status["adaptation_enabled"]):
            return "online Jacobian adaptation is enabled"
        if not bool(self.get_parameter(
                "causal_require_estimator_tracking").value):
            return None
        if (require_clean and status["estimator_health"] != "TRACKING"):
            return (
                "catheter estimator is not TRACKING: "
                f"{status['estimator_health']}")
        if status["estimator_health"] not in ("TRACKING", "DEGRADED"):
            return (
                "catheter estimator is unavailable: "
                f"{status['estimator_health']}")
        required = int(self.get_parameter(
            "causal_minimum_accepted_observations").value)
        if status["accepted_observations"] < required:
            return (
                "insufficient accepted marker observations: "
                f"{status['accepted_observations']} < {required}")
        maximum_rejections = int(self.get_parameter(
            "causal_maximum_marker_rejections").value)
        if status["consecutive_rejections"] >= maximum_rejections:
            return (
                "repeated marker rejection: "
                f"{status['consecutive_rejections']} >= {maximum_rejections}")
        if require_clean and status["marker_diagnostic"] != "TRACKING":
            return (
                "marker diagnostic is not TRACKING: "
                f"{status['marker_diagnostic']}")
        return None

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

    def _publish_causal_trace(self, t: float, command) -> None:
        """Publish a compact synchronous trace beside the native-rate bag."""
        episode = self._gen.active_episode(t)
        message = CausalExperimentTrace()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._source
        message.episode_index = int(self._gen.episode_index(t))
        message.episode_name = episode.name
        message.excitation_basis = episode.excitation_basis
        message.speed_tier = episode.speed_tier
        message.repetition = int(episode.repetition)
        message.experiment_elapsed_s = float(t)
        message.episode_elapsed_s = float(t - episode.start_s)
        message.intended_command_timestamp_ns = int(
            self._t0.nanoseconds + round(float(t) * 1e9))
        message.episode_start_timestamp_ns = int(
            self._causal_episode_start_timestamp_ns)
        axis_mask = np.zeros(6, dtype=bool)
        axis_mask[self._target_idx] = (
            episode.maximum_command_speed_limits > 0.0)
        message.command_axis_mask = axis_mask.tolist()
        message.episode_start_logical_position = list(
            self._causal_episode_start_position)
        message.episode_start_raw_encoder_counts = list(
            self._causal_episode_start_encoder)
        message.requested_logical_velocity = [
            float(value) for value in self._velocity(t)]
        message.commanded_logical_velocity = [
            float(value) for value in command]
        projection = self._causal_contract.project_velocity(
            command, self._last_pos)
        message.requested_motor_axis_velocity = (
            projection.requested_motor_axis_velocity.tolist())
        message.predicted_motor_rpm = [
            int(value) for value in projection.motor_rpm]
        message.predicted_motor_radians_per_second = (
            projection.motor_radians_per_second.tolist())
        message.predicted_realized_logical_velocity = (
            projection.realized_logical_velocity.tolist())
        message.reference_logical_position = [
            float(value) for value in self._trajectory_reference(t)]
        if self._last_pos is None:
            message.measured_logical_position = [math.nan] * 6
            message.measured_position_receipt_timestamp_ns = 0
        else:
            message.measured_logical_position = [
                float(value) for value in self._last_pos]
            message.measured_position_receipt_timestamp_ns = int(
                self._pos_history[-1][0]) if self._pos_history else 0
        if self._last_enc is None:
            message.raw_encoder_counts = [math.nan] * 6
            message.raw_encoder_receipt_timestamp_ns = 0
        else:
            message.raw_encoder_counts = [
                float(value) for value in self._last_enc["data"]]
            message.raw_encoder_receipt_timestamp_ns = int(
                self._last_enc["stamp_ns"])
        self._causal_trace_pub.publish(message)

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
        if fault["transition"] == ManagerEvent.MOTION_RETRYING:
            self.get_logger().warn(
                "transient device stall; firmware is retrying motion: "
                f"{fault}")
            self._marker("stall_retry", **fault)
            return
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
