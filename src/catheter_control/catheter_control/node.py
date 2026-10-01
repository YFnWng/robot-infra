"""Guarded ROS 2 wrapper for the streaming catheter model and MPPI."""
from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import replace
from functools import wraps
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

from control_interface.msg import (
    ControlCycleTiming, ControlStream, DeviceStream, EstimatorStateTrace,
    ManagerEvent, MppiResponseTrace, TipReferenceHorizon)
from control_interface.srv import GenerateSparseTargets
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Point32, PointStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import torch

from .causal_schedule import CausalMarkerSchedule
from .backlash import (
    BacklashConfig, BacklashFeedforwardCompensator,
    BacklashStateEstimator, TakeupTransactionArbiter)
from .compute_device import (
    compute_device_diagnostics, resolve_compute_device)
from .engaged_gain import EngagedGainConfig
from .hardware_contract import ENCODER_RADIANS_PER_COUNT, load_hardware_contract
from .lifecycle import (
    ControllerState, FreshnessLimits, GateInputs, paired_source_skew_s,
    readiness, recoverable_encoder_processing_lag)
from .mppi import CatheterMppi, MppiConfig
from .path_tracking import ReferenceHorizon
from .reversal_scheduler import (
    ReversalDirectionScheduler, ReversalSchedulerConfig)
from .timing import PeriodicTimerProbe, TimingWindows
from .tracking import TipForecastMonitor, tip_tracking_error_mm


SOURCE_NAME = "catheter_mppi"


def _instrument_timer(name):
    """Measure timer release lateness and complete callback duration."""
    def decorate(callback):
        @wraps(callback)
        def measured(self, *args, **kwargs):
            started = self._steady()
            probe = self._timer_probes.get(name)
            if probe is not None:
                self._timing.record_seconds(
                    f"{name}_timer_lateness", probe.observe(started))
            try:
                return callback(self, *args, **kwargs)
            finally:
                self._timing.record_seconds(
                    f"{name}_callback_duration", self._steady()-started)
        return measured
    return decorate


def _stamp_ns(header) -> int:
    return (int(header.stamp.sec)*1_000_000_000
            + int(header.stamp.nanosec))


def _load_runtime(cr_meta_root: str, cr_common_root: str):
    meta = Path(cr_meta_root).expanduser().resolve()
    common = Path(cr_common_root).expanduser().resolve()
    if not (meta / "deployment" / "v171_streaming_runtime.py").is_file():
        raise ValueError(f"invalid cr_meta_lnn_root: {meta}")
    if not (common / "cr_common" / "__init__.py").is_file():
        raise ValueError(f"invalid cr_common_root: {common}")
    for path in (meta.parent, common):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from cr_meta_lnn.deployment import V171StreamingCatheterRuntime
    return V171StreamingCatheterRuntime


def _channel_map(message: PointCloud) -> dict[str, np.ndarray]:
    return {
        str(channel.name): np.asarray(channel.values, dtype=np.float64)
        for channel in message.channels
    }


def marker_measurement(message: PointCloud, required_frame: str):
    """Validate and reorder one marker point cloud by marker ID."""
    if message.header.frame_id != required_frame:
        raise ValueError("marker_frame_mismatch")
    if len(message.points) != 4:
        raise ValueError("marker_count")
    channels = _channel_map(message)
    required = (
        "marker_id", "confidence", "reprojection_error_px",
        "source_rig_count")
    if any(name not in channels or channels[name].shape != (4,)
           for name in required):
        raise ValueError("marker_quality_channels")
    marker_id = channels["marker_id"]
    if (not np.all(np.isfinite(marker_id))
            or sorted(int(value) for value in marker_id) != [0, 1, 2, 3]
            or not np.allclose(marker_id, np.round(marker_id))):
        raise ValueError("marker_ids")
    order = np.argsort(marker_id.astype(int))
    points = np.asarray(
        [[point.x, point.y, point.z] for point in message.points],
        dtype=np.float64)[order]
    quality = {
        name: channels[name][order]
        for name in required if name != "marker_id"
    }
    if (not np.all(np.isfinite(points))
            or any(not np.all(np.isfinite(value))
                   for value in quality.values())):
        raise ValueError("nonfinite_marker_measurement")
    timestamp_ns = _stamp_ns(message.header)
    if timestamp_ns <= 0:
        raise ValueError("invalid_marker_timestamp")
    return timestamp_ns, points, quality


class CatheterControlNode(Node):
    """Guarded closed-loop controller; hardware safety stays upstream."""

    def __init__(self):
        super().__init__("catheter_mppi")
        self._declare_parameters()
        self.source = str(self.get_parameter("source_name").value)
        if self.source != SOURCE_NAME:
            raise ValueError(f"source_name must be {SOURCE_NAME!r}")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.command_output_enabled = bool(
            self.get_parameter("command_output_enabled").value)
        self.marker_estimator = str(
            self.get_parameter("marker_estimator").value).strip().lower()
        if self.marker_estimator not in {"gauss_newton", "ekf", "ukf"}:
            raise ValueError(
                "marker_estimator must be one of gauss_newton, ekf, or ukf")
        self.estimator_filter_initial_covariance = float(
            self.get_parameter(
                "estimator_filter_initial_covariance").value)
        self.estimator_filter_process_std_sqrt_s = float(
            self.get_parameter(
                "estimator_filter_process_std_sqrt_s").value)
        self.estimator_initial_roll_hypotheses = int(
            self.get_parameter(
                "estimator_initial_roll_hypotheses").value)
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")

        self._steady = time.monotonic
        self._lock = threading.RLock()
        self._snapshot_lock = threading.Lock()
        self._plan_lock = threading.Lock()
        self._timing = TimingWindows()
        self._timer_probes = {}
        self.state = ControllerState.DISARMED
        self.state_reason = "not_armed"
        self.armed = False
        self.fault_latched = False
        self.last_fault_snapshot = {}
        self.mode_claimed = False
        self.mode_requested_at = None
        self.collection_present = False
        self.manager_ready = False
        self.manager_time = None
        self.position = None
        self.position_valid = False
        self.position_time = None
        self.encoder_time = None
        # Raw receive freshness advances only for a strictly newer hardware
        # stamp. It may authorize a bounded zero pause, never motion.
        self.encoder_receive_time = None
        self.position_source_timestamp_ns = None
        self.encoder_source_timestamp_ns = None
        self.encoder_valid = False
        self.pending_encoder_counts = None
        self.pending_encoder_timestamp_ns = None
        self.pending_encoder_arrival = None
        self._raw_encoder_history = deque(maxlen=256)
        # Timestamped transmission checkpoints are owned by the same 50 Hz
        # estimator callback as the learned runtime. Delayed marker updates
        # restore and replay this belief rather than correcting current time.
        self._backlash_history = deque(maxlen=256)
        self.latest_raw_encoder_counts = None
        self.pending_marker_timestamp_ns = None
        self.pending_marker_points = None
        self.pending_marker_quality = None
        self.pending_marker_arrival = None
        # Source-time fence used only by the simulation reset service.  It
        # prevents device/marker callbacks that were already queued when a
        # reset began from repopulating the freshly cleared estimator inputs.
        self._simulation_reset_source_floor_ns = 0
        self.marker_time = None
        self.marker_diagnostic_time = None
        self.marker_diagnostic_error = True
        self.marker_diagnostic_message = "unavailable"
        self.target = None
        self.reference_source = "NONE"
        self.path_reference = None
        self.observed_tip = None
        self.observed_tip_time = None
        self.observed_tip_timestamp_ns = None
        self.tip_forecast_monitor = TipForecastMonitor(max_pending=32)
        self.last_tip_forecast_result = None
        self._last_tip_error_log_time = None
        self.last_command = np.zeros(6, dtype=np.float64)
        self.last_effective_command = np.zeros(6, dtype=np.float64)
        self.last_command_time = None
        self.last_plan = None
        self.last_marker_result = None
        self.runtime_initialized = False
        self.estimator_health = "UNINITIALIZED"
        self.accepted_observations = 0
        self.consecutive_rejections = 0
        self.planner_warmed = False
        self.consecutive_deadline_misses = 0
        self.estimator_catchup_started_at = None
        self.estimator_catchup_pause_count = 0
        self.model_diagnostics = {}
        self.model_valid = False
        self._planner_snapshot = None
        self._planner_snapshot_source_time = None
        self._planner_transmission_snapshot = None

        self.limits = FreshnessLimits(
            manager_s=float(self.get_parameter("manager_timeout_s").value),
            feedback_s=float(self.get_parameter("feedback_timeout_s").value),
            marker_s=float(self.get_parameter("marker_timeout_s").value),
            marker_diagnostic_s=float(
                self.get_parameter("marker_diagnostic_timeout_s").value),
            feedback_pair_skew_s=float(
                self.get_parameter("feedback_pair_max_skew_s").value))
        self.required_observations = int(
            self.get_parameter("initialization_observations").value)
        self.required_initialization_inliers = int(
            self.get_parameter(
                "initialization_consecutive_inliers").value)
        self.maximum_rejections = int(
            self.get_parameter("maximum_marker_rejections").value)
        self.maximum_deadline_misses = int(
            self.get_parameter("maximum_planner_deadline_misses").value)
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value)
        self.estimator_catchup_timeout_s = float(
            self.get_parameter("estimator_catchup_timeout_s").value)
        tip_error_log_rate = float(
            self.get_parameter("tip_error_log_rate_hz").value)
        self.tip_error_log_period_s = (
            None if tip_error_log_rate == 0.0 else 1.0/tip_error_log_rate)
        self.mode_settle_s = float(
            self.get_parameter("mode_settle_s").value)
        if (self.required_observations < 1
                or self.required_initialization_inliers < 1
                or self.maximum_rejections < 1
                or self.maximum_deadline_misses < 1
                or self.command_timeout_s <= 0.0
                or self.estimator_catchup_timeout_s <= 0.0
                or tip_error_log_rate < 0.0
                or self.mode_settle_s < 0.0):
            raise ValueError("controller guard parameters are invalid")

        self.torch_intraop_threads = int(
            self.get_parameter("torch_intraop_threads").value)
        self.torch_interop_threads = int(
            self.get_parameter("torch_interop_threads").value)
        if (self.torch_intraop_threads < 1
                or self.torch_interop_threads < 1):
            raise ValueError("torch thread counts must be positive")
        torch.set_num_threads(self.torch_intraop_threads)
        torch.set_num_interop_threads(self.torch_interop_threads)
        self.compute_device = resolve_compute_device(
            str(self.get_parameter("device").value))

        runtime_type = _load_runtime(
            str(self.get_parameter("cr_meta_lnn_root").value),
            str(self.get_parameter("cr_common_root").value))
        self.runtime = runtime_type(
            str(self.get_parameter("v171_distal_checkpoint").value),
            str(self.get_parameter("jacobian_initialization_json").value),
            interface_transmission_checkpoint=str(self.get_parameter(
                "interface_transmission_checkpoint").value),
            distal_tendon_allocation_checkpoint=str(self.get_parameter(
                "distal_tendon_allocation_checkpoint").value),
            device=str(self.compute_device.device),
            estimator_initialization_observations=self.required_observations,
            estimator_initialization_consecutive_inliers=(
                self.required_initialization_inliers),
            estimator_maximum_lag_s=float(
                self.get_parameter("maximum_marker_lag_s").value),
            marker_estimator=self.marker_estimator,
            estimator_filter_initial_covariance=(
                self.estimator_filter_initial_covariance),
            estimator_filter_process_std_sqrt_s=(
                self.estimator_filter_process_std_sqrt_s),
            estimator_initial_roll_hypotheses=(
                self.estimator_initial_roll_hypotheses),
            estimator_history_reconciliation_enabled=bool(
                self.get_parameter(
                    "estimator_history_reconciliation_enabled").value),
            estimator_history_reconciliation_maximum_shift=float(
                self.get_parameter(
                    "estimator_history_reconciliation_maximum_shift").value),
            adaptation_enabled=bool(
                self.get_parameter("adaptation_enabled").value),
            adaptation_minimum_observations=int(self.get_parameter(
                "adaptation_minimum_observations").value),
            adaptation_minimum_normalized_action=float(self.get_parameter(
                "adaptation_minimum_normalized_action").value),
            adaptation_minimum_rotation_deg=float(self.get_parameter(
                "adaptation_minimum_rotation_deg").value),
            adaptation_minimum_translation_mm=float(self.get_parameter(
                "adaptation_minimum_translation_mm").value),
            adaptation_minimum_response_snr=float(self.get_parameter(
                "adaptation_minimum_response_snr").value),
            adaptation_maximum_window_s=float(self.get_parameter(
                "adaptation_maximum_window_s").value),
            adaptation_directional_purity=float(self.get_parameter(
                "adaptation_directional_purity").value),
            adaptation_reversal_holdoff_normalized_action=float(
                self.get_parameter(
                    "adaptation_reversal_holdoff_normalized_action").value),
            adaptation_reversal_holdoff_normalized_action_by_axis=[
                float(self.get_parameter(
                    "adaptation_reversal_holdoff_normalized_action_shaft_0"
                ).value),
                float(self.get_parameter(
                    "adaptation_reversal_holdoff_normalized_action_shaft_1"
                ).value),
                float(self.get_parameter(
                    "adaptation_reversal_holdoff_normalized_action_shaft_2"
                ).value),
            ],
            adaptation_confirmation_windows=int(self.get_parameter(
                "adaptation_confirmation_windows").value),
            adaptation_consistency_cosine=float(self.get_parameter(
                "adaptation_consistency_cosine").value),
            adaptation_minimum_column_gain=float(self.get_parameter(
                "adaptation_minimum_column_gain").value),
            adaptation_maximum_column_gain=float(self.get_parameter(
                "adaptation_maximum_column_gain").value),
            adaptation_maximum_direction_deviation_deg=float(
                self.get_parameter(
                    "adaptation_maximum_direction_deviation_deg").value))
        self.model_diagnostics = self.runtime.diagnostics()
        self.model_valid = True
        self.contract = load_hardware_contract(
            str(self.get_parameter("limits_file").value),
            str(self.get_parameter("catheter").value))
        controller_velocity_max = np.asarray(
            self.get_parameter("controller_velocity_max").value,
            dtype=np.float64)
        if (controller_velocity_max.shape != (6,)
                or not np.all(np.isfinite(controller_velocity_max))):
            raise ValueError(
                "controller_velocity_max must contain six finite values")
        inherited = controller_velocity_max < 0.0
        resolved_velocity_max = np.where(
            inherited, self.contract.velocity_max, controller_velocity_max)
        if np.any(resolved_velocity_max > self.contract.velocity_max):
            raise ValueError(
                "controller_velocity_max cannot exceed hardware limits")
        resolved_velocity_min = np.minimum(
            self.contract.velocity_min, resolved_velocity_max)
        self.contract = replace(
            self.contract,
            velocity_min=resolved_velocity_min,
            velocity_max=resolved_velocity_max)
        if self.contract.model_encoder_count_lower is None:
            raise ValueError(
                "selected catheter profile lacks v171 model encoder bounds")
        self.backlash_compensation_enabled = bool(self.get_parameter(
            "backlash_compensation_enabled").value)
        symmetric_backlash_width = tuple(
            float(value) for value in self.get_parameter(
                "backlash_width_rad").value)
        positive_backlash_width = tuple(
            float(value) for value in self.get_parameter(
                "backlash_width_positive_rad").value)
        negative_backlash_width = tuple(
            float(value) for value in self.get_parameter(
                "backlash_width_negative_rad").value)
        directional_widths_present = any(
            value > 0.0 for value in (
                *positive_backlash_width, *negative_backlash_width))
        interface_artifacts = getattr(
            self.runtime, "interface_transmission_artifacts", None)
        # MPPI plans exclusively in confirmed-engaged coordinates.  Raw shaft
        # travel during a take-up transaction is owned by the compensator and
        # is never exposed as an immediately usable model response, including
        # for the tendon shaft.  The frozen distal model may still evolve while
        # that shaft moves, but observed distal bending is used to terminate
        # the transaction before a fresh MPPI plan is accepted.
        self.raw_response_during_interface_takeup = np.zeros(3, dtype=bool)
        self.takeup_response_free_mask = np.ones(3, dtype=bool)
        if interface_artifacts is not None:
            if not self.backlash_compensation_enabled:
                raise ValueError(
                    "v175 interface transmission requires backlash "
                    "compensation")
            fitted_width = tuple(float(value) for value in (
                interface_artifacts.symmetric_reversal_width_rad.detach()
                .cpu().double().tolist()))
            symmetric_backlash_width = fitted_width
            positive_backlash_width = (0.0, 0.0, 0.0)
            negative_backlash_width = (0.0, 0.0, 0.0)
            directional_widths_present = False
            self.get_logger().info(
                "v175 interface transmission loaded: symmetric shaft "
                f"take-up={list(round(value, 6) for value in fitted_width)} "
                "rad; using refit shaft-equivalent Jacobian")
        backlash_config = BacklashConfig(
            width_rad=symmetric_backlash_width,
            width_positive_rad=(
                positive_backlash_width if directional_widths_present
                else None),
            width_negative_rad=(
                negative_backlash_width if directional_widths_present
                else None),
            takeup_velocity=tuple(float(value) for value in self.get_parameter(
                "backlash_takeup_velocity").value),
            minimum_motor_increment_rad=float(self.get_parameter(
                "backlash_minimum_motor_increment_rad").value),
            minimum_transmitted_increment_rad=float(self.get_parameter(
                "backlash_minimum_transmitted_increment_rad").value),
            directional_purity=float(self.get_parameter(
                "backlash_directional_purity").value),
            response_direction_cosine=float(self.get_parameter(
                "backlash_response_direction_cosine").value),
            minimum_response_evidence=float(self.get_parameter(
                "backlash_minimum_response_evidence").value),
            minimum_distal_bending_increment=float(self.get_parameter(
                "backlash_minimum_distal_bending_increment").value),
            distal_confirmation_enabled=True,
            width_learning_rate=float(self.get_parameter(
                "backlash_width_learning_rate").value),
            engagement_confirmation_observations=int(self.get_parameter(
                "backlash_engagement_confirmation_observations").value),
            provisional_rejection_observations=int(self.get_parameter(
                "backlash_provisional_rejection_observations").value))
        engaged_gain_config = EngagedGainConfig(
            enabled=bool(self.get_parameter(
                "engaged_gain_enabled").value),
            tendon_axis=backlash_config.tendon_axis,
            minimum_gain=float(self.get_parameter(
                "engaged_gain_minimum").value),
            maximum_gain=float(self.get_parameter(
                "engaged_gain_maximum").value),
            prior_mean=tuple(float(value) for value in self.get_parameter(
                "engaged_gain_prior_mean").value),
            prior_log_std=float(self.get_parameter(
                "engaged_gain_prior_log_std").value),
            reversal_log_std=float(self.get_parameter(
                "engaged_gain_reversal_log_std").value),
            process_log_std_sqrt_s=float(self.get_parameter(
                "engaged_gain_process_log_std_sqrt_s").value),
            observation_std=float(self.get_parameter(
                "engaged_gain_observation_std").value),
            minimum_nominal_increment=float(self.get_parameter(
                "engaged_gain_minimum_nominal_increment").value),
            huber_sigma=float(self.get_parameter(
                "engaged_gain_huber_sigma").value),
            maximum_normalized_innovation=float(self.get_parameter(
                "engaged_gain_maximum_normalized_innovation").value),
            contradiction_log_std=float(self.get_parameter(
                "engaged_gain_contradiction_log_std").value),
            confidence_log_width=float(self.get_parameter(
                "engaged_gain_confidence_log_width").value),
            minimum_updates=int(self.get_parameter(
                "engaged_gain_minimum_updates").value),
            credible_sigma=float(self.get_parameter(
                "engaged_gain_credible_sigma").value))
        self.backlash_config = backlash_config
        self.engaged_gain_config = engaged_gain_config
        self.backlash_estimator = BacklashStateEstimator(
            backlash_config, engaged_gain_config)
        self.backlash_compensator = BacklashFeedforwardCompensator(
            backlash_config)
        self.takeup_arbiter = TakeupTransactionArbiter(
            self.backlash_compensator,
            response_free_mask=self.takeup_response_free_mask,
            confirmation_hold_timeout_s=float(self.get_parameter(
                "takeup_confirmation_hold_timeout_s").value))
        # A take-up direction that reaches the final projected position/rate
        # boundary before measured response is temporarily unavailable to
        # MPPI. The block is physical-shaft directional state, not a mutation
        # of the configured hard limits, and is released automatically after
        # opposite motion restores a realizable take-up command.
        self.blocked_motor_direction = np.zeros(3, dtype=np.int8)
        self.takeup_saturation_position = None
        self.takeup_saturation_position_timestamp_ns = None
        self.takeup_saturation_release_reason = "none"
        self.backlash_takeup_motor_rates = (
            self.backlash_compensator.motor_radians_per_second(
                self.contract))
        self.backlash_snapshot = self.backlash_estimator.snapshot()
        rollout_backlash = tuple(
            float(value) for value in self.get_parameter(
                "mppi_reversal_backlash_rad").value)
        if (self.backlash_compensation_enabled
                and any(value > 0.0 for value in rollout_backlash)):
            raise ValueError(
                "backlash compensation and MPPI rollout backlash cannot be "
                "enabled together (that would count the dead zone twice)")
        legacy_transmission_aware_rollout = bool(self.get_parameter(
            "mppi_transmission_aware_rollout").value)
        takeup_transaction_requested = bool(self.get_parameter(
            "takeup_transaction_enabled").value)
        if ((legacy_transmission_aware_rollout
             or takeup_transaction_requested)
                and not self.backlash_compensation_enabled):
            raise ValueError(
                "take-up transaction requires backlash compensation")
        # Accept the legacy switch for old launch files, but keep the physical
        # transaction explicit. MPPI itself always plans post-take-up.
        self.takeup_transaction_enabled = bool(
            (takeup_transaction_requested
             or legacy_transmission_aware_rollout)
            and self.backlash_compensation_enabled)
        self.reversal_scheduler = ReversalDirectionScheduler(
            ReversalSchedulerConfig(
                enabled=(bool(self.get_parameter(
                    "reversal_scheduler_enabled").value)
                    and self.takeup_transaction_enabled),
                required_plans=int(self.get_parameter(
                    "reversal_scheduler_required_plans").value),
                minimum_absolute_cost_improvement=float(self.get_parameter(
                    "reversal_scheduler_minimum_absolute_cost_improvement"
                ).value),
                minimum_fractional_cost_improvement=float(self.get_parameter(
                    "reversal_scheduler_minimum_fractional_cost_improvement"
                ).value),
                minimum_terminal_error_improvement_mm=float(
                    self.get_parameter(
                        "reversal_scheduler_minimum_terminal_error_improvement_mm"
                    ).value),
                minimum_accepted_observations=int(self.get_parameter(
                    "reversal_scheduler_minimum_accepted_observations"
                ).value),
                cooldown_s=float(self.get_parameter(
                    "reversal_scheduler_cooldown_s").value)))
        self.planner = CatheterMppi(
            self.runtime, self.contract,
            MppiConfig(
                horizon_steps=int(
                    self.get_parameter("horizon_steps").value),
                step_s=float(self.get_parameter("rollout_step_s").value),
                point_rollout_step_s=float(self.get_parameter(
                    "mppi_point_rollout_step_s").value),
                point_rollout_coarse_steps=bool(self.get_parameter(
                    "mppi_point_rollout_coarse_steps").value),
                path_rollout_coarse_steps=bool(self.get_parameter(
                    "mppi_path_rollout_coarse_steps").value),
                point_prediction_tail_steps=int(self.get_parameter(
                    "mppi_point_prediction_tail_steps").value),
                point_prediction_tail_step_s=float(self.get_parameter(
                    "mppi_point_prediction_tail_step_s").value),
                samples=int(self.get_parameter("samples").value),
                noise_std=tuple(float(value) for value in self.get_parameter(
                    "mppi_noise_std").value),
                noise_correlation=float(self.get_parameter(
                    "mppi_noise_correlation").value),
                exploration_fraction=float(self.get_parameter(
                    "mppi_exploration_fraction").value),
                reversal_backlash_rad=rollout_backlash,
                takeup_risk_weight=float(self.get_parameter(
                    "mppi_takeup_risk_weight").value),
                takeup_confirmation_time_s=float(self.get_parameter(
                    "mppi_takeup_confirmation_time_s").value),
                engaged_gain_scenarios=bool(self.get_parameter(
                    "mppi_engaged_gain_scenarios").value),
                engaged_gain_risk_beta=float(self.get_parameter(
                    "mppi_engaged_gain_risk_beta").value),
                engaged_gain_cvar_alpha=float(self.get_parameter(
                    "mppi_engaged_gain_cvar_alpha").value),
                engaged_gain_maximum_first_step_shift=float(
                    self.get_parameter(
                        "mppi_engaged_gain_maximum_first_step_shift").value),
                engaged_gain_learning_velocity_scale=float(
                    self.get_parameter(
                        "mppi_engaged_gain_learning_velocity_scale").value),
                capture_radius_mm=float(self.get_parameter(
                    "mppi_capture_radius_mm").value),
                capture_minimum_terminal_improvement_mm=float(
                    self.get_parameter(
                        "mppi_capture_minimum_terminal_improvement_mm").value),
                capture_hold_s=float(self.get_parameter(
                    "mppi_capture_hold_s").value),
                capture_response_minimum_prediction_mm=float(
                    self.get_parameter(
                        "mppi_capture_response_minimum_prediction_mm").value),
                capture_response_minimum_ratio=float(self.get_parameter(
                    "mppi_capture_response_minimum_ratio").value),
                raw_response_during_interface_takeup=tuple(
                    bool(value) for value in
                    self.raw_response_during_interface_takeup),
                transmission_aware_rollout=False,
                rotation_direction_latch=False,
                best_candidate_guard=bool(self.get_parameter(
                    "mppi_best_candidate_guard").value),
                grouped_mode_sampling=bool(self.get_parameter(
                    "mppi_grouped_mode_sampling").value),
                takeup_limit_reserve_scale=float(self.get_parameter(
                    "mppi_takeup_limit_reserve_scale").value),
                seed=int(self.get_parameter("mppi_seed").value),
                planning_deadline_s=float(
                    self.get_parameter("planning_deadline_s").value)))
        self.reset_pending = False

        # One callback owns every mutation of the learned runtime. Device and
        # marker subscriptions only replace pending inputs. This makes causal
        # ordering explicit instead of letting two executor groups race for a
        # shared model mutex.
        self.estimator_group = MutuallyExclusiveCallbackGroup()
        self.marker_input_group = MutuallyExclusiveCallbackGroup()
        self.device_group = MutuallyExclusiveCallbackGroup()
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.plan_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_group = MutuallyExclusiveCallbackGroup()
        self._create_ros_interfaces()
        plan_rate = float(self.get_parameter("plan_rate_hz").value)
        command_rate = float(self.get_parameter("command_rate_hz").value)
        diagnostic_rate = float(
            self.get_parameter("diagnostic_rate_hz").value)
        encoder_rate = float(
            self.get_parameter("encoder_update_rate_hz").value)
        marker_rate = float(
            self.get_parameter("marker_update_rate_hz").value)
        if min(plan_rate, command_rate, diagnostic_rate, encoder_rate,
               marker_rate) <= 0.0:
            raise ValueError("timer rates must be positive")
        if self.planner.config.planning_deadline_s >= 1.0/plan_rate:
            raise ValueError(
                "planning deadline must be shorter than the plan period")
        self._marker_schedule = CausalMarkerSchedule(
            marker_rate, self._steady())
        timer_started = self._steady()
        self._timer_probes = {
            "estimator": PeriodicTimerProbe(
                1.0/encoder_rate, timer_started),
            "plan": PeriodicTimerProbe(1.0/plan_rate, timer_started),
            "heartbeat": PeriodicTimerProbe(
                1.0/command_rate, timer_started),
        }
        self.estimator_timer = self.create_timer(
            1.0/encoder_rate, self._estimator_tick,
            callback_group=self.estimator_group)
        self.plan_timer = self.create_timer(
            1.0/plan_rate, self._plan_tick,
            callback_group=self.plan_group)
        self.heartbeat_timer = self.create_timer(
            1.0/command_rate, self._heartbeat_tick,
            callback_group=self.heartbeat_group)
        self.diagnostic_timer = self.create_timer(
            1.0/diagnostic_rate, self._diagnostic_tick,
            callback_group=self.io_group)
        self.get_logger().info(
            "catheter MPPI loaded DISARMED with %s marker estimator on %s "
            "(%s), %d samples x %d steps; controller velocity max=%s; "
            "command output is %s; set target "
            "then call /catheter_mppi/set_armed" % (
                self.marker_estimator,
                self.compute_device.device, self.compute_device.name,
                self.planner.config.samples,
                self.planner.config.horizon_steps,
                np.array2string(
                    self.contract.velocity_max, separator=",", precision=3),
                "ENABLED" if self.command_output_enabled else "DISABLED"))

    def _declare_parameters(self):
        root = os.environ.get(
            "CR_META_LNN_ROOT", "/home/chen-lab/Yifan/cr_meta_lnn")
        common = os.environ.get(
            "CR_COMMON_ROOT", "/home/chen-lab/Yifan/cr-common")
        self.declare_parameter("source_name", SOURCE_NAME)
        self.declare_parameter("command_output_enabled", False)
        self.declare_parameter("simulation_state_reset_enabled", False)
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("cr_meta_lnn_root", root)
        self.declare_parameter("cr_common_root", common)
        self.declare_parameter(
            "v171_distal_checkpoint",
            str(Path(root) / "artifacts" / "deployed"
                / "20260929_175554_grouped_no_rotation"
                / "real_distal_first_order_v171_multistep_map_em.pt"))
        self.declare_parameter(
            "jacobian_initialization_json",
            str(Path(root) / "artifacts" / "deployed"
                / "20260929_175554_grouped_no_rotation"
                / "real_joint_local_distal_v174.json"))
        self.declare_parameter("interface_transmission_checkpoint", "")
        self.declare_parameter("distal_tendon_allocation_checkpoint", "")
        # Stay in prequential shadow mode until causal replay is reviewed.
        self.declare_parameter("adaptation_enabled", False)
        self.declare_parameter("adaptation_minimum_observations", 4)
        self.declare_parameter("adaptation_minimum_normalized_action", 0.01)
        self.declare_parameter("adaptation_minimum_rotation_deg", 0.10)
        self.declare_parameter("adaptation_minimum_translation_mm", 0.30)
        self.declare_parameter("adaptation_minimum_response_snr", 3.0)
        self.declare_parameter("adaptation_maximum_window_s", 1.0)
        self.declare_parameter("adaptation_directional_purity", 0.80)
        self.declare_parameter(
            "adaptation_reversal_holdoff_normalized_action", 0.02)
        self.declare_parameter(
            "adaptation_reversal_holdoff_normalized_action_shaft_0", -1.0)
        self.declare_parameter(
            "adaptation_reversal_holdoff_normalized_action_shaft_1", -1.0)
        self.declare_parameter(
            "adaptation_reversal_holdoff_normalized_action_shaft_2", -1.0)
        self.declare_parameter("adaptation_confirmation_windows", 2)
        self.declare_parameter("adaptation_consistency_cosine", 0.50)
        self.declare_parameter("adaptation_minimum_column_gain", 0.25)
        self.declare_parameter("adaptation_maximum_column_gain", 4.0)
        self.declare_parameter(
            "adaptation_maximum_direction_deviation_deg", 60.0)
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "imricor_test")
        # Negative entries inherit the hardware profile.  Zero disables an
        # axis only inside this controller; manager/firmware limits remain the
        # final independent safety authority.
        self.declare_parameter("controller_velocity_max", [-1.0]*6)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("marker_estimator", "gauss_newton")
        self.declare_parameter("estimator_filter_initial_covariance", 0.25)
        self.declare_parameter("estimator_filter_process_std_sqrt_s", 1.0)
        self.declare_parameter("estimator_initial_roll_hypotheses", 24)
        self.declare_parameter(
            "estimator_history_reconciliation_enabled", True)
        self.declare_parameter(
            "estimator_history_reconciliation_maximum_shift", 120.0)
        self.declare_parameter("horizon_steps", 4)
        self.declare_parameter("rollout_step_s", 0.04)
        self.declare_parameter("mppi_point_rollout_step_s", 0.0)
        self.declare_parameter("mppi_point_rollout_coarse_steps", False)
        self.declare_parameter("mppi_path_rollout_coarse_steps", False)
        self.declare_parameter("mppi_point_prediction_tail_steps", 0)
        self.declare_parameter("mppi_point_prediction_tail_step_s", 0.12)
        self.declare_parameter("samples", 32)
        self.declare_parameter("mppi_noise_std", [4.0, 20.0, 2.0])
        self.declare_parameter("mppi_noise_correlation", 0.65)
        self.declare_parameter("mppi_exploration_fraction", 0.15)
        self.declare_parameter(
            "mppi_reversal_backlash_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("backlash_compensation_enabled", False)
        self.declare_parameter("backlash_width_rad", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "backlash_width_positive_rad", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "backlash_width_negative_rad", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "backlash_takeup_velocity", [8.0, 40.0, 4.5])
        self.declare_parameter(
            "backlash_minimum_motor_increment_rad", 0.01)
        self.declare_parameter(
            "backlash_minimum_transmitted_increment_rad", 0.10)
        self.declare_parameter("backlash_directional_purity", 0.90)
        self.declare_parameter(
            "backlash_response_direction_cosine", 0.50)
        self.declare_parameter(
            "backlash_minimum_response_evidence", 0.50)
        self.declare_parameter(
            "backlash_minimum_distal_bending_increment", 0.05)
        self.declare_parameter("backlash_width_learning_rate", 0.05)
        self.declare_parameter(
            "backlash_engagement_confirmation_observations", 1)
        self.declare_parameter(
            "backlash_provisional_rejection_observations", 2)
        self.declare_parameter("engaged_gain_enabled", False)
        self.declare_parameter("engaged_gain_minimum", 0.10)
        self.declare_parameter("engaged_gain_maximum", 2.0)
        self.declare_parameter("engaged_gain_prior_mean", [1.0, 1.0])
        self.declare_parameter("engaged_gain_prior_log_std", 0.70)
        self.declare_parameter("engaged_gain_reversal_log_std", 0.80)
        self.declare_parameter(
            "engaged_gain_process_log_std_sqrt_s", 0.05)
        self.declare_parameter("engaged_gain_observation_std", 0.08)
        self.declare_parameter(
            "engaged_gain_minimum_nominal_increment", 0.02)
        self.declare_parameter("engaged_gain_huber_sigma", 3.0)
        self.declare_parameter(
            "engaged_gain_maximum_normalized_innovation", 8.0)
        self.declare_parameter(
            "engaged_gain_contradiction_log_std", 0.70)
        self.declare_parameter("engaged_gain_confidence_log_width", 0.50)
        self.declare_parameter("engaged_gain_minimum_updates", 2)
        self.declare_parameter("engaged_gain_credible_sigma", 1.645)
        self.declare_parameter("mppi_engaged_gain_scenarios", False)
        self.declare_parameter("mppi_engaged_gain_risk_beta", 0.50)
        self.declare_parameter("mppi_engaged_gain_cvar_alpha", 0.67)
        self.declare_parameter(
            "mppi_engaged_gain_maximum_first_step_shift", 0.0)
        self.declare_parameter(
            "mppi_engaged_gain_learning_velocity_scale", 1.0)
        self.declare_parameter("mppi_capture_radius_mm", 0.0)
        self.declare_parameter(
            "mppi_capture_minimum_terminal_improvement_mm", 0.0)
        self.declare_parameter("mppi_capture_hold_s", 0.0)
        self.declare_parameter(
            "mppi_capture_response_minimum_prediction_mm", 0.25)
        self.declare_parameter(
            "mppi_capture_response_minimum_ratio", 0.50)
        self.declare_parameter("mppi_transmission_aware_rollout", False)
        self.declare_parameter("takeup_transaction_enabled", False)
        self.declare_parameter(
            "takeup_confirmation_hold_timeout_s", 1.0)
        self.declare_parameter("mppi_rotation_direction_latch", False)
        self.declare_parameter("mppi_best_candidate_guard", True)
        self.declare_parameter("mppi_grouped_mode_sampling", True)
        self.declare_parameter("mppi_takeup_limit_reserve_scale", 1.0)
        self.declare_parameter("mppi_takeup_risk_weight", 4.0)
        self.declare_parameter("mppi_takeup_confirmation_time_s", 0.10)
        self.declare_parameter("reversal_scheduler_enabled", True)
        self.declare_parameter("reversal_scheduler_required_plans", 3)
        self.declare_parameter(
            "reversal_scheduler_minimum_absolute_cost_improvement", 5.0)
        self.declare_parameter(
            "reversal_scheduler_minimum_fractional_cost_improvement", 0.0)
        self.declare_parameter(
            "reversal_scheduler_minimum_terminal_error_improvement_mm", 0.25)
        self.declare_parameter(
            "reversal_scheduler_minimum_accepted_observations", 3)
        self.declare_parameter("reversal_scheduler_cooldown_s", 1.0)
        self.declare_parameter("mppi_seed", 0)
        self.declare_parameter("planning_deadline_s", 0.06)
        self.declare_parameter("plan_rate_hz", 15.0)
        self.declare_parameter("command_rate_hz", 100.0)
        self.declare_parameter("diagnostic_rate_hz", 10.0)
        self.declare_parameter("encoder_update_rate_hz", 50.0)
        self.declare_parameter("marker_update_rate_hz", 20.0)
        self.declare_parameter("torch_intraop_threads", 2)
        self.declare_parameter("torch_interop_threads", 1)
        self.declare_parameter("manager_timeout_s", 0.5)
        self.declare_parameter("feedback_timeout_s", 0.15)
        # If raw ENC remains fresh while estimator correction runs, command
        # zero for this bounded window before retaining encoder_stale.
        self.declare_parameter("estimator_catchup_timeout_s", 0.35)
        self.declare_parameter("marker_timeout_s", 0.15)
        self.declare_parameter("maximum_marker_lag_s", 0.15)
        self.declare_parameter("marker_diagnostic_timeout_s", 0.5)
        self.declare_parameter("feedback_pair_max_skew_s", 0.15)
        self.declare_parameter("command_timeout_s", 0.15)
        self.declare_parameter("path_reference_timeout_s", 0.20)
        self.declare_parameter("tip_error_log_rate_hz", 1.0)
        self.declare_parameter("mode_settle_s", 0.10)
        self.declare_parameter("initialization_observations", 8)
        self.declare_parameter("initialization_consecutive_inliers", 2)
        self.declare_parameter("maximum_marker_rejections", 3)
        self.declare_parameter("maximum_planner_deadline_misses", 3)
        self.declare_parameter(
            "marker_topic", "/shape_tracking/markers")
        self.declare_parameter(
            "marker_diagnostic_topic", "/shape_tracking/marker_status")

    def _create_ros_interfaces(self):
        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        motion_qos = QoSProfile(depth=1)
        motion_qos.reliability = ReliabilityPolicy.RELIABLE
        motion_qos.durability = DurabilityPolicy.VOLATILE
        self.control_pub = self.create_publisher(
            ControlStream, "/teleop/control", motion_qos)
        self.event_pub = self.create_publisher(
            ManagerEvent, "/teleop/event", 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/catheter_mppi/status", 10)
        self.trajectory_pub = self.create_publisher(
            PointCloud, "/catheter_mppi/predicted_tip", 10)
        self.plan_pub = self.create_publisher(
            ControlStream, "/catheter_mppi/planned_control", 10)
        trace_qos = QoSProfile(depth=20)
        trace_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        trace_qos.durability = DurabilityPolicy.VOLATILE
        self.response_trace_pub = self.create_publisher(
            MppiResponseTrace, "/catheter_mppi/response_trace", trace_qos)
        self.estimator_trace_pub = self.create_publisher(
            EstimatorStateTrace, "/catheter_mppi/estimator_trace", trace_qos)
        self.control_cycle_timing_pub = self.create_publisher(
            ControlCycleTiming, "/catheter_mppi/control_cycle_timing",
            trace_qos)
        self.create_subscription(
            DeviceStream, "/device/state", self._device_state_cb,
            qos_profile_sensor_data, callback_group=self.device_group)
        self.create_subscription(
            PointCloud, str(self.get_parameter("marker_topic").value),
            self._marker_cb, qos_profile_sensor_data,
            callback_group=self.marker_input_group)
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("marker_diagnostic_topic").value),
            self._marker_diagnostic_cb, 10, callback_group=self.io_group)
        self.create_subscription(
            ManagerEvent, "/manager/safety_status", self._safety_cb,
            safety_qos, callback_group=self.io_group)
        self.create_subscription(
            ManagerEvent, "/manager/event", self._manager_event_cb, 20,
            callback_group=self.io_group)
        self.create_subscription(
            PointStamped, "/catheter_mppi/target_tip", self._target_cb, 10,
            callback_group=self.io_group)
        reference_qos = QoSProfile(depth=1)
        reference_qos.reliability = ReliabilityPolicy.RELIABLE
        reference_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            TipReferenceHorizon, "/catheter_mppi/reference_horizon",
            self._path_reference_cb, reference_qos,
            callback_group=self.io_group)
        self.create_subscription(
            String, "/collection/events", self._collection_event_cb, 10,
            callback_group=self.io_group)
        self.create_service(
            SetBool, "/catheter_mppi/set_armed", self._arm_cb,
            callback_group=self.service_group)
        self.create_service(
            Trigger, "/catheter_mppi/emergency_stop", self._stop_cb,
            callback_group=self.service_group)
        self.create_service(
            GenerateSparseTargets,
            "/catheter_mppi/generate_sparse_targets",
            self._generate_sparse_targets_cb,
            callback_group=self.service_group)
        if bool(self.get_parameter("simulation_state_reset_enabled").value):
            self.create_service(
                Trigger, "/catheter_mppi/reset_simulation_state",
                self._reset_simulation_state_cb,
                # The learned runtime has exactly one mutation owner.  A
                # simulation reset is another runtime mutation, so serialize
                # it with the estimator timer rather than the general service
                # group.  Otherwise an in-flight correction can commit its
                # pre-reset state after runtime.reset().
                callback_group=self.estimator_group)

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _generate_sparse_targets_cb(self, request, response):
        """Preview disarmed sparse targets from one accepted model snapshot."""
        if not self._plan_lock.acquire(blocking=False):
            response.success = False
            response.message = "planner_busy"
            return response
        try:
            with self._lock:
                now = self._steady()
                if self.armed or self.mode_claimed:
                    raise RuntimeError("controller_must_be_disarmed")
                if self.fault_latched:
                    raise RuntimeError("controller_faulted")
                if not self.manager_ready:
                    raise RuntimeError("manager_not_ready")
                if (not self.position_valid or not self.encoder_valid
                        or not self.model_valid):
                    raise RuntimeError("feedback_or_model_invalid")
                if self.estimator_health != "TRACKING":
                    raise RuntimeError("estimator_not_tracking")
                if (self.observed_tip is None
                        or self.observed_tip_time is None
                        or now-self.observed_tip_time > self.limits.marker_s):
                    raise RuntimeError("observed_tip_stale")
                position = self.position.copy()
                observed_tip = self.observed_tip.copy()
            with self._timed_lock(self._snapshot_lock, "snapshot_preview"):
                root = self._planner_snapshot
                snapshot_time = self._planner_snapshot_source_time
                transmission = self._planner_transmission_snapshot
            if root is None or snapshot_time is None:
                raise RuntimeError("planner_snapshot_unavailable")
            if self._steady()-snapshot_time > self.limits.feedback_s:
                raise RuntimeError("planner_snapshot_stale")
            flat = np.asarray(request.logical_displacements, dtype=np.float64)
            if flat.size == 0 or flat.size % 3:
                raise ValueError(
                    "logical_displacements must contain flattened triplets")
            prediction = self.planner.predict_sparse_targets(
                root, position, observed_tip, flat.reshape(-1, 3),
                int(request.rollout_steps),
                np.asarray(request.minimum_endpoint_reserve,
                           dtype=np.float64),
                transmission_state=transmission)
            displacement_mm = 1000.0*np.linalg.norm(
                prediction.predicted_tip_displacement_m, axis=1)
            minimum = float(request.minimum_tip_displacement_mm)
            maximum = float(request.maximum_tip_displacement_mm)
            if (not math.isfinite(minimum) or not math.isfinite(maximum)
                    or minimum <= 0.0 or maximum <= minimum):
                raise ValueError("invalid tip displacement bounds")
            outside = np.flatnonzero(
                (displacement_mm < minimum) | (displacement_mm > maximum))
            if len(outside):
                raise ValueError(
                    "model target displacement outside requested bounds: "
                    + ",".join(
                        f"{index}={displacement_mm[index]:.3f}mm"
                        for index in outside))
            response.targets = [
                Point(x=float(value[0]), y=float(value[1]),
                      z=float(value[2]))
                for value in prediction.target_tip_m]
            response.realized_logical_displacements = (
                prediction.realized_logical_displacement.reshape(-1).tolist())
            response.endpoint_joint_positions = (
                prediction.endpoint_joint_position.reshape(-1).tolist())
            response.success = True
            response.message = "generated %d guarded sparse targets" % len(
                response.targets)
            self.get_logger().info(
                "%s; tip displacement mm=%s" % (
                    response.message,
                    np.round(displacement_mm, 3).tolist()))
        except Exception as error:
            response.success = False
            response.message = f"{type(error).__name__}:{error}"
        finally:
            self._plan_lock.release()
        return response

    def _message_time_ns(self, message) -> int:
        timestamp = _stamp_ns(message.header)
        return timestamp if timestamp > 0 else self._now_ns()

    def _record_header_age(self, name: str, message):
        timestamp_ns = _stamp_ns(message.header)
        if timestamp_ns > 0:
            self._timing.record_seconds(
                f"{name}_callback_header_age",
                (self._now_ns()-timestamp_ns)/1e9)

    @contextmanager
    def _timed_lock(self, lock, name: str):
        waiting_started = self._steady()
        lock.acquire()
        acquired = self._steady()
        try:
            yield
        finally:
            released = self._steady()
            lock.release()
            self._timing.record_seconds(
                f"{name}_lock_wait", acquired-waiting_started)
            self._timing.record_seconds(
                f"{name}_lock_hold", released-acquired)

    def _device_state_cb(self, message: DeviceStream):
        if len(message.data) != 6 or not all(
                math.isfinite(value) for value in message.data):
            self._fault_if_active("malformed_device_feedback")
            return
        if message.predicate == DeviceStream.POS:
            self._record_header_age("device_pos", message)
        elif message.predicate == DeviceStream.ENC:
            self._record_header_age("device_enc", message)
        now = self._steady()
        if message.predicate == DeviceStream.POS:
            timestamp_ns = self._message_time_ns(message)
            if timestamp_ns <= self._simulation_reset_source_floor_ns:
                return
            position = np.asarray(message.data, dtype=np.float64)
            valid = self.contract.feedback_position_is_valid(position)
            with self._lock:
                self.position = position
                self.position_valid = valid
                self.position_time = now
                self.position_source_timestamp_ns = timestamp_ns
            if not valid:
                self._fault_if_active("position_feedback_out_of_range")
            return
        if message.predicate != DeviceStream.ENC:
            return
        timestamp_ns = self._message_time_ns(message)
        if timestamp_ns <= self._simulation_reset_source_floor_ns:
            return
        counts = np.asarray(message.data, dtype=np.float64)
        valid = self.contract.model_encoder_counts_are_valid(counts)
        with self._lock:
            self.encoder_valid = valid
            if not valid:
                self.pending_encoder_counts = None
                self.pending_encoder_timestamp_ns = None
                self.pending_encoder_arrival = None
                self.encoder_time = now
            else:
                # Pairing describes transport/source coherence, so track the
                # newest received ENC stamp here. The separate encoder_time is
                # advanced only after estimator processing and remains the
                # fail-closed processed-feedback freshness signal.
                previous_source_timestamp_ns = (
                    self.encoder_source_timestamp_ns)
                if (previous_source_timestamp_ns is None
                        or timestamp_ns >= previous_source_timestamp_ns):
                    self.encoder_source_timestamp_ns = timestamp_ns
                if (previous_source_timestamp_ns is None
                        or timestamp_ns > previous_source_timestamp_ns):
                    self.encoder_receive_time = now
        if not valid:
            self._fault_if_active("encoder_feedback_out_of_range")
            return
        with self._lock:
            # The device publishes near 180 Hz. Keep only the newest sample;
            # the learned recurrence is advanced by a bounded-rate timer.
            if (self.pending_encoder_timestamp_ns is None
                    or timestamp_ns >= self.pending_encoder_timestamp_ns):
                self.pending_encoder_counts = counts
                self.pending_encoder_timestamp_ns = timestamp_ns
                self.pending_encoder_arrival = now

    def _take_encoder_sample(self):
        with self._lock:
            if self.pending_encoder_counts is None:
                return None
            if not self.position_valid:
                return None
            sample = (
                self.pending_encoder_timestamp_ns,
                self.pending_encoder_counts,
                self.pending_encoder_arrival)
            self.pending_encoder_counts = None
            self.pending_encoder_timestamp_ns = None
            self.pending_encoder_arrival = None
            return sample

    def _publish_planner_snapshot(
            self, state, source_time, transmission_state=None):
        if state is None or source_time is None:
            return
        with self._timed_lock(self._snapshot_lock, "snapshot_publish"):
            self._planner_snapshot = state
            self._planner_snapshot_source_time = source_time
            self._planner_transmission_snapshot = transmission_state

    def _process_encoder_sample(self, sample):
        if sample is None:
            return False
        timestamp_ns, counts, arrival = sample
        self._timing.record_seconds(
            "encoder_pending_age", self._steady()-arrival)
        started = self._steady()
        backlash_snapshot = None
        try:
            if (self.runtime.state is not None
                    and timestamp_ns <= self.runtime.state.timestamp_ns):
                return False
            raw_counts = np.asarray(counts, dtype=np.float64).copy()
            self._raw_encoder_history.append(
                (int(timestamp_ns), raw_counts[:3].copy()))
            self.latest_raw_encoder_counts = raw_counts[:3].copy()
            effective_motor = None
            if self.backlash_compensation_enabled:
                raw_motor = (
                    raw_counts[:3]*ENCODER_RADIANS_PER_COUNT)
                effective_motor = self.backlash_estimator.advance_motor(
                    raw_motor)
                self._backlash_history.append([
                    int(timestamp_ns), raw_motor.copy(),
                    self.backlash_estimator.clone_state()])
                backlash_snapshot = self.backlash_estimator.snapshot()
            if self.runtime.state is None:
                state = self.runtime.initialize(
                    timestamp_ns, raw_counts,
                    interface_motor_angle_rad=effective_motor)
            else:
                state = self.runtime.advance_encoder(
                    timestamp_ns, raw_counts,
                    interface_motor_angle_rad=effective_motor)
            model_diagnostics = self.runtime.diagnostics()
        except (ValueError, RuntimeError) as error:
            with self._lock:
                self._fault_locked(
                    f"encoder_update:{type(error).__name__}")
            return False
        finally:
            self._timing.record_seconds(
                "encoder_owner_duration", self._steady()-started)
        with self._lock:
            if backlash_snapshot is not None:
                # Commit the transmission snapshot with the runtime state it
                # conditioned, so a planner cannot pair a new gap state with
                # the previous model snapshot.
                self.backlash_snapshot = backlash_snapshot
            self.runtime_initialized = True
            self.estimator_health = state.health
            self.accepted_observations = state.accepted_observations
            self.consecutive_rejections = state.consecutive_rejections
            self.encoder_time = arrival
            self.model_diagnostics = model_diagnostics
            self.model_valid = bool(
                model_diagnostics.get("jacobian_valid", True))
        self._publish_planner_snapshot(
            state, arrival,
            self.backlash_snapshot if self.backlash_compensation_enabled
            else None)
        return True

    def _observe_backlash_at_timestamp(
            self, timestamp_ns, interface_pose, model, strain,
            nominal_distal_lambda=None, observed_distal_lambda=None):
        """Correct the transmission belief at source time and replay.

        This executes only in the sole estimator-owner callback. It mirrors
        the learned runtime's delayed-marker rewind without adding a second
        mutable state owner or blocking the planner snapshot path.
        """
        if not self._backlash_history:
            return self.backlash_snapshot
        entries = list(self._backlash_history)
        index = next((
            i for i in range(len(entries)-1, -1, -1)
            if entries[i][0] <= int(timestamp_ns)), None)
        if index is None:
            return self.backlash_snapshot
        self.backlash_estimator.restore_state(entries[index][2])
        self.backlash_estimator.observe_response(
            entries[index][1], interface_pose, model.jacobian,
            model.state_scale, strain,
            self.runtime.port.mode.detach().cpu().numpy(),
            timestamp_ns=int(timestamp_ns),
            nominal_distal_lambda=nominal_distal_lambda,
            observed_distal_lambda=observed_distal_lambda)
        entries[index][2] = self.backlash_estimator.clone_state()
        for replay_index in range(index+1, len(entries)):
            self.backlash_estimator.advance_motor(entries[replay_index][1])
            entries[replay_index][2] = (
                self.backlash_estimator.clone_state())
        self._backlash_history = deque(entries, maxlen=256)
        return self.backlash_estimator.snapshot()

    def _marker_cb(self, message: PointCloud):
        try:
            timestamp_ns, points, quality = marker_measurement(
                message, self.frame_id)
        except ValueError as error:
            self._fault_if_active(str(error))
            return
        self._record_header_age("marker", message)
        arrival = self._steady()
        with self._lock:
            if timestamp_ns <= self._simulation_reset_source_floor_ns:
                return
            # Camera output is 30 Hz, while Gauss-Newton correction is bounded
            # below that rate. Keep only the newest complete measurement so
            # estimator work cannot build an increasingly stale DDS queue.
            if (self.pending_marker_timestamp_ns is None
                    or timestamp_ns >= self.pending_marker_timestamp_ns):
                self.pending_marker_timestamp_ns = timestamp_ns
                self.pending_marker_points = points
                self.pending_marker_quality = quality
                self.pending_marker_arrival = arrival

    def _take_causal_marker_sample(self, now):
        state_timestamp_ns = (
            None if self.runtime.state is None
            else self.runtime.state.timestamp_ns)
        with self._lock:
            if self.pending_marker_timestamp_ns is None:
                return None
            decision = self._marker_schedule.decision(
                now, self.pending_marker_timestamp_ns,
                state_timestamp_ns)
            if decision != "ready":
                if decision == "awaiting_encoder":
                    self._timing.record_seconds(
                        "marker_causal_deferral",
                        (self.pending_marker_timestamp_ns
                         - state_timestamp_ns)*1e-9)
                return None
            timestamp_ns = self.pending_marker_timestamp_ns
            points = self.pending_marker_points
            quality = self.pending_marker_quality
            arrival = self.pending_marker_arrival
            self.pending_marker_timestamp_ns = None
            self.pending_marker_points = None
            self.pending_marker_quality = None
            self.pending_marker_arrival = None
        self._marker_schedule.commit(now)
        return timestamp_ns, points, quality, arrival

    def _process_marker_sample(self, sample):
        if sample is None:
            return False
        timestamp_ns, points, quality, arrival = sample
        self._timing.record_seconds(
            "marker_pending_age", self._steady()-arrival)
        started = self._steady()
        try:
            if self.runtime.state is None:
                return False
            result = self.runtime.observe_markers(
                timestamp_ns, points, quality)
            state = self.runtime.state
            accepted = state.accepted_observations
            rejected = state.consecutive_rejections
            health = state.health
            model_diagnostics = self.runtime.diagnostics()
            snapshot = self.runtime.clone_state()
            observation_snapshot = (
                self.runtime.clone_state_at_or_before(timestamp_ns)
                if result.accepted else None)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(
                "marker estimator exception (%s): %s" % (
                    type(error).__name__, error))
            with self._lock:
                self._fault_locked(
                    f"marker_update:{type(error).__name__}")
            return False
        finally:
            self._timing.record_seconds(
                "marker_owner_duration", self._steady()-started)
        forecast_result = None
        with self._lock:
            self.runtime_initialized = True
            self.estimator_health = health
            self.accepted_observations = accepted
            self.consecutive_rejections = rejected
            self.last_marker_result = result
            self.model_diagnostics = model_diagnostics
            self.model_valid = bool(
                model_diagnostics.get("jacobian_valid", True))
            if result.accepted:
                # The runtime has already checked the image timestamp against
                # its encoder timestamp. Start control freshness when that
                # validated correction is committed; using DDS arrival here
                # double-counts queue/optimization delay and can expire a
                # correction immediately after it is successfully applied.
                committed = self._steady()
                self.marker_time = committed
                observed_tip = np.asarray(
                    points[-1], dtype=np.float64).copy()
                forecast_result = self.tip_forecast_monitor.observe(
                    timestamp_ns, observed_tip)
                if forecast_result is not None:
                    self.last_tip_forecast_result = forecast_result
                    if self._observe_capture_response_locked(forecast_result):
                        self.last_command.fill(0.0)
                        self.last_effective_command.fill(0.0)
                        self.last_command_time = committed
                        self.state_reason = "capture_response_shortfall_replan"
                self.observed_tip = observed_tip
                self.observed_tip_time = committed
                self.observed_tip_timestamp_ns = timestamp_ns
                if observation_snapshot is not None:
                    raw_encoder_counts = next((
                        values for sample_timestamp, values
                        in reversed(self._raw_encoder_history)
                        if sample_timestamp <= timestamp_ns), None)
                    model = observation_snapshot.adaptive_jacobian
                    if raw_encoder_counts is not None:
                        self.backlash_snapshot = (
                            self._observe_backlash_at_timestamp(
                                timestamp_ns,
                                observation_snapshot.interface_pose.detach()
                                .cpu().numpy(), model,
                                observation_snapshot.strain.detach()
                                .cpu().numpy(),
                                result.distal_lambda_prior,
                                result.distal_lambda_posterior))
                    failed_axes = np.flatnonzero(
                        self.backlash_snapshot.failed)
                    if self.armed and failed_axes.size:
                        failed_axis = int(failed_axes[0])
                        classification = (
                            self.backlash_snapshot
                            .response_classification[failed_axis])
                        fault = (
                            "backlash_response_contradicted"
                            if classification == "CONTRADICTORY" else
                            "backlash_takeup_unconfirmed")
                        self._fault_locked(
                            f"{fault}:axis_{failed_axis}")
            else:
                if self.armed or rejected == 1:
                    self.get_logger().warn(
                        "marker update rejected: %s (%d/%d)" % (
                            result.reason, rejected,
                            self.maximum_rejections))
                if self.armed and rejected >= self.maximum_rejections:
                    self._fault_locked(
                        f"repeated_marker_rejection:{result.reason}")
            source_time = self.encoder_time
        if forecast_result is not None:
            self._publish_response_trace(forecast_result)
        if observation_snapshot is not None:
            if not result.accepted:
                raw_encoder_counts = next((
                    values for sample_timestamp, values
                    in reversed(self._raw_encoder_history)
                    if sample_timestamp <= timestamp_ns), None)
            self._publish_estimator_trace(
                timestamp_ns, observation_snapshot, result,
                model_diagnostics, raw_encoder_counts,
                observed_tip_m=np.asarray(points[-1], dtype=np.float64))
        self._publish_planner_snapshot(
            snapshot, source_time,
            self.backlash_snapshot if self.backlash_compensation_enabled
            else None)
        return True

    @_instrument_timer("estimator")
    def _estimator_tick(self):
        # This callback is the only owner of mutable runtime state. Catch up to
        # the newest encoder before considering a marker, then drain once more
        # after the expensive correction so the published snapshot is current.
        self._process_encoder_sample(self._take_encoder_sample())
        marker = self._take_causal_marker_sample(self._steady())
        self._process_marker_sample(marker)
        self._process_encoder_sample(self._take_encoder_sample())

    def _marker_diagnostic_cb(self, message: DiagnosticArray):
        relevant = [status for status in message.status
                    if status.name == "automation/four_ring_markers"]
        if not relevant:
            return
        status = relevant[0]
        with self._lock:
            self.marker_diagnostic_time = self._steady()
            self.marker_diagnostic_error = (
                status.level == DiagnosticStatus.ERROR)
            self.marker_diagnostic_message = str(status.message)
            if self.marker_diagnostic_error and self.mode_claimed:
                self._fault_locked("marker_diagnostic_error")

    def _safety_cb(self, message: ManagerEvent):
        with self._lock:
            self.manager_ready = message.text == "MANAGER_READY"
            self.manager_time = self._steady()
            if not self.manager_ready and self.mode_claimed:
                self._fault_locked(f"manager:{message.text}")

    def _manager_event_cb(self, message: ManagerEvent):
        if (message.predicate == ManagerEvent.STOP_MOTOR
                or message.text.startswith("COMMAND_REJECTED:")):
            self._fault_if_active(
                message.text or "manager_stop_event")

    def _target_cb(self, message: PointStamped):
        point = np.asarray(
            [message.point.x, message.point.y, message.point.z],
            dtype=np.float64)
        if (message.header.frame_id != self.frame_id
                or not np.all(np.isfinite(point))):
            self._fault_if_active("invalid_target")
            return
        with self._lock:
            if self.armed and self.reference_source == "PATH":
                self._fault_locked("reference_source_conflict")
                return
            self.target = point
            self.reference_source = "POINT"
            self.path_reference = None
            self.reversal_scheduler.reset_intent()
            self.planner.reset_capture_target()
            # A completed response must belong to one target interval. Do not
            # let a forecast made for the previous target label the first
            # camera sample of a new axis trial.
            self.tip_forecast_monitor.clear()
            self.last_tip_forecast_result = None

    def _path_reference_cb(self, message: TipReferenceHorizon):
        """Accept one replace-only, timestamped Cartesian path preview."""
        with self._lock:
            if not message.positions:
                if (self.path_reference is None
                        or not message.path_id
                        or message.path_id == self.path_reference.path_id):
                    self.path_reference = None
                    if self.reference_source == "PATH":
                        self.reference_source = "NONE"
                        self.target = None
                return
        try:
            if message.header.frame_id != self.frame_id:
                raise ValueError("path_reference_frame_mismatch")
            positions = np.asarray(
                [[point.x, point.y, point.z]
                 for point in message.positions], dtype=np.float64)
            tangents = np.asarray(
                [[value.x, value.y, value.z]
                 for value in message.tangents], dtype=np.float64)
            if (tangents.shape != positions.shape
                    or not np.all(np.isfinite(tangents))):
                raise ValueError("invalid_path_reference_tangents")
            reference = ReferenceHorizon(
                path_id=str(message.path_id),
                sequence=int(message.sequence),
                source_timestamp_ns=_stamp_ns(message.header),
                received_at_s=self._steady(),
                sample_period_s=float(message.sample_period_s),
                positions_m=positions,
                tangents=tangents,
                expiry_s=min(
                    float(message.expiry_s),
                    float(self.get_parameter(
                        "path_reference_timeout_s").value)),
                progress_m=float(message.progress_m),
                total_length_m=float(message.total_length_m),
                final_hold=bool(message.final_hold),
                progress_paused=bool(message.progress_paused))
        except (TypeError, ValueError) as error:
            self._fault_if_active(str(error))
            return
        with self._lock:
            prior = self.path_reference
            if (prior is not None and reference.path_id == prior.path_id
                    and reference.sequence <= prior.sequence):
                return
            if (self.armed and self.reference_source == "POINT"):
                self._fault_locked("reference_source_conflict")
                return
            if (self.armed and prior is not None
                    and reference.path_id != prior.path_id):
                self._fault_locked("path_reference_id_changed")
                return
            if prior is None or reference.path_id != prior.path_id:
                self.reversal_scheduler.reset_intent()
            self.path_reference = reference
            self.reference_source = "PATH"
            offset_s = max(
                0.0, (self._now_ns()-reference.source_timestamp_ns)/1e9)
            index = min(
                len(reference.positions_m)-1,
                int(round(offset_s/reference.sample_period_s)))
            self.target = reference.positions_m[index].copy()
            self.planner.reset_capture_target()
            self.tip_forecast_monitor.clear()
            self.last_tip_forecast_result = None

    def _collection_event_cb(self, _message: String):
        with self._lock:
            self.collection_present = True
            self._fault_locked("collection_event_detected")

    def _arm_cb(self, request: SetBool.Request, response: SetBool.Response):
        # Never let a service request wait indefinitely behind recurrent
        # sensor work. Collection coexistence is checked again at the start of
        # every plan tick, before a manager mode can be claimed.
        if not self._lock.acquire(timeout=0.25):
            response.success = False
            response.message = "controller_busy"
            return response
        try:
            if not request.data:
                self._release_locked()
                self.armed = False
                self.fault_latched = False
                self.state = ControllerState.DISARMED
                self.state_reason = "disarmed_by_user"
                response.success = True
                response.message = self.state.value
                return response
            if self.collection_present:
                response.success = False
                response.message = "collection_node_present"
                return response
            self._release_locked()
            self.reset_pending = True
            self.armed = True
            self.fault_latched = False
            self.last_fault_snapshot = {}
            self.state = ControllerState.WAITING_FOR_MANAGER
            self.state_reason = "armed_waiting_for_inputs"
            response.success = True
            response.message = self.state.value
            return response
        finally:
            self._lock.release()

    def _stop_cb(self, _request: Trigger.Request, response: Trigger.Response):
        with self._lock:
            self._fault_locked("emergency_stop", stop_motor=True)
        response.success = True
        response.message = "FAULTED"
        return response

    def _reset_simulation_state_cb(
            self, _request: Trigger.Request, response: Trigger.Response):
        """Reset estimator/control memory only on an explicitly simulated node."""
        # Match the planner's lock order (_plan_lock -> _lock) so a reset can
        # never deadlock against a timer callback finishing its disarmed tick.
        if not self._plan_lock.acquire(timeout=0.25):
            response.success = False
            response.message = "planner_busy"
            return response
        try:
            with self._lock:
                if self.armed or self.mode_claimed:
                    response.success = False
                    response.message = "controller_must_be_disarmed"
                    return response
                self.runtime.reset()
                self.backlash_estimator = BacklashStateEstimator(
                    self.backlash_config, self.engaged_gain_config)
                self.backlash_snapshot = self.backlash_estimator.snapshot()
                self.takeup_arbiter.reset()
                self.reversal_scheduler.reset()
                self._clear_takeup_saturation_locked("simulation_reset")
                self.pending_encoder_counts = None
                self.pending_encoder_timestamp_ns = None
                self.pending_encoder_arrival = None
                self.pending_marker_timestamp_ns = None
                self.pending_marker_points = None
                self.pending_marker_quality = None
                self.pending_marker_arrival = None
                self._simulation_reset_source_floor_ns = self._now_ns()
                self._raw_encoder_history.clear()
                self._backlash_history.clear()
                self.latest_raw_encoder_counts = None
                self.encoder_valid = False
                self.encoder_time = None
                self.encoder_receive_time = None
                self.encoder_source_timestamp_ns = None
                self.position_valid = False
                self.position_time = None
                self.position_source_timestamp_ns = None
                self.marker_time = None
                self.observed_tip = None
                self.observed_tip_time = None
                self.observed_tip_timestamp_ns = None
                self.last_marker_result = None
                self.runtime_initialized = False
                self.estimator_health = "UNINITIALIZED"
                self.accepted_observations = 0
                self.consecutive_rejections = 0
                self.model_diagnostics = self.runtime.diagnostics()
                self.model_valid = True
                self.target = None
                self.reference_source = "NONE"
                self.path_reference = None
                self.last_command.fill(0.0)
                self.last_effective_command.fill(0.0)
                self.last_command_time = None
                self.last_plan = None
                self.tip_forecast_monitor.clear()
                self.last_tip_forecast_result = None
                self.planner_warmed = False
                self.consecutive_deadline_misses = 0
                self.estimator_catchup_started_at = None
                self.estimator_catchup_pause_count = 0
                with self._snapshot_lock:
                    self._planner_snapshot = None
                    self._planner_snapshot_source_time = None
                    self._planner_transmission_snapshot = None
                self.planner.reset()
                self.reset_pending = False
                self.state = ControllerState.DISARMED
                self.state_reason = "simulation_state_reset"
        finally:
            self._plan_lock.release()
        response.success = True
        response.message = "simulation controller state reset"
        self.get_logger().info(response.message)
        return response

    def _scan_collection_locked(self):
        names = {name.rsplit("/", 1)[-1] for name in self.get_node_names()}
        if "collection" in names:
            self.collection_present = True

    def _gate_inputs_locked(self, now: float) -> GateInputs:
        def age(value):
            return None if value is None else max(0.0, now-value)

        skew = paired_source_skew_s(
            self.position_source_timestamp_ns,
            self.encoder_source_timestamp_ns)
        return GateInputs(
            manager_ready=self.manager_ready,
            manager_age_s=age(self.manager_time),
            position_age_s=age(self.position_time),
            encoder_age_s=age(self.encoder_time),
            encoder_receive_age_s=age(self.encoder_receive_time),
            position_encoder_skew_s=skew,
            marker_age_s=age(self.marker_time),
            marker_diagnostic_age_s=age(self.marker_diagnostic_time),
            marker_diagnostic_error=self.marker_diagnostic_error,
            accepted_observations=self.accepted_observations,
            required_observations=self.required_observations,
            consecutive_rejections=self.consecutive_rejections,
            maximum_rejections=self.maximum_rejections,
            target_available=self.target is not None,
            collection_present=self.collection_present,
            position_valid=self.position_valid,
            encoder_valid=self.encoder_valid,
            model_valid=self.model_valid,
            estimator_health=self.estimator_health,
            last_marker_update_rejected=(
                self.last_marker_result is not None
                and not self.last_marker_result.accepted),
            last_marker_update_reason=(
                "none" if self.last_marker_result is None
                else str(self.last_marker_result.reason)))

    def _clear_estimator_catchup_locked(self):
        self.estimator_catchup_started_at = None

    def _pause_for_estimator_catchup_locked(self, now, reason):
        """Force zero for a bounded processed-encoder backlog.

        Raw device freshness cannot authorize motion. It only prevents a
        false latched transport fault while the estimator owner finishes a
        bounded correction; every heartbeat in this state publishes zero.
        """
        if reason != "encoder_stale":
            return False
        new_pause = self.estimator_catchup_started_at is None
        if new_pause:
            self.estimator_catchup_started_at = now
        elapsed = now-self.estimator_catchup_started_at
        inputs = self._gate_inputs_locked(now)
        if not recoverable_encoder_processing_lag(
                inputs, self.limits, elapsed,
                self.estimator_catchup_timeout_s):
            return False
        if new_pause:
            self.estimator_catchup_pause_count += 1
        self.last_command.fill(0.0)
        self.last_effective_command.fill(0.0)
        self.last_command_time = now
        self.consecutive_deadline_misses = 0
        self.state = ControllerState.ACTIVE
        self.state_reason = "estimator_catchup_zero"
        self._publish_velocity_locked(np.zeros(6))
        return True

    def _compensated_command_locked(self, desired, position):
        if not self.backlash_compensation_enabled:
            return np.asarray(desired, dtype=np.float64).copy()
        return self.backlash_compensator.command(
            desired, position, self.contract, self.backlash_snapshot)

    def _clear_takeup_saturation_locked(self, reason):
        self.blocked_motor_direction.fill(0)
        self.takeup_saturation_position = None
        self.takeup_saturation_position_timestamp_ns = None
        self.takeup_saturation_release_reason = str(reason)

    def _record_takeup_saturation_locked(self, decision, position):
        mask = np.asarray(decision.saturated_mask, dtype=bool)
        if mask.shape != (3,) or not np.any(mask):
            return
        direction = np.asarray(decision.direction, dtype=np.int8)
        if direction.shape != (3,) or not np.any(direction):
            return
        # Preserve the complete infeasible coupled mode. Recording only
        # saturated axes allows an isolated-axis feasibility check to
        # erase a block even while the original combination is impossible.
        self.blocked_motor_direction[:] = direction
        self.takeup_saturation_position = np.asarray(
            position, dtype=np.float64).copy()
        self.takeup_saturation_position_timestamp_ns = (
            self.position_source_timestamp_ns)
        self.takeup_saturation_release_reason = "active"

    def _refresh_blocked_motor_directions_locked(self, position):
        """Release a saturation block after feasibility or re-engagement.

        The block describes the take-up state at the saturation origin. Once
        every involved shaft is response-confirmed ENGAGED, that origin state
        no longer exists; normal joint-limit projection remains authoritative.
        """
        direction = self.blocked_motor_direction.copy()
        if not np.any(direction):
            return
        if self.takeup_arbiter.physical_direction_vector_is_feasible(
                direction, position, self.contract):
            self._clear_takeup_saturation_locked("feasible_again")
            return
        active = np.flatnonzero(direction)
        phase = tuple(self.backlash_snapshot.phase)
        remaining = np.asarray(
            self.backlash_snapshot.remaining_rad, dtype=np.float64)
        if all(phase[axis] == "ENGAGED"
               and remaining[axis] <= 1e-9 for axis in active):
            self._clear_takeup_saturation_locked(
                "engagement_revalidated")

    def _release_takeup_for_replan_locked(self):
        """Release one transaction without erasing useful planner memory.

        Ordinary response completion invalidates the executable action, not
        the MPPI sampling distribution or the previous desired direction.
        Saturation changes the feasible set and therefore retains the former
        hard-reset behavior.
        """
        transaction_state = self.takeup_arbiter.state
        released = self.takeup_arbiter.release_for_replan()
        if (released and transaction_state
                == TakeupTransactionArbiter.SATURATED_REPLAN):
            self.planner.reset()
            self.last_effective_command.fill(0.0)
        return released, transaction_state

    @_instrument_timer("plan")
    def _plan_tick(self):
        callback_started = self._steady()
        if not self._plan_lock.acquire(blocking=False):
            return
        try:
            warmup = False
            with self._lock:
                if self.reset_pending:
                    self.planner.reset()
                    self.reset_pending = False
                if not self.armed or self.fault_latched:
                    return
                self._scan_collection_locked()
                now = self._steady()
                gate_state, reason = readiness(
                    self._gate_inputs_locked(now), self.limits)
                if gate_state != ControllerState.READY or reason != "ready":
                    if (self.state == ControllerState.ACTIVE
                            or self.mode_claimed):
                        if self._pause_for_estimator_catchup_locked(
                                now, reason):
                            return
                        self._fault_locked(reason)
                    else:
                        self.state, self.state_reason = gate_state, reason
                    return
                self._clear_estimator_catchup_locked()
                if not self.planner_warmed:
                    # Run the framework's lazy first rollout before claiming
                    # a control mode. Its command is discarded below.
                    warmup = True
                    self.state = ControllerState.READY
                    self.state_reason = "warming_planner"
                elif not self.mode_claimed:
                    self._send_mode_locked(ManagerEvent.JOINT_VEL)
                    self.mode_claimed = True
                    self.mode_requested_at = now
                    self.state = ControllerState.READY
                    self.state_reason = "waiting_for_joint_velocity_mode"
                    return
                if (not warmup and self.takeup_transaction_enabled
                        and self.takeup_arbiter.state
                        in (TakeupTransactionArbiter.TAKEUP_ACTIVE,
                            TakeupTransactionArbiter.CONFIRMATION_HOLD)):
                    decision = self.takeup_arbiter.advance(
                        self.position, self.contract, self.backlash_snapshot)
                    if decision.state == TakeupTransactionArbiter.FAILED:
                        self._fault_locked(decision.reason)
                        return
                    if (decision.state
                            == TakeupTransactionArbiter.SATURATED_REPLAN):
                        self._record_takeup_saturation_locked(
                            decision, self.position)
                    self.last_command = decision.command_logical_velocity
                    self.last_command_time = now
                    self.state = ControllerState.ACTIVE
                    self.state_reason = decision.reason
                    # The zero command at the completion boundary must be
                    # visible for a full planner period. A new post-take-up
                    # plan is generated on the next tick.
                    return
                if (not warmup and self.takeup_transaction_enabled
                        and self.takeup_arbiter.state
                        in (TakeupTransactionArbiter.REPLAN_REQUIRED,
                            TakeupTransactionArbiter.SATURATED_REPLAN)):
                    _, completed_transaction = (
                        self._release_takeup_for_replan_locked())
                    self.last_command.fill(0.0)
                    self.last_command_time = now
                    self.state_reason = (
                        "replanning_after_takeup_saturation"
                        if completed_transaction
                        == TakeupTransactionArbiter.SATURATED_REPLAN else
                        "replanning_after_takeup_response")
                if not warmup:
                    if now-self.mode_requested_at < self.mode_settle_s:
                        return
                    # Do not advertise ACTIVE until a first valid command has
                    # actually been committed. The heartbeat runs at 100 Hz;
                    # marking ACTIVE before the first plan completes lets it
                    # observe no command and latch a spurious stale fault.
                    if self.last_command_time is None:
                        self.state = ControllerState.READY
                        self.state_reason = "planning_first_command"
                    if (self.reference_source == "POINT"
                            and self.planner.point_capture_hold_active()):
                        # The preceding full solve already selected this
                        # deterministic branch. Re-running GPU rollouts cannot
                        # change the command during its fixed hold.
                        self.last_command.fill(0.0)
                        self.last_effective_command.fill(0.0)
                        self.last_command_time = now
                        self.consecutive_deadline_misses = 0
                        self.state = ControllerState.ACTIVE
                        self.state_reason = "capture_hold_zero"
                        return
                position = self.position.copy()
                self._refresh_blocked_motor_directions_locked(position)
                blocked_motor_direction = (
                    self.blocked_motor_direction.copy())
                reversal_state = self.reversal_scheduler.synchronize(
                    self.backlash_snapshot.phase,
                    self.backlash_snapshot.engaged_direction,
                    self.accepted_observations, now)
                target = self.target.copy()
                reference_source = self.reference_source
                path_reference = self.path_reference
                previous = self.last_effective_command.copy()
                observed_tip = (
                    None if self.observed_tip is None else
                    self.observed_tip.copy())

            # The estimator owner publishes complete replace-only snapshots.
            # Reading one never waits for marker correction or rewind/replay.
            with self._timed_lock(self._snapshot_lock, "plan_snapshot"):
                root = self._planner_snapshot
                snapshot_source_time = self._planner_snapshot_source_time
            if root is None or snapshot_source_time is None:
                with self._lock:
                    self._fault_locked("planner_snapshot_unavailable")
                return
            if (self._steady()-snapshot_source_time
                    > self.limits.feedback_s):
                with self._lock:
                    if self._pause_for_estimator_catchup_locked(
                            self._steady(), "encoder_stale"):
                        return
                    self._fault_locked("planner_snapshot_stale")
                return
            if reference_source == "PATH":
                if path_reference is None:
                    with self._lock:
                        self._fault_locked("path_reference_missing")
                    return
                try:
                    target, target_tangent = path_reference.sample(
                        root.timestamp_ns,
                        self.planner.config.horizon_steps,
                        self.planner.config.step_s, self._steady())
                except ValueError as error:
                    with self._lock:
                        self._fault_locked(str(error))
                    return
            plan = self.planner.plan(
                root, position, target,
                target_tangent_base=(
                    None if reference_source != "PATH"
                    or path_reference.final_hold else target_tangent),
                observed_tip_base_m=observed_tip,
                previous_logical_velocity=previous,
                deadline_started_s=callback_started,
                transmission_state=self.backlash_snapshot,
                takeup_motor_radians_per_second=(
                    self.backlash_compensator.motor_radians_per_second(
                        self.contract)),
                blocked_motor_direction=blocked_motor_direction,
                direction_lease=reversal_state.lease_direction,
                approved_reversal_direction=(
                    reversal_state.approved_direction))
            if warmup:
                # Warm-start control values must not inherit the discarded
                # rollout. A deadline miss alone is expected on a cold path;
                # all other planner failures remain real faults.
                self.planner.reset()
                with self._lock:
                    plan = self.planner.enforce_deadline(
                        plan, callback_started)
                    self._publish_control_cycle_timing_locked(
                        plan, callback_started, root.timestamp_ns)
                    if not self.armed or self.fault_latched:
                        return
                    if plan.reason not in ("ok", "deadline_missed"):
                        self._fault_locked(f"planner_warmup:{plan.reason}")
                        return
                    self.planner_warmed = True
                    self.state = ControllerState.READY
                    self.state_reason = "planner_warmed"
                    self.get_logger().info(
                        "MPPI warm-up complete; executable plans remain "
                        "subject to the configured deadline")
                return
            with self._lock:
                plan = self.planner.enforce_deadline(
                    plan, callback_started)
                self._publish_control_cycle_timing_locked(
                    plan, callback_started, root.timestamp_ns)
                if not self.armed or self.fault_latched:
                    return
                gate_state, reason = readiness(
                    self._gate_inputs_locked(self._steady()), self.limits)
                if gate_state != ControllerState.READY or reason != "ready":
                    if self._pause_for_estimator_catchup_locked(
                            self._steady(), reason):
                        return
                    self._fault_locked(reason)
                    return
                self._clear_estimator_catchup_locked()
                if not plan.valid and plan.reason == "deadline_missed":
                    self.consecutive_deadline_misses += 1
                    # Retain the current miss before any threshold-triggered
                    # fault so the terminal diagnostic reports the plan that
                    # actually caused the transition, not an older miss.
                    self.last_plan = plan
                    if (self.consecutive_deadline_misses
                            >= self.maximum_deadline_misses):
                        self._fault_locked(
                            "planner:repeated_deadline_miss")
                        return
                    # Never execute or retain an overdue action. Publish zero
                    # for this cycle and allow the next independently sampled
                    # plan to recover; repeated misses latch above.
                    self.last_command.fill(0.0)
                    self.last_effective_command.fill(0.0)
                    self.last_command_time = self._steady()
                    self.state = ControllerState.ACTIVE
                    self.state_reason = "planner_deadline_miss_zero"
                    self._publish_plan_locked(plan)
                    self.get_logger().warn(
                        "MPPI deadline missed; commanding zero (%d/%d)" % (
                            self.consecutive_deadline_misses,
                            self.maximum_deadline_misses))
                    return
                if not plan.valid:
                    self._fault_locked(f"planner:{plan.reason}")
                    return
                first_active_plan = self.last_command_time is None
                self.consecutive_deadline_misses = 0
                self.last_plan = plan
                self.reversal_scheduler.observe_plan(
                    plan.proposed_reversal_direction,
                    plan.reversal_axis_cost_improvement,
                    plan.reversal_axis_terminal_error_improvement_mm,
                    self.accepted_observations, self._steady())
                self.last_effective_command = (
                    plan.command_logical_velocity.copy())
                execute_plan = True
                if self.takeup_transaction_enabled:
                    decision = self.takeup_arbiter.begin(
                        self.last_effective_command, position, self.contract,
                        self.backlash_snapshot)
                    if decision.state == TakeupTransactionArbiter.FAILED:
                        self._fault_locked(decision.reason)
                        return
                    if (decision.state
                            == TakeupTransactionArbiter.SATURATED_REPLAN):
                        self._record_takeup_saturation_locked(
                            decision, position)
                    self.last_command = decision.command_logical_velocity
                    execute_plan = decision.execute_plan
                    self.state_reason = decision.reason
                    if (decision.state
                            in (TakeupTransactionArbiter.TAKEUP_ACTIVE,
                                TakeupTransactionArbiter.CONFIRMATION_HOLD)):
                        self.reversal_scheduler.mark_transaction_started(
                            decision.direction)
                else:
                    self.last_command = self._compensated_command_locked(
                        self.last_effective_command, position)
                    self.state_reason = "active"
                self.last_command_time = self._steady()
                self.state = ControllerState.ACTIVE
                compensating_backlash = not np.allclose(
                    self.last_command, self.last_effective_command,
                    rtol=0.0, atol=1e-12)
                forecast_matches_command = (
                    execute_plan and not compensating_backlash
                    and not np.any(
                        self.backlash_snapshot.taking_up
                        & self.takeup_response_free_mask))
                if forecast_matches_command:
                    self.planner.note_capture_control_executed(
                        self.last_command)
                if (forecast_matches_command
                        and plan.command_tip_sequence_m is not None
                        and self.observed_tip is not None
                        and self.observed_tip_timestamp_ns is not None):
                    self.tip_forecast_monitor.add(
                        root.timestamp_ns,
                        self.observed_tip_timestamp_ns,
                        (self.planner.config.point_rollout_step_s
                         if reference_source != "PATH"
                         and self.planner.config.point_rollout_step_s > 0.0
                         else self.planner.config.step_s),
                        self.observed_tip,
                        plan.command_tip_sequence_m[0],
                        command_logical_velocity=(
                            self.last_command),
                        motor_radians_per_second=(
                            plan.compensated_motor_radians_per_second_sequence[0]),
                        joint_position=position,
                        model_motor_angle_rad=(
                            root.motor_angle_rad.detach().cpu().numpy()),
                        interface_pose=(
                            root.interface_pose.detach().cpu().numpy()
                            .reshape(-1)),
                        interface_jacobian=np.asarray(
                            root.adaptive_jacobian.jacobian,
                            dtype=np.float64).reshape(-1),
                        target_tip_m=(
                            target[-1] if target.ndim == 2 else target),
                        capture_hold=plan.hold_branch_applied)
                self._publish_plan_locked(plan)
                if first_active_plan:
                    self.get_logger().info(
                        "catheter MPPI ACTIVE; first plan %.3f ms" % (
                            1e3*plan.elapsed_s))
        finally:
            self._plan_lock.release()

    @_instrument_timer("heartbeat")
    def _heartbeat_tick(self):
        # A timer must never consume an executor thread while waiting behind a
        # recurrent sensor update. Skipping one tick is fail-safe; when output
        # is enabled the independent manager watchdog still owns freshness.
        if not self._lock.acquire(blocking=False):
            return
        try:
            if not self.mode_claimed:
                return
            if not self.armed or self.fault_latched:
                self._publish_velocity_locked(np.zeros(6))
                return
            now = self._steady()
            gate_state, reason = readiness(
                self._gate_inputs_locked(now), self.limits)
            if gate_state != ControllerState.READY or reason != "ready":
                if self._pause_for_estimator_catchup_locked(now, reason):
                    return
                self._fault_locked(reason)
                return
            self._clear_estimator_catchup_locked()
            if self.state != ControllerState.ACTIVE:
                self._publish_velocity_locked(np.zeros(6))
                return
            if (self.takeup_transaction_enabled
                    and self.takeup_arbiter.state
                    in (TakeupTransactionArbiter.TAKEUP_ACTIVE,
                        TakeupTransactionArbiter.CONFIRMATION_HOLD,
                        TakeupTransactionArbiter.REPLAN_REQUIRED,
                        TakeupTransactionArbiter.SATURATED_REPLAN,
                        TakeupTransactionArbiter.FAILED)):
                decision = self.takeup_arbiter.advance(
                    self.position, self.contract, self.backlash_snapshot)
                if decision.state == TakeupTransactionArbiter.FAILED:
                    self._fault_locked(decision.reason)
                    return
                if (decision.state
                        == TakeupTransactionArbiter.SATURATED_REPLAN):
                    self._record_takeup_saturation_locked(
                        decision, self.position)
                self.last_command = decision.command_logical_velocity
                self.last_command_time = now
                self.state_reason = decision.reason
                command = self.contract.project_publishable_velocity(
                    self.last_command, self.position)
                self._publish_velocity_locked(command)
                return
            if (self.last_command_time is None
                    or now-self.last_command_time > self.command_timeout_s):
                # A missed planner deadline explicitly replaces the cached
                # command with zero. Keep heartbeating that fail-safe zero
                # while the next expensive rollout is in flight; the planner
                # owns the independent consecutive-miss fault. Treating a
                # known-zero cache as a stale motion command caused a
                # misleading planned_command_stale fault after one miss.
                if (self.state_reason == "planner_deadline_miss_zero"
                        and not np.any(self.last_command)):
                    self._publish_velocity_locked(np.zeros(6))
                    return
                self._fault_locked("planned_command_stale")
                return
            # A plan is refreshed at a lower rate than this heartbeat. Apply
            # the autonomous operational-position reserve against the newest
            # feedback on every transmission so a cached command cannot drive
            # through that reserve between planner commits.
            if not self.takeup_transaction_enabled:
                self.last_command = self._compensated_command_locked(
                    self.last_effective_command, self.position)
            command = self.contract.project_publishable_velocity(
                self.last_command, self.position)
            self._publish_velocity_locked(command)
        finally:
            self._lock.release()

    def _publish_velocity_locked(self, velocity):
        if not self.command_output_enabled:
            return
        values = np.asarray(velocity, dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            values = np.zeros(6)
        message = ControlStream()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.source
        message.joint_vel = [float(value) for value in values]
        self.control_pub.publish(message)

    def _send_mode_locked(self, mode: int):
        if not self.command_output_enabled:
            return
        message = ManagerEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.source
        message.predicate = ManagerEvent.MODE
        message.text = chr(mode)
        self.event_pub.publish(message)

    def _release_locked(self):
        if self.mode_claimed:
            self._publish_velocity_locked(np.zeros(6))
            self._send_mode_locked(ManagerEvent.NONE)
        self.mode_claimed = False
        self.mode_requested_at = None
        self.last_command.fill(0.0)
        self.last_effective_command.fill(0.0)
        self.takeup_arbiter.reset()
        self.reversal_scheduler.reset()
        self._clear_takeup_saturation_locked("controller_release")
        self.last_command_time = None
        # A disarmed controller has no current executable plan. Clearing this
        # also prevents a final deadline-miss phase sample from being repeated
        # in every subsequent status message and biasing audit distributions.
        self.last_plan = None
        self.tip_forecast_monitor.clear()
        self.last_tip_forecast_result = None
        self.consecutive_deadline_misses = 0
        self.estimator_catchup_started_at = None
        self.reset_pending = True

    def _fault_if_active(self, reason: str):
        with self._lock:
            if self.state == ControllerState.ACTIVE or self.mode_claimed:
                self._fault_locked(reason)

    def _fault_snapshot_locked(self, reason: str):
        """Capture the evidence that existed before fail-closed teardown."""
        now = self._steady()
        inputs = self._gate_inputs_locked(now)
        latest = self._timing.latest()
        plan = self.last_plan
        return {
            "reason": str(reason),
            "marker_age_ms": (None if inputs.marker_age_s is None
                              else 1e3*inputs.marker_age_s),
            "marker_source_age_ms": (
                None if self.observed_tip_timestamp_ns is None else
                max(0.0, 1e-6*(self._now_ns()
                                - self.observed_tip_timestamp_ns))),
            "marker_diagnostic_age_ms": (
                None if inputs.marker_diagnostic_age_s is None else
                1e3*inputs.marker_diagnostic_age_s),
            "marker_diagnostic_error": bool(
                self.marker_diagnostic_error),
            "marker_diagnostic": self.marker_diagnostic_message,
            "consecutive_rejections": int(
                self.consecutive_rejections),
            "marker_update_reason": (
                "none" if self.last_marker_result is None else
                str(self.last_marker_result.reason)),
            "position_age_ms": (None if inputs.position_age_s is None
                                else 1e3*inputs.position_age_s),
            "encoder_age_ms": (None if inputs.encoder_age_s is None
                               else 1e3*inputs.encoder_age_s),
            "encoder_receive_age_ms": (
                None if inputs.encoder_receive_age_s is None else
                1e3*inputs.encoder_receive_age_s),
            "planner_snapshot_age_ms": (
                None if self._planner_snapshot_source_time is None else
                1e3*max(0.0, now-self._planner_snapshot_source_time)),
            "planner_elapsed_ms": (
                None if plan is None else 1e3*plan.elapsed_s),
            "planner_reason": (None if plan is None else plan.reason),
            "planner_deadline_ms": (
                1e3*self.planner.config.planning_deadline_s),
            "consecutive_deadline_misses": int(
                self.consecutive_deadline_misses),
            "latest_timing_ms": latest,
            "model_marker_timing_ms": {
                key: self.model_diagnostics.get(key)
                for key in ("marker_timing_rewind_ms",
                            "marker_timing_correction_ms",
                            "marker_timing_replay_ms",
                            "marker_timing_total_ms")},
        }

    def _fault_locked(self, reason: str, stop_motor: bool = False):
        if not self.armed and not stop_motor:
            return
        self.last_fault_snapshot = self._fault_snapshot_locked(reason)
        if self.mode_claimed:
            self._publish_velocity_locked(np.zeros(6))
        if stop_motor:
            event = ManagerEvent()
            event.header.stamp = self.get_clock().now().to_msg()
            event.header.frame_id = self.source
            event.predicate = ManagerEvent.STOP_MOTOR
            self.event_pub.publish(event)
        if self.mode_claimed:
            self._send_mode_locked(ManagerEvent.NONE)
        self.mode_claimed = False
        self.armed = False
        self.fault_latched = True
        self.state = ControllerState.FAULTED
        self.state_reason = str(reason)
        self.last_command.fill(0.0)
        self.last_effective_command.fill(0.0)
        self.takeup_arbiter.reset()
        self.reversal_scheduler.reset()
        self.blocked_motor_direction.fill(0)
        self.takeup_saturation_position = None
        self.takeup_saturation_position_timestamp_ns = None
        self.last_command_time = None
        self.estimator_catchup_started_at = None
        self.tip_forecast_monitor.clear()
        self.reset_pending = True
        self.get_logger().error(
            "catheter MPPI fault: %s; snapshot=%s" % (
                reason, json.dumps(self.last_fault_snapshot,
                                   separators=(",", ":"),
                                   sort_keys=True)))

    def _publish_control_cycle_timing_locked(
            self, plan, callback_started: float, state_timestamp_ns: int):
        """Publish one low-overhead causal timing row per completed solve."""
        now = self._steady()
        latest = self._timing.latest()
        message = ControlCycleTiming()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.source
        message.estimator_state_timestamp_ns = int(state_timestamp_ns)
        message.accepted_marker_timestamp_ns = int(
            self.observed_tip_timestamp_ns or 0)
        message.marker_source_age_ms = (
            math.nan if self.observed_tip_timestamp_ns is None else
            max(0.0, 1e-6*(self._now_ns()
                            - self.observed_tip_timestamp_ns)))
        message.accepted_marker_commit_age_ms = (
            math.nan if self.marker_time is None else
            1e3*max(0.0, now-self.marker_time))
        message.position_commit_age_ms = (
            math.nan if self.position_time is None else
            1e3*max(0.0, now-self.position_time))
        message.encoder_commit_age_ms = (
            math.nan if self.encoder_time is None else
            1e3*max(0.0, now-self.encoder_time))
        message.planner_snapshot_age_ms = (
            math.nan if self._planner_snapshot_source_time is None else
            1e3*max(0.0, now-self._planner_snapshot_source_time))
        message.planner_callback_elapsed_ms = 1e3*max(
            0.0, now-callback_started)
        message.planner_elapsed_ms = 1e3*float(plan.elapsed_s)
        message.sample_projection_ms = float(plan.sample_projection_ms)
        message.rollout_ms = float(plan.rollout_ms)
        message.cost_weighting_ms = float(plan.cost_weighting_ms)
        message.update_projection_ms = float(plan.update_projection_ms)
        message.estimator_callback_latest_ms = float(
            latest.get("estimator_callback_duration", math.nan))
        message.estimator_timer_lateness_latest_ms = float(
            latest.get("estimator_timer_lateness", math.nan))
        message.marker_pending_age_latest_ms = float(
            latest.get("marker_pending_age", math.nan))
        message.marker_owner_duration_latest_ms = float(
            latest.get("marker_owner_duration", math.nan))
        message.marker_rewind_ms = float(self.model_diagnostics.get(
            "marker_timing_rewind_ms", math.nan))
        message.marker_correction_ms = float(self.model_diagnostics.get(
            "marker_timing_correction_ms", math.nan))
        message.marker_replay_ms = float(self.model_diagnostics.get(
            "marker_timing_replay_ms", math.nan))
        message.marker_total_ms = float(self.model_diagnostics.get(
            "marker_timing_total_ms", math.nan))
        message.accepted_observations = int(self.accepted_observations)
        message.consecutive_rejections = int(self.consecutive_rejections)
        message.consecutive_deadline_misses = int(
            self.consecutive_deadline_misses)
        message.plan_valid = bool(plan.valid)
        message.plan_reason = str(plan.reason)
        message.controller_state = self.state.value
        message.controller_reason = self.state_reason
        self.control_cycle_timing_pub.publish(message)

    def _publish_plan_locked(self, plan):
        message = ControlStream()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.source
        message.joint_vel = [
            float(value) for value in plan.command_logical_velocity]
        self.plan_pub.publish(message)
        trajectory = plan.command_tip_sequence_m
        if trajectory is None:
            return
        cloud = PointCloud()
        cloud.header.stamp = message.header.stamp
        cloud.header.frame_id = self.frame_id
        cloud.points = [Point32(x=float(point[0]), y=float(point[1]),
                                z=float(point[2]))
                        for point in trajectory]
        self.trajectory_pub.publish(cloud)

    def _observe_capture_response_locked(self, result) -> bool:
        if (not result.capture_hold or result.target_tip_m is None
                or result.start_tip_m is None
                or result.predicted_terminal_tip_m is None
                or result.observed_tip_m is None):
            return False
        target = np.asarray(result.target_tip_m, dtype=np.float64)
        start_error = 1e3*float(np.linalg.vector_norm(
            target-result.start_tip_m))
        predicted_error = 1e3*float(np.linalg.vector_norm(
            target-result.predicted_terminal_tip_m))
        measured_error = 1e3*float(np.linalg.vector_norm(
            target-result.observed_tip_m))
        return self.planner.observe_capture_response(
            result.predicted_delta_mm,
            result.measured_delta_mm,
            predicted_progress_mm=start_error-predicted_error,
            measured_progress_mm=start_error-measured_error)

    def _publish_response_trace(self, result):
        """Publish one compact, causal model-versus-hardware response row."""
        required = (
            result.command_logical_velocity,
            result.motor_radians_per_second,
            result.joint_position,
            result.model_motor_angle_rad,
            result.interface_pose,
            result.interface_jacobian,
            result.target_tip_m,
            result.start_tip_m,
            result.predicted_terminal_tip_m,
            result.observed_tip_m,
        )
        if any(value is None for value in required):
            return
        message = MppiResponseTrace()
        message.header.stamp.sec = (
            result.observation_timestamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = (
            result.observation_timestamp_ns % 1_000_000_000)
        message.header.frame_id = self.frame_id
        message.forecast_start_timestamp_ns = result.start_timestamp_ns
        message.start_observation_timestamp_ns = (
            result.start_observation_timestamp_ns)
        message.forecast_due_timestamp_ns = result.due_timestamp_ns
        message.observation_timestamp_ns = result.observation_timestamp_ns
        message.command_logical_velocity = (
            result.command_logical_velocity.tolist())
        message.motor_radians_per_second = (
            result.motor_radians_per_second.tolist())
        message.joint_position = result.joint_position.tolist()
        message.model_motor_angle_rad = result.model_motor_angle_rad.tolist()
        message.interface_pose = result.interface_pose.tolist()
        message.interface_jacobian = result.interface_jacobian.tolist()
        message.target_tip_m = result.target_tip_m.tolist()
        message.start_tip_m = result.start_tip_m.tolist()
        message.predicted_tip_m = result.predicted_terminal_tip_m.tolist()
        message.observed_tip_m = result.observed_tip_m.tolist()
        message.predicted_delta_mm = result.predicted_delta_mm.tolist()
        message.measured_delta_mm = result.measured_delta_mm.tolist()
        message.endpoint_error_mm = result.endpoint_error_mm.tolist()
        message.endpoint_error_norm_mm = result.endpoint_error_norm_mm
        message.direction_cosine_valid = result.direction_cosine is not None
        message.direction_cosine = (
            0.0 if result.direction_cosine is None
            else result.direction_cosine)
        self.response_trace_pub.publish(message)

    @staticmethod
    def _optional_float(value):
        return math.nan if value is None else float(value)

    def _publish_estimator_trace(
            self, observation_timestamp_ns, state, result, diagnostics,
            raw_encoder_counts=None, observed_tip_m=None):
        """Publish the post-correction state at the camera source time.

        ``state`` comes from the rewind entry at the observation timestamp,
        before replay to the newest encoder. This makes response windows causal
        and prevents a present-time state from being assigned an older stamp.
        """
        message = EstimatorStateTrace()
        message.header.stamp.sec = (
            int(observation_timestamp_ns) // 1_000_000_000)
        message.header.stamp.nanosec = (
            int(observation_timestamp_ns) % 1_000_000_000)
        message.header.frame_id = self.frame_id
        message.observation_timestamp_ns = int(observation_timestamp_ns)
        message.state_timestamp_ns = int(state.timestamp_ns)

        motor = state.motor_angle_rad.detach().cpu().numpy()
        message.motor_angle_rad = motor.tolist()
        if raw_encoder_counts is None:
            raw_encoder_counts = motor/ENCODER_RADIANS_PER_COUNT
        message.raw_encoder_counts = np.asarray(
            raw_encoder_counts, dtype=np.float64).tolist()
        message.downstream = state.downstream.detach().cpu().numpy().tolist()
        message.interface_pose = (
            state.interface_pose.detach().cpu().numpy().reshape(-1).tolist())
        message.distal_strain = (
            state.strain.detach().cpu().numpy().reshape(-1).tolist())
        estimated_markers = self.runtime.markers_for_state(
            state).detach().cpu().numpy()
        message.estimated_tip_m = estimated_markers[-1].tolist()
        message.observed_tip_m = (
            [math.nan] * 3 if observed_tip_m is None else
            np.asarray(observed_tip_m, dtype=np.float64).tolist())
        covariance = state.estimator_covariance.detach().cpu().numpy()
        message.estimator_covariance_diagonal = (
            np.diag(covariance).tolist())
        message.estimator_covariance_trace = float(np.trace(covariance))

        message.accepted_observations = int(state.accepted_observations)
        message.consecutive_rejections = int(state.consecutive_rejections)
        message.estimator_health = state.health
        message.marker_update_reason = result.reason
        message.marker_rms_before_mm = self._optional_float(
            result.rms_before_mm)
        message.marker_rms_after_mm = self._optional_float(
            result.rms_after_mm)
        message.innovation_nis = self._optional_float(result.innovation_nis)
        message.innovation_nis_per_dof = self._optional_float(
            result.innovation_nis_per_dof)
        message.observable_rank = int(result.observable_rank)

        model = state.adaptive_jacobian
        message.interface_jacobian = np.asarray(
            model.jacobian, dtype=np.float64).reshape(-1).tolist()
        message.backlash_remaining_rad = (
            self.backlash_snapshot.remaining_rad.tolist())
        message.backlash_remaining_lower_rad = (
            self.backlash_snapshot.remaining_lower_rad.tolist())
        message.backlash_remaining_upper_rad = (
            self.backlash_snapshot.remaining_upper_rad.tolist())
        message.backlash_width_positive_lower_rad = (
            self.backlash_snapshot.width_positive_lower_rad.tolist())
        message.backlash_width_positive_upper_rad = (
            self.backlash_snapshot.width_positive_upper_rad.tolist())
        message.backlash_width_negative_lower_rad = (
            self.backlash_snapshot.width_negative_lower_rad.tolist())
        message.backlash_width_negative_upper_rad = (
            self.backlash_snapshot.width_negative_upper_rad.tolist())
        message.backlash_reversal_start_motor_rad = (
            self.backlash_snapshot.reversal_start_motor_rad.tolist())
        message.backlash_engagement_anchor_motor_rad = (
            self.backlash_snapshot.engagement_anchor_motor_rad.tolist())
        message.backlash_accumulated_takeup_rad = (
            self.backlash_snapshot.accumulated_takeup_rad.tolist())
        message.backlash_effective_motor_rad = (
            self.backlash_snapshot.effective_motor_rad.tolist())
        message.backlash_effective_motor_uncertainty_rad = (
            self.backlash_snapshot.effective_motor_uncertainty_rad.tolist())
        message.backlash_last_evidence_timestamp_ns = (
            self.backlash_snapshot.last_evidence_timestamp_ns.tolist())
        message.backlash_motion_direction = (
            self.backlash_snapshot.motion_direction.astype(np.int32).tolist())
        message.backlash_engaged_direction = (
            self.backlash_snapshot.engaged_direction.astype(np.int32).tolist())
        message.backlash_confidence = (
            self.backlash_snapshot.confidence.tolist())
        message.backlash_confirmation_count = (
            self.backlash_snapshot.confirmation_count.astype(np.int32).tolist())
        message.backlash_phase = list(self.backlash_snapshot.phase)
        message.backlash_response_classification = list(
            self.backlash_snapshot.response_classification)
        message.backlash_inferred_transmitted_increment_rad = (
            self.backlash_snapshot.inferred_transmitted_increment_rad.tolist())
        message.backlash_response_evidence = (
            self.backlash_snapshot.response_evidence.tolist())
        message.backlash_joint_response_residual = float(
            self.backlash_snapshot.joint_response_residual)
        message.backlash_distal_bending_increment = float(
            self.backlash_snapshot.distal_bending_increment)
        message.backlash_tendon_distal_response_evidence = float(
            self.backlash_snapshot.tendon_distal_response_evidence)
        message.backlash_tendon_distal_response_confirmed = bool(
            self.backlash_snapshot.tendon_distal_response_confirmed)
        message.adaptation_enabled = bool(
            diagnostics.get("adaptation_enabled", False))
        message.adaptation_axis = int(getattr(state, "last_rls_axis", -1))
        message.adaptation_reason = state.last_rls_reason
        message.adaptation_weight = float(state.last_rls_weight)
        message.adaptation_update_norm = float(state.last_rls_update_norm)
        message.adaptation_window_frames = int(state.last_rls_window_frames)
        message.rewind_ms = float(
            diagnostics.get("marker_timing_rewind_ms", 0.0))
        message.correction_ms = float(
            diagnostics.get("marker_timing_correction_ms", 0.0))
        message.replay_ms = float(
            diagnostics.get("marker_timing_replay_ms", 0.0))
        message.total_update_ms = float(
            diagnostics.get("marker_timing_total_ms", 0.0))
        self.estimator_trace_pub.publish(message)

    def _diagnostic_tick(self):
        # Diagnostics are best-effort and must not occupy an executor thread
        # while a sensor callback owns the state.
        if not self._lock.acquire(blocking=False):
            return
        tracking_log_message = None
        try:
            now = self._steady()
            inputs = self._gate_inputs_locked(now)
            tip_error_xyz_mm = None
            tip_error_norm_mm = None
            if self.target is not None and self.observed_tip is not None:
                tip_error_xyz_mm, tip_error_norm_mm = tip_tracking_error_mm(
                    self.target, self.observed_tip)
            report = DiagnosticArray()
            report.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "catheter_control/mppi"
            status.hardware_id = self.source
            status.message = self.state.value
            status.level = (
                DiagnosticStatus.ERROR
                if self.state == ControllerState.FAULTED else
                DiagnosticStatus.OK
                if self.state in (ControllerState.READY,
                                  ControllerState.ACTIVE) else
                DiagnosticStatus.WARN)
            capture_diagnostics = self.planner.capture_diagnostics()
            values = {
                "reason": self.state_reason,
                "fault_snapshot": (
                    "none" if not self.last_fault_snapshot else
                    json.dumps(self.last_fault_snapshot, separators=(",", ":"),
                               sort_keys=True)),
                "armed": str(self.armed),
                "mode_claimed": str(
                    self.mode_claimed and self.command_output_enabled),
                "control_session_started": str(self.mode_claimed),
                "command_output_enabled": str(self.command_output_enabled),
                "simulation_state_reset_enabled": str(bool(
                    self.get_parameter(
                        "simulation_state_reset_enabled").value)),
                "planner_warmed": str(self.planner_warmed),
                "consecutive_deadline_misses": str(
                    self.consecutive_deadline_misses),
                "manager_ready": str(self.manager_ready),
                "position_age_ms": (
                    "none" if inputs.position_age_s is None
                    else f"{1e3*inputs.position_age_s:.3f}"),
                "encoder_age_ms": (
                    "none" if inputs.encoder_age_s is None
                    else f"{1e3*inputs.encoder_age_s:.3f}"),
                "encoder_receive_age_ms": (
                    "none" if inputs.encoder_receive_age_s is None
                    else f"{1e3*inputs.encoder_receive_age_s:.3f}"),
                "estimator_catchup_timeout_ms": format(
                    1e3*self.estimator_catchup_timeout_s, ".6g"),
                "estimator_catchup_pause_count": str(
                    self.estimator_catchup_pause_count),
                "feedback_pair_skew_ms": (
                    "none" if inputs.position_encoder_skew_s is None
                    else f"{1e3*inputs.position_encoder_skew_s:.3f}"),
                "feedback_pair_skew_basis": "device_header_stamp",
                "marker_age_ms": (
                    "none" if inputs.marker_age_s is None
                    else f"{1e3*inputs.marker_age_s:.3f}"),
                "target_tip_m": (
                    "none" if self.target is None
                    else json.dumps(
                        self.target.tolist(), separators=(",", ":"))),
                "reference_source": self.reference_source,
                "path_reference_id": (
                    "none" if self.path_reference is None
                    else self.path_reference.path_id),
                "path_reference_sequence": (
                    "none" if self.path_reference is None
                    else str(self.path_reference.sequence)),
                "path_reference_age_ms": (
                    "none" if self.path_reference is None
                    else "%.3f" % (1e3*max(
                        0.0, now-self.path_reference.received_at_s))),
                "observed_tip_m": (
                    "none" if self.observed_tip is None
                    else json.dumps(
                        self.observed_tip.tolist(), separators=(",", ":"))),
                "observed_tip_age_ms": (
                    "none" if self.observed_tip_time is None
                    else f"{1e3*max(0.0, now-self.observed_tip_time):.3f}"),
                "tip_error_xyz_mm": (
                    "none" if tip_error_xyz_mm is None
                    else json.dumps(
                        tip_error_xyz_mm.tolist(), separators=(",", ":"))),
                "tip_error_norm_mm": (
                    "none" if tip_error_norm_mm is None
                    else f"{tip_error_norm_mm:.6g}"),
                "estimator_health": (
                    self.estimator_health),
                "transient_marker_rejection_tolerated": str(
                    inputs.estimator_health == "DEGRADED"
                    and inputs.last_marker_update_rejected
                    and 0 < inputs.consecutive_rejections
                    < inputs.maximum_rejections),
                "accepted_observations": str(inputs.accepted_observations),
                "consecutive_rejections": str(
                    inputs.consecutive_rejections),
                "marker_diagnostic": self.marker_diagnostic_message,
                "marker_estimator": self.marker_estimator,
                "estimator_filter_initial_covariance": str(
                    self.estimator_filter_initial_covariance),
                "estimator_filter_process_std_sqrt_s": str(
                    self.estimator_filter_process_std_sqrt_s),
                "estimator_initial_roll_hypotheses": str(
                    self.estimator_initial_roll_hypotheses),
                "marker_update_reason": (
                    "none" if self.last_marker_result is None
                    else self.last_marker_result.reason),
                "marker_rms_before_mm": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.rms_before_mm is None
                    else f"{self.last_marker_result.rms_before_mm:.6g}"),
                "marker_rms_after_mm": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.rms_after_mm is None
                    else f"{self.last_marker_result.rms_after_mm:.6g}"),
                "marker_maximum_residual_before_mm": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.maximum_residual_before_mm
                    is None else format(
                        self.last_marker_result.maximum_residual_before_mm,
                        ".6g")),
                "marker_maximum_residual_after_mm": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.maximum_residual_after_mm
                    is None else format(
                        self.last_marker_result.maximum_residual_after_mm,
                        ".6g")),
                "marker_nis": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.normalized_innovation is None
                    else format(
                        self.last_marker_result.normalized_innovation,
                        ".6g")),
                "marker_postfit_normalized_residual": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.normalized_innovation is None
                    else format(
                        self.last_marker_result.normalized_innovation,
                        ".6g")),
                "marker_innovation_nis": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.innovation_nis is None
                    else format(
                        self.last_marker_result.innovation_nis, ".6g")),
                "marker_innovation_nis_per_dof": (
                    "none" if self.last_marker_result is None
                    or self.last_marker_result.innovation_nis_per_dof is None
                    else format(
                        self.last_marker_result.innovation_nis_per_dof,
                        ".6g")),
                "marker_innovation_dof": (
                    "none" if self.last_marker_result is None
                    else str(self.last_marker_result.innovation_dof)),
                "marker_observable_rank": (
                    "none" if self.last_marker_result is None
                    else str(self.last_marker_result.observable_rank)),
                "command": json.dumps(self.last_command.tolist()),
                "effective_command": json.dumps(
                    self.last_effective_command.tolist()),
                "backlash_compensation_enabled": str(
                    self.backlash_compensation_enabled),
                "backlash_model_encoder_input": (
                    "estimated_transmitted"
                    if self.backlash_compensation_enabled else "raw_shaft"),
                "upstream_raw_encoder_counts_first_three": (
                    "none" if self.latest_raw_encoder_counts is None
                    else json.dumps(
                        self.latest_raw_encoder_counts.tolist(),
                        separators=(",", ":"))),
                "backlash_width_rad": json.dumps(
                    self.backlash_snapshot.width_rad.tolist()),
                "backlash_width_positive_rad": json.dumps(
                    self.backlash_snapshot.width_positive_rad.tolist()),
                "backlash_width_negative_rad": json.dumps(
                    self.backlash_snapshot.width_negative_rad.tolist()),
                "backlash_remaining_rad": json.dumps(
                    self.backlash_snapshot.remaining_rad.tolist()),
                "backlash_remaining_lower_rad": json.dumps(
                    self.backlash_snapshot.remaining_lower_rad.tolist()),
                "backlash_remaining_upper_rad": json.dumps(
                    self.backlash_snapshot.remaining_upper_rad.tolist()),
                "backlash_width_positive_interval_rad": json.dumps([
                    self.backlash_snapshot.width_positive_lower_rad.tolist(),
                    self.backlash_snapshot.width_positive_upper_rad.tolist()]),
                "backlash_width_negative_interval_rad": json.dumps([
                    self.backlash_snapshot.width_negative_lower_rad.tolist(),
                    self.backlash_snapshot.width_negative_upper_rad.tolist()]),
                "backlash_reversal_start_motor_rad": json.dumps(
                    self.backlash_snapshot.reversal_start_motor_rad.tolist()),
                "backlash_engagement_anchor_motor_rad": json.dumps(
                    self.backlash_snapshot.engagement_anchor_motor_rad.tolist()),
                "backlash_accumulated_takeup_rad": json.dumps(
                    self.backlash_snapshot.accumulated_takeup_rad.tolist()),
                "backlash_effective_motor_rad": json.dumps(
                    self.backlash_snapshot.effective_motor_rad.tolist()),
                "backlash_effective_motor_uncertainty_rad": json.dumps(
                    self.backlash_snapshot
                    .effective_motor_uncertainty_rad.tolist()),
                "backlash_last_evidence_timestamp_ns": json.dumps(
                    self.backlash_snapshot.last_evidence_timestamp_ns.tolist()),
                "backlash_motion_direction": json.dumps(
                    self.backlash_snapshot.motion_direction.tolist()),
                "backlash_engaged_direction": json.dumps(
                    self.backlash_snapshot.engaged_direction.tolist()),
                "backlash_confidence": json.dumps(
                    self.backlash_snapshot.confidence.tolist()),
                "backlash_confirmation_count": json.dumps(
                    self.backlash_snapshot.confirmation_count.tolist()),
                "backlash_inferred_transmitted_increment_rad": json.dumps(
                    self.backlash_snapshot
                    .inferred_transmitted_increment_rad.tolist()),
                "backlash_response_evidence": json.dumps(
                    self.backlash_snapshot.response_evidence.tolist()),
                "backlash_response_classification": json.dumps(
                    list(self.backlash_snapshot.response_classification)),
                "backlash_provisional_rejection_count": json.dumps(
                    self.backlash_snapshot
                    .provisional_rejection_count.tolist()),
                "backlash_joint_response_residual": format(
                    self.backlash_snapshot.joint_response_residual, ".6g"),
                "backlash_distal_bending_increment": format(
                    self.backlash_snapshot.distal_bending_increment, ".6g"),
                "backlash_tendon_distal_response_evidence": format(
                    self.backlash_snapshot
                    .tendon_distal_response_evidence, ".6g"),
                "backlash_tendon_distal_response_confirmed": str(
                    self.backlash_snapshot
                    .tendon_distal_response_confirmed),
                "backlash_phase": json.dumps(
                    list(self.backlash_snapshot.phase)),
                "engaged_gain_enabled": str(
                    self.backlash_snapshot.engaged_gain.enabled),
                "engaged_gain_mean": json.dumps(
                    self.backlash_snapshot.engaged_gain.mean.tolist()),
                "engaged_gain_lower": json.dumps(
                    self.backlash_snapshot.engaged_gain.lower.tolist()),
                "engaged_gain_upper": json.dumps(
                    self.backlash_snapshot.engaged_gain.upper.tolist()),
                "engaged_gain_update_count": json.dumps(
                    self.backlash_snapshot.engaged_gain.update_count.tolist()),
                "engaged_gain_status": json.dumps(
                    self.backlash_snapshot.engaged_gain.status),
                "engaged_gain_last_reason": json.dumps(
                    self.backlash_snapshot.engaged_gain.last_reason),
                "takeup_transaction_enabled": str(
                    self.takeup_transaction_enabled),
                "takeup_transaction_state": self.takeup_arbiter.state,
                "takeup_transaction_generation": str(
                    self.takeup_arbiter.generation),
                "takeup_transaction_active_mask": json.dumps(
                    self.takeup_arbiter.active_mask.astype(int).tolist()),
                "takeup_transaction_pending_mask": json.dumps(
                    self.takeup_arbiter.pending_mask.astype(int).tolist()),
                "takeup_transaction_direction": json.dumps(
                    self.takeup_arbiter.direction.tolist()),
                "takeup_transaction_saturated_mask": json.dumps(
                    self.takeup_arbiter.saturated_mask.astype(int).tolist()),
                "takeup_transaction_leakage_mask": json.dumps(
                    self.takeup_arbiter.leakage_mask.astype(int).tolist()),
                "takeup_requested_motor_rad_s": json.dumps(
                    self.takeup_arbiter
                    .requested_motor_radians_per_second.tolist()),
                "takeup_realized_motor_rad_s": json.dumps(
                    self.takeup_arbiter
                    .realized_motor_radians_per_second.tolist()),
                "reversal_scheduler_enabled": str(
                    self.reversal_scheduler.config.enabled),
                "reversal_scheduler_gating_active": str(
                    self.reversal_scheduler.config.enabled
                    and not self.planner.config.grouped_mode_sampling),
                "mppi_grouped_mode_sampling": str(
                    self.planner.config.grouped_mode_sampling),
                "mppi_best_candidate_guard": str(
                    self.planner.config.best_candidate_guard),
                "reversal_mode_selector": (
                    "grouped_mppi" if self.planner.config.grouped_mode_sampling
                    else ("legacy_reversal_scheduler"
                          if self.reversal_scheduler.config.enabled
                          else "plain_mppi")),
                "mppi_samples": str(self.planner.config.samples),
                "mppi_engaged_gain_scenarios": str(
                    self.planner.config.engaged_gain_scenarios),
                "mppi_engaged_gain_risk_beta": format(
                    self.planner.config.engaged_gain_risk_beta, ".6g"),
                "mppi_engaged_gain_cvar_alpha": format(
                    self.planner.config.engaged_gain_cvar_alpha, ".6g"),
                "mppi_engaged_gain_maximum_first_step_shift": format(
                    self.planner.config
                    .engaged_gain_maximum_first_step_shift, ".6g"),
                "mppi_engaged_gain_learning_velocity_scale": format(
                    self.planner.config
                    .engaged_gain_learning_velocity_scale, ".6g"),
                "mppi_capture_radius_mm": format(
                    self.planner.config.capture_radius_mm, ".6g"),
                "mppi_capture_minimum_terminal_improvement_mm": format(
                    self.planner.config
                    .capture_minimum_terminal_improvement_mm, ".6g"),
                "mppi_capture_hold_s": format(
                    self.planner.config.capture_hold_s, ".6g"),
                "mppi_capture_response_minimum_prediction_mm": format(
                    self.planner.config
                    .capture_response_minimum_prediction_mm, ".6g"),
                "mppi_capture_response_minimum_ratio": format(
                    self.planner.config.capture_response_minimum_ratio, ".6g"),
                "capture_passive_response_scale": format(
                    capture_diagnostics["passive_response_scale"], ".6g"),
                "capture_last_response_ratio": format(
                    capture_diagnostics["last_response_ratio"], ".6g"),
                "capture_last_response_reason": str(
                    capture_diagnostics["last_response_reason"]),
                "capture_release_count": str(
                    capture_diagnostics["release_count"]),
                "capture_rearm_blocked": str(
                    capture_diagnostics["rearm_blocked"]),
                "mppi_point_rollout_step_s": format(
                    self.planner.config.point_rollout_step_s, ".6g"),
                "mppi_point_rollout_coarse_steps": str(
                    self.planner.config.point_rollout_coarse_steps).lower(),
                "mppi_path_rollout_coarse_steps": str(
                    self.planner.config.path_rollout_coarse_steps).lower(),
                "mppi_point_prediction_tail_steps": str(
                    self.planner.config.point_prediction_tail_steps),
                "mppi_point_prediction_tail_step_s": format(
                    self.planner.config.point_prediction_tail_step_s, ".6g"),
                "mppi_takeup_risk_cost_weight": format(
                    self.planner.config.takeup_risk_weight, ".6g"),
                "mppi_takeup_confirmation_time_s": format(
                    self.planner.config.takeup_confirmation_time_s, ".6g"),
                "takeup_confirmation_hold_timeout_s": format(
                    self.takeup_arbiter.confirmation_hold_timeout_s, ".6g"),
                "mppi_active_proposal_groups": str(
                    1 << int(np.count_nonzero(
                        self.reversal_scheduler.lease_direction))
                    if self.planner.config.grouped_mode_sampling else 1),
                "mppi_minimum_samples_per_active_group": str(
                    self.planner.config.samples // (
                        1 << int(np.count_nonzero(
                            self.reversal_scheduler.lease_direction)))
                    if self.planner.config.grouped_mode_sampling else
                    self.planner.config.samples),
                "reversal_lease_direction": json.dumps(
                    self.reversal_scheduler.lease_direction.tolist()),
                "reversal_pending_direction": json.dumps(
                    self.reversal_scheduler.pending_direction.tolist()),
                "reversal_pending_count": json.dumps(
                    self.reversal_scheduler.pending_count.tolist()),
                "reversal_approved_direction": json.dumps(
                    self.reversal_scheduler.approved_direction.tolist()),
                "reversal_scheduler_reason": (
                    self.reversal_scheduler.reason),
                "reversal_absolute_cost_improvement": format(
                    self.reversal_scheduler.last_absolute_improvement,
                    ".6g"),
                "reversal_fractional_cost_improvement": format(
                    self.reversal_scheduler.last_fractional_improvement,
                    ".6g"),
                "reversal_terminal_error_improvement_mm": json.dumps(
                    self.reversal_scheduler.last_terminal_improvement_mm
                    .tolist()),
                "planner_blocked_motor_direction": json.dumps(
                    self.blocked_motor_direction.tolist()),
                "takeup_saturation_position": (
                    "none" if self.takeup_saturation_position is None else
                    json.dumps(
                        self.takeup_saturation_position.tolist(),
                        separators=(",", ":"))),
                "takeup_saturation_position_timestamp_ns": (
                    "none" if self.takeup_saturation_position_timestamp_ns
                    is None else str(
                        self.takeup_saturation_position_timestamp_ns)),
                "takeup_saturation_release_reason": (
                    self.takeup_saturation_release_reason),
                "position_feedback_valid": str(self.position_valid),
                "encoder_feedback_valid": str(self.encoder_valid),
                "estimator_runtime_owner": "single_timer",
                "planner_state_exchange": "replace_only_snapshot",
                "planner_snapshot_age_ms": (
                    "none" if self._planner_snapshot_source_time is None
                    else f"{1e3*max(0.0, now-self._planner_snapshot_source_time):.3f}"),
                "torch_intraop_threads": str(self.torch_intraop_threads),
                "torch_interop_threads": str(self.torch_interop_threads),
                "controller_velocity_min": json.dumps(
                    self.contract.velocity_min.tolist(), separators=(",", ":")),
                "controller_velocity_max": json.dumps(
                    self.contract.velocity_max.tolist(), separators=(",", ":")),
                "raw_response_during_interface_takeup": json.dumps(
                    self.raw_response_during_interface_takeup.tolist(),
                    separators=(",", ":")),
                "takeup_response_free_mask": json.dumps(
                    self.takeup_response_free_mask.tolist(),
                    separators=(",", ":")),
            }
            values.update(compute_device_diagnostics(self.compute_device))
            values.update({
                key: (str(value) if isinstance(value, int)
                      else f"{value:.6g}")
                for key, value in self._timing.snapshot_and_reset().items()
            })
            if self.last_plan is not None:
                values.update({
                    "plan_elapsed_ms": f"{1e3*self.last_plan.elapsed_s:.3f}",
                    "best_cost": f"{self.last_plan.best_cost:.6g}",
                    "effective_samples": (
                        f"{self.last_plan.effective_samples:.3f}"),
                    "plan_sample_projection_ms": (
                        f"{self.last_plan.sample_projection_ms:.3f}"),
                    "plan_rollout_ms": f"{self.last_plan.rollout_ms:.3f}",
                    "plan_engaged_gain_scenario_count": str(
                        self.last_plan.engaged_gain_scenario_count),
                    "plan_selected_engaged_gain_scenarios": json.dumps(
                        self.last_plan.selected_engaged_gain_scenarios.tolist()),
                    "plan_selected_gain_tracking_costs": json.dumps(
                        self.last_plan.selected_gain_tracking_costs.tolist()),
                    "plan_selected_maximum_first_step_lambda_shift": format(
                        self.last_plan
                        .selected_maximum_first_step_lambda_shift, ".6g"),
                    "plan_cost_weighting_ms": (
                        f"{self.last_plan.cost_weighting_ms:.3f}"),
                    "plan_update_projection_ms": (
                        f"{self.last_plan.update_projection_ms:.3f}"),
                    "plan_command_prediction_kind": (
                        "scored_feasible_candidate"
                        if self.last_plan.scored_candidate_guard_applied else
                        "weighted_feasible_candidate_mean"),
                    "plan_transmission_prediction_applied": str(
                        self.last_plan.transmission_prediction_applied),
                    "plan_rotation_direction_latched": str(
                        self.last_plan.rotation_direction_latched),
                    "plan_takeup_direction_latched": str(
                        self.last_plan.takeup_direction_latched),
                    "plan_best_candidate_selected": str(
                        self.last_plan.best_candidate_selected),
                    "plan_scored_candidate_guard_applied": str(
                        self.last_plan.scored_candidate_guard_applied),
                    "plan_blocked_motor_direction": (
                        "none" if self.last_plan.blocked_motor_direction
                        is None else json.dumps(
                            self.last_plan.blocked_motor_direction.tolist())),
                    "plan_blocked_candidate_count": str(
                        self.last_plan.blocked_candidate_count),
                    "plan_direction_lease": json.dumps(
                        self.last_plan.direction_lease.tolist()),
                    "plan_approved_reversal_direction": json.dumps(
                        self.last_plan.approved_reversal_direction.tolist()),
                    "plan_proposed_reversal_direction": json.dumps(
                        self.last_plan.proposed_reversal_direction.tolist()),
                    "plan_direction_lease_applied": str(
                        self.last_plan.direction_lease_applied),
                    "plan_unrestricted_candidate_index": str(
                        self.last_plan.unrestricted_candidate_index),
                    "plan_lease_constrained_candidate_index": str(
                        self.last_plan.lease_constrained_candidate_index),
                    "plan_unrestricted_total_cost": format(
                        self.last_plan.unrestricted_total_cost, ".6g"),
                    "plan_lease_constrained_total_cost": format(
                        self.last_plan.lease_constrained_total_cost, ".6g"),
                    "plan_reversal_axis_cost_improvement": json.dumps(
                        self.last_plan.reversal_axis_cost_improvement.tolist()),
                    "plan_reversal_axis_terminal_error_improvement_mm": (
                        json.dumps(
                            self.last_plan
                            .reversal_axis_terminal_error_improvement_mm
                            .tolist())),
                    "plan_hold_branch_applied": str(
                        self.last_plan.hold_branch_applied),
                    "plan_hold_branch_terminal_error_mm": format(
                        self.last_plan.hold_branch_terminal_error_mm, ".6g"),
                    "plan_zero_terminal_error_mm": format(
                        self.last_plan.zero_terminal_error_mm, ".6g"),
                    "plan_raw_zero_terminal_error_mm": format(
                        self.last_plan.raw_zero_terminal_error_mm, ".6g"),
                    "plan_capture_passive_response_scale": format(
                        self.last_plan.capture_passive_response_scale, ".6g"),
                    "plan_selected_gain_learning_velocity_scale": format(
                        self.last_plan
                        .selected_gain_learning_velocity_scale, ".6g"),
                    "plan_proposal_group_count": str(
                        self.last_plan.proposal_group_count),
                    "plan_prediction_horizon_steps": str(
                        self.last_plan.prediction_horizon_steps),
                    "plan_prediction_horizon_s": format(
                        self.last_plan.prediction_horizon_s, ".6g"),
                    "plan_tendon_probe_candidate_count": str(
                        self.last_plan.tendon_probe_candidate_count),
                    "plan_selected_tendon_probe": str(
                        self.last_plan.selected_tendon_probe),
                    "plan_selected_reversal_mask": str(
                        self.last_plan.selected_reversal_mask),
                    "plan_unrestricted_reversal_mask": str(
                        self.last_plan.unrestricted_reversal_mask),
                    "plan_mode_best_total_cost": json.dumps(
                        self.last_plan.mode_best_total_cost.tolist(),
                        separators=(",", ":")),
                    "plan_takeup_joint_position_offset": json.dumps(
                        self.last_plan.takeup_joint_position_offset.tolist(),
                        separators=(",", ":")),
                    "plan_selected_takeup_risk_s": format(
                        self.last_plan.selected_takeup_risk_s, ".6g"),
                    "plan_selected_takeup_risk_cost": format(
                        self.last_plan.selected_takeup_risk_cost, ".6g"),
                    "plan_selected_switch_count": str(
                        self.last_plan.selected_switch_count),
                    "plan_selected_candidate_index": str(
                        self.last_plan.selected_candidate_index),
                    "plan_selected_total_cost": (
                        f"{self.last_plan.selected_total_cost:.6g}"),
                    "plan_zero_total_cost": (
                        f"{self.last_plan.zero_total_cost:.6g}"),
                    "plan_weighted_tracking_cost": (
                        f"{self.last_plan.weighted_tracking_cost:.6g}"),
                    "plan_zero_tracking_cost": (
                        f"{self.last_plan.zero_tracking_cost:.6g}"),
                    "plan_best_tracking_cost": (
                        f"{self.last_plan.best_tracking_cost:.6g}"),
                    "plan_logical_velocity_sequence": json.dumps(
                        self.last_plan.logical_velocity_sequence.tolist(),
                        separators=(",", ":")),
                    "plan_motor_radians_per_second_sequence": json.dumps(
                        self.last_plan.motor_radians_per_second_sequence
                        .tolist(),
                        separators=(",", ":")),
                    "plan_compensated_motor_radians_per_second_sequence": (
                        "none" if self.last_plan
                        .compensated_motor_radians_per_second_sequence is None
                        else json.dumps(
                            self.last_plan
                            .compensated_motor_radians_per_second_sequence
                            .tolist(), separators=(",", ":"))),
                    "plan_transmitted_motor_radians_per_second_sequence": (
                        "none" if self.last_plan
                        .transmitted_motor_radians_per_second_sequence is None
                        else json.dumps(
                            self.last_plan
                            .transmitted_motor_radians_per_second_sequence
                            .tolist(), separators=(",", ":"))),
                })
                if self.last_plan.command_tip_sequence_m is not None:
                    terminal = self.last_plan.command_tip_sequence_m[-1]
                    values["plan_command_predicted_terminal_tip_m"] = (
                        json.dumps(terminal.tolist(), separators=(",", ":")))
                    if self.target is not None:
                        _, terminal_error = tip_tracking_error_mm(
                            self.target, terminal)
                        values[
                            "plan_command_predicted_terminal_error_mm"] = (
                                f"{terminal_error:.6g}")
            response = self.last_tip_forecast_result
            if response is not None:
                values.update({
                    "response_forecast_start_timestamp_ns": str(
                        response.start_timestamp_ns),
                    "response_forecast_due_timestamp_ns": str(
                        response.due_timestamp_ns),
                    "response_forecast_horizon_ms": (
                        f"{1e-6*(response.due_timestamp_ns-response.start_timestamp_ns):.6g}"),
                    "response_observation_timestamp_ns": str(
                        response.observation_timestamp_ns),
                    "response_observation_lateness_ms": (
                        "{:.6g}".format(1e-6*(
                            response.observation_timestamp_ns
                            - response.due_timestamp_ns))),
                    "response_start_observation_skew_ms": (
                        f"{response.start_observation_skew_ms:.6g}"),
                    "response_predicted_tip_delta_mm": json.dumps(
                        response.predicted_delta_mm.tolist(),
                        separators=(",", ":")),
                    "response_measured_tip_delta_mm": json.dumps(
                        response.measured_delta_mm.tolist(),
                        separators=(",", ":")),
                    "response_endpoint_error_xyz_mm": json.dumps(
                        response.endpoint_error_mm.tolist(),
                        separators=(",", ":")),
                    "response_endpoint_error_norm_mm": (
                        f"{response.endpoint_error_norm_mm:.6g}"),
                    "response_direction_cosine": (
                        "none" if response.direction_cosine is None
                        else f"{response.direction_cosine:.6g}"),
                    "response_pending_forecasts": str(
                        self.tip_forecast_monitor.pending_count),
                })
            for key in (
                    "distal_sha256", "jacobian_sha256", "lambda",
                    "last_dt_s",
                    "estimator_covariance_trace",
                    "estimator_covariance_min_eigenvalue",
                    "estimator_covariance_max_eigenvalue",
                    "estimator_observable_rank",
                    "marker_timing_rewind_ms",
                    "marker_timing_correction_ms",
                    "marker_timing_replay_ms",
                    "marker_timing_total_ms",
                    "rls_covariance_trace",
                    "rls_covariance_min_eigenvalue",
                    "rls_covariance_max_eigenvalue", "rls_weight", "rls_axis",
                    "rls_update_norm", "rls_reason", "rls_window_frames",
                    "rls_motion_intervals", "rls_normalized_action_norm",
                    "rls_directional_purity",
                    "rls_response_rotation_deg",
                    "rls_response_translation_mm",
                    "rls_rotation_snr", "rls_translation_snr",
                    "rls_pending_confirmation",
                    "rls_reversal_holdoff_axis",
                    "adaptation_reversal_holdoff_normalized_action_by_axis",
                    "adaptation_enabled",
                    "adaptation_minimum_observations",
                    "adaptation_minimum_normalized_action",
                    "adaptation_minimum_rotation_deg",
                    "adaptation_minimum_translation_mm",
                    "adaptation_minimum_response_snr",
                    "estimator_history_reconciliation_enabled",
                    "history_equilibrium_correction",
                    "history_reconciliation_last_delta",
                    "jacobian_condition_number",
                    "rewind_entries"):
                if key in self.model_diagnostics:
                    values[f"model_{key}"] = str(
                        self.model_diagnostics[key])
            for key in ("initialization_inlier_streak",
                        "initialization_complete"):
                if key in self.model_diagnostics:
                    values[f"model_{key}"] = str(
                        self.model_diagnostics[key])
            if "jacobian" in self.model_diagnostics:
                values["model_jacobian"] = json.dumps(
                    self.model_diagnostics["jacobian"], separators=(",", ":"))
            for key in ("raw_encoder_counts_first_three", "raw_motor_angle_rad",
                        "motor_angle_rad", "downstream"):
                if key in self.model_diagnostics:
                    values[f"model_{key}"] = json.dumps(
                        self.model_diagnostics[key], separators=(",", ":"))
            values["model_valid"] = str(self.model_valid)
            status.values = [
                KeyValue(key=key, value=value)
                for key, value in values.items()]
            report.status = [status]
            self.status_pub.publish(report)
            if (self.state == ControllerState.ACTIVE
                    and tip_error_xyz_mm is not None
                    and self.tip_error_log_period_s is not None
                    and (self._last_tip_error_log_time is None
                         or now-self._last_tip_error_log_time
                         >= self.tip_error_log_period_s)):
                self._last_tip_error_log_time = now
                tracking_log_message = (
                    "tip tracking error %.3f mm; target-observed xyz="
                    "[%.3f, %.3f, %.3f] mm" % (
                        tip_error_norm_mm, *tip_error_xyz_mm))
                response = self.last_tip_forecast_result
                if response is not None:
                    cosine = ("none" if response.direction_cosine is None
                              else f"{response.direction_cosine:.3f}")
                    tracking_log_message += (
                        "; horizon response predicted=[%.3f, %.3f, %.3f] "
                        "measured=[%.3f, %.3f, %.3f] mm, endpoint-error="
                        "%.3f mm, direction-cosine=%s" % (
                            *response.predicted_delta_mm,
                            *response.measured_delta_mm,
                            response.endpoint_error_norm_mm, cosine))
        finally:
            self._lock.release()
        if tracking_log_message is not None:
            self.get_logger().info(tracking_log_message)

    def shutdown(self):
        with self._lock:
            self._release_locked()
            self.armed = False
            self.state = ControllerState.DISARMED
            self.state_reason = "shutdown"


def main(args=None):
    rclpy.init(args=args)
    node = None
    # Leave capacity for control services even if sensor, plan, heartbeat, and
    # status callbacks become ready together.
    executor = MultiThreadedExecutor(num_threads=8)
    try:
        node = CatheterControlNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
