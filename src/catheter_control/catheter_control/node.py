"""Guarded ROS 2 wrapper for the streaming catheter model and MPPI."""
from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import replace
import json
import math
import threading
import time

from control_interface.msg import (
    ControlCycleTiming, ControlStream, DeviceStream, EstimatorStateTrace,
    ManagerEvent, MppiResponseTrace, MppiAlignedResponseTrace, TipReferenceHorizon)
from control_interface.srv import GenerateSparseTargets, OpenPositionStream
from control_interface.msg import StreamedPositionControl
from .safety.position_execution import make_position_execution, root_encoder_counts, position_execution_in_flight, measured_standstill
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

from .orchestration.causal_schedule import CausalMarkerSchedule
from .transmission.backlash import (
    BacklashConfig, BacklashFeedforwardCompensator,
    BacklashStateEstimator, TakeupTransactionArbiter)
from .orchestration.compute_device import (
    compute_device_diagnostics, resolve_compute_device)
from .orchestration.diagnostics import (
    instrument_timer, marker_diagnostic_values, model_diagnostic_values,
    plan_diagnostic_values, planner_policy_diagnostic_values,
    response_diagnostic_values, runtime_diagnostic_values,
    transmission_diagnostic_values)
from .orchestration.estimator_owner import marker_measurement, stamp_ns
from .orchestration.parameters import declare_parameters
from .orchestration.runtime import load_runtime_pair
from .orchestration.planner_process import ProcessPlanner
from .orchestration.reduced_filter_owner import OrderedInbox, ReducedFilterOwner
from .orchestration.reduced_filter_settings import (
    load_reduced_filter, plan_due, reduced_filter_contract, reduced_planner_config,
    require_filter_owned_switches_off)
from .transmission.engaged_gain import EngagedGainConfig
from .safety.hardware_contract import ENCODER_RADIANS_PER_COUNT, load_hardware_contract
from .safety.lifecycle import (
    ControllerState, FreshnessLimits, GateInputs, paired_source_skew_s,
    readiness, recoverable_runtime_interruption)
from .planning.mppi import CatheterMppi, MppiConfig
from .planning.engagement import reconcile_engagement, tendon_confirmation, confirmation_from_checkpoint, confirm_at_source
from .orchestration.configuration import validate_experimental_composition
from .planning.path_tracking import ReferenceHorizon
from .transmission.reversal_scheduler import (
    ReversalDirectionScheduler, ReversalSchedulerConfig)
from .orchestration.timing import PeriodicTimerProbe, TimingWindows
from .orchestration.task_gc import TaskGcPolicy, gc_work
from .planning.tracking import TipForecastMonitor, tip_tracking_error_mm


SOURCE_NAME = "catheter_mppi"
# Commanding reduced-filter backend: the filter owns estimation, play and response; this node keeps the
# command heartbeat, arming, mode claim, gates and fault handling (user-approved design, 2026-11-03).
REDUCED_BACKEND = "reduced_filter"
# Owner health -> readiness-gate health. HOLDING is a transient rejection: the gates tolerate it while the
# root refuses, so that cycle commands zero; a persisting hold fails marker freshness into the bounded pause.
REDUCED_GATE_HEALTH = {"TRACKING": "TRACKING", "HOLDING": "DEGRADED"}
REDUCED_OWNER_PERIOD_S = 0.005


class CatheterControlNode(Node):
    """Guarded closed-loop controller; hardware safety stays upstream."""

    def __init__(self):
        super().__init__("catheter_mppi")
        declare_parameters(self, source_name=SOURCE_NAME)
        backend = str(self.get_parameter("estimator_backend").value)
        if backend not in ("v171", REDUCED_BACKEND):
            raise ValueError(
                f"this controller runs estimator_backend=v171 or {REDUCED_BACKEND}; the "
                "reduced-filter dry run has its own node class")
        self.reduced_filter = backend == REDUCED_BACKEND
        self.source = str(self.get_parameter("source_name").value)
        if self.source != SOURCE_NAME:
            raise ValueError(f"source_name must be {SOURCE_NAME!r}")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.command_output_enabled = bool(
            self.get_parameter("command_output_enabled").value)
        self.control_method = str(self.get_parameter("control_method").value)
        self.mppi_execution_mode = str(self.get_parameter("mppi_execution_mode").value)
        if self.mppi_execution_mode not in {'velocity', 'position_stream'}:
            raise ValueError('mppi_execution_mode must be velocity or position_stream')
        self.position_execution = None
        self.position_stream_session = 0
        self.position_stream_future = None
        self.position_stream_open_started = None
        self.position_stream_command_id = 0
        self.position_stream_encoder_counts = None
        if self.control_method not in {"mppi", "adapj"}:
            raise ValueError("control_method must be mppi or adapj")
        self.engagement_control_enabled = bool(self.get_parameter("engagement_control_enabled").value)
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
        if self.reduced_filter:
            require_filter_owned_switches_off(
                lambda name: self.get_parameter(name).value)
            if (self.control_method != "mppi"
                    or self.mppi_execution_mode != "velocity"):
                raise ValueError(
                    "the reduced-filter backend runs MPPI velocity control only")

        self._steady = time.monotonic
        self._lock = threading.RLock()
        self._snapshot_lock = threading.Lock()
        self._plan_lock = threading.Lock()
        self._task_gc = TaskGcPolicy(
            enabled=bool(self.get_parameter("task_gc_enabled").value),
            maximum_s=float(self.get_parameter("task_gc_maximum_s").value),
            growth_mb=float(self.get_parameter("task_gc_growth_mb").value))
        self._timing = TimingWindows()
        self._timer_probes = {}
        self.state = ControllerState.DISARMED
        self.state_reason = "not_armed"
        self.armed = False
        self._shutting_down = False
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
        # Full input cadence for AdapJ alignment/standstill, independent of the
        # latest-only estimator owner. All access is under the control lock.
        self._adapj_encoder_history = deque(maxlen=1024)
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
        self._planner_warmup_supersessions = 0
        self.consecutive_deadline_misses = 0
        self.estimator_catchup_started_at = None
        self.estimator_catchup_observations = 0
        self.estimator_catchup_pause_count = 0
        self.model_diagnostics = {}
        self.model_valid = False
        self._planner_snapshot = None
        self._planner_snapshot_source_time = None
        self._tip_readout_offset_m = None
        self._planner_tip_readout_offset_m = None
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
                or not math.isfinite(self.estimator_catchup_timeout_s)
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
        requested_estimator = str(self.get_parameter("estimator_device").value).strip()
        self.estimator_compute_device = resolve_compute_device(
            requested_estimator or str(self.compute_device.device))

        runtime_options = dict(
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
        if self.reduced_filter:
            # The reduced filter replaces the v171 runtime pair; validity comes
            # only from the filter owner's health (see _reduced_owner_tick).
            self.runtime_bundle = self.planner_runtime_bundle = None
            self.runtime = self.planner_runtime = None
            self.reduced_backend, self.reduced_stream = load_reduced_filter(
                str(self.get_parameter("reduced_filter_artifact_root").value),
                float(self.get_parameter("reduced_filter_image_period_ms").value),
                int(self.get_parameter("reduced_filter_startup_pending_images").value),
                str(self.compute_device.device))
            self.model_diagnostics = {}
            self.model_valid = False
        else:
            self.runtime_bundle, self.planner_runtime_bundle = load_runtime_pair(
                str(self.get_parameter("model_manifest").value),
                estimator_device=str(self.estimator_compute_device.device),
                estimator_dtype=str(self.get_parameter("estimator_dtype").value).strip() or "float32",
                planner_device=str(self.compute_device.device), options=runtime_options,
                experimental_engagement=self.engagement_control_enabled)
            if self.engagement_control_enabled:
                runtime_options = {}
            self.runtime = self.runtime_bundle.runtime
            self.planner_runtime = self.planner_runtime_bundle.runtime
            if self.engagement_control_enabled:
                self.runtime.use_external_tendon_confirmation()
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
            velocity_max=resolved_velocity_max,
            synchronized_sheath_rotation=bool(self.get_parameter(
                "synchronized_sheath_rotation").value))
        if self.reduced_filter:
            # Same controller-only limits as the dry run: no rotation, tendon
            # upper limit inside the filter's support (manager limits unchanged).
            self.contract = reduced_filter_contract(
                self.contract, controller_velocity_max, float(
                    self.get_parameter("reduced_filter_tendon_upper_mm").value))
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
            fitted_width = tuple(float(value) for value in (
                interface_artifacts.symmetric_reversal_width_rad.detach()
                .cpu().double().tolist()))
            if self.backlash_compensation_enabled:
                symmetric_backlash_width = fitted_width
                positive_backlash_width = (0.0, 0.0, 0.0)
                negative_backlash_width = (0.0, 0.0, 0.0)
                directional_widths_present = False
            self.get_logger().info(
                "v175 interface transmission loaded: symmetric shaft "
                f"take-up={list(round(value, 6) for value in fitted_width)} "
                "rad; external compensation="
                f"{self.backlash_compensation_enabled}")
        if self.engagement_control_enabled and self.runtime.proximal is not None:
            transmission = self.runtime.proximal.transmission
            symmetric_backlash_width = tuple(((transmission.positive_play+transmission.negative_play)
                /transmission.axis_scale.abs()).cpu().tolist())
            directional_widths_present = False
        tendon_travel_limit_rad = 0.0
        if bool(self.get_parameter(
                "backlash_tendon_takeup_joint_range_enabled").value):
            # Only the tendon acquisition budget changes. Projection still
            # enforces the directional headroom from the current position,
            # including knob/chassis coupling and encoder-count limits.
            tendon_span = self.contract.position_upper[2] - self.contract.position_lower[2]
            tendon_mm_per_rad = abs(self.contract.motor_radians_per_second_to_motor_axis_velocity(
                np.ones(6))[2])
            tendon_travel_limit_rad = float(tendon_span / tendon_mm_per_rad)
            self.get_logger().warn(
                f"Experimental tendon take-up joint-range budget: {tendon_span:.3f} mm "
                f"({tendon_travel_limit_rad:.3f} rad); joint limits remain enforced")
        backlash_config = BacklashConfig(
            tendon_unconfirmed_travel_limit_rad=tendon_travel_limit_rad,
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
            response_window_max_s=float(self.get_parameter(
                "backlash_response_window_max_s").value),
            minimum_distal_bending_increment=float(self.get_parameter(
                "backlash_minimum_distal_bending_increment").value),
            minimum_distal_marker_deformation_mm=float(self.get_parameter(
                "backlash_minimum_distal_marker_deformation_mm").value),
            distal_confirmation_release_ratio=float(self.get_parameter(
                "backlash_distal_confirmation_release_ratio").value),
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
        if self.mppi_execution_mode == 'position_stream':
            if self.backlash_compensation_enabled and not self.takeup_transaction_enabled:
                raise ValueError('position_stream requires explicit takeup transactions, not legacy feedforward')
            if self.contract.synchronized_sheath_rotation:
                raise ValueError('position_stream synchronized rotation needs separate paired-endpoint qualification')
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
            self.reduced_backend, self.contract, reduced_planner_config(self)
        ) if self.reduced_filter else CatheterMppi(
            self.planner_runtime, self.contract,
            MppiConfig(
                always_engaged_rollout=(self.takeup_transaction_enabled
                                       and not self.engagement_control_enabled),
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
                temperature=float(self.get_parameter("mppi_temperature").value),
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
        self.planner_process_enabled = bool(
            self.get_parameter("planner_process_enabled").value)
        if self.engagement_control_enabled:
            if not self.takeup_transaction_enabled:
                raise ValueError('G5 requires takeup compensation')
            validate_experimental_composition(self.runtime,self.planner,self.takeup_arbiter,
                isolated=self.planner_process_enabled,planner_device=self.compute_device.device,
                marker_estimator=self.marker_estimator,gain_enabled=self.engaged_gain_config.enabled,
                scheduler_enabled=self.reversal_scheduler.config.enabled)
        if self.planner_process_enabled:
            if self.runtime.device.type != "cpu":
                raise ValueError("planner_process_enabled requires estimator_device=cpu")
            self.planner = ProcessPlanner(
                self.planner_runtime, self.contract, self.planner.config,
                manifest=str(self.get_parameter("model_manifest").value),
                device=str(self.compute_device.device), options=runtime_options,
                identity=self.runtime_bundle.identity.manifest_sha256,
                threads=(self.torch_intraop_threads, self.torch_interop_threads))
        self.reset_pending = False
        if self.reduced_filter:
            self.reduced_owner = ReducedFilterOwner(
                self.reduced_stream, self.reduced_backend,
                candidate_counts=self.planner.candidate_counts(),
                step_s=self.planner.config.point_rollout_step_s, clock=time.perf_counter)
            self.reduced_inbox = OrderedInbox()
            # Committed image of the last plan; plan_due runs on image stamps.
            self.reduced_last_plan_image_ns = None
            # Set when a tick reaches the planner, so a skipped tick keeps the due image.
            self.reduced_plan_attempted = False

        self.adapj = None
        if self.control_method == "adapj":
            if self.engagement_control_enabled or self.planner_process_enabled:
                raise ValueError("AdapJ requires baseline and local execution")
            if self.takeup_transaction_enabled != self.backlash_compensation_enabled:
                raise ValueError("AdapJ compensation requires the shared takeup transaction")
            from .planning.adapj import TipAdapJ
            self.adapj = TipAdapJ(self.contract,
                1./float(self.get_parameter("plan_rate_hz").value),
                float(self.get_parameter("adapj_reference_step_mm").value),
                exclude_takeup_from_initialization=self.takeup_transaction_enabled,
                position_mode=self.mppi_execution_mode == 'position_stream')
            if self.mppi_execution_mode == 'position_stream' and self.adapj.step_s > .25:
                raise ValueError('AdapJ position_stream interval must not exceed 250 ms')

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
        self.plan_rate_hz = plan_rate
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
        # A planned command must outlive the plan period, or the heartbeat
        # latches planned_command_stale between consecutive plans.
        if self.command_timeout_s <= 1.0/plan_rate:
            raise ValueError(
                "command_timeout_s must exceed the plan period")
        self._marker_schedule = CausalMarkerSchedule(
            marker_rate, self._steady())
        timer_started = self._steady()
        if self.reduced_filter:
            # One 5 ms owner callback holds the filter and plans right after a
            # committed image (commit-triggered); there is no plan timer.
            self._timer_probes = {
                "estimator": PeriodicTimerProbe(REDUCED_OWNER_PERIOD_S, timer_started),
                "heartbeat": PeriodicTimerProbe(
                    1.0/command_rate, timer_started),
            }
            self.estimator_timer = self.create_timer(
                REDUCED_OWNER_PERIOD_S, self._reduced_owner_tick,
                callback_group=self.estimator_group)
            self.plan_timer = None
        else:
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
        self.gc_maintenance_timer = self.create_timer(
            1.0, self._gc_maintenance_tick,
            callback_group=MutuallyExclusiveCallbackGroup())
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

    def _create_ros_interfaces(self):
        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        motion_qos = QoSProfile(depth=1)
        motion_qos.reliability = ReliabilityPolicy.RELIABLE
        motion_qos.durability = DurabilityPolicy.VOLATILE
        self.control_pub = self.create_publisher(
            ControlStream, "/teleop/control", motion_qos)
        self.position_control_pub = self.create_publisher(
            StreamedPositionControl, '/teleop/position_control', motion_qos)
        self.position_stream_client = self.create_client(
            OpenPositionStream, '/manager/open_position_stream', callback_group=self.service_group)
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
        self.aligned_response_trace_pub = self.create_publisher(
            MppiAlignedResponseTrace, "/catheter_mppi/aligned_response_trace", trace_qos)
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
        if self.adapj is not None:
            self.create_service(SetBool, "/catheter_mppi/adapj_initialization",
                self._adapj_initialization_cb, callback_group=self.service_group)
            self.create_service(Trigger, "/catheter_mppi/adapj_cancel_initialization",
                self._adapj_cancel_initialization_cb, callback_group=self.service_group)
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
            root = self._planner_root_on_device(root)
            prediction = self.planner.predict_sparse_targets(
                root, position, observed_tip, flat.reshape(-1, 3),
                int(request.rollout_steps),
                np.asarray(request.minimum_endpoint_reserve,
                           dtype=np.float64),
                transmission_state=transmission,
                reserve_takeup=not self.engagement_control_enabled,
                enforce_endpoint_reserve=not self.engagement_control_enabled)
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
                warning = (
                    "model target displacement outside requested bounds: "
                    + ",".join(
                        f"{index}={displacement_mm[index]:.3f}mm"
                        for index in outside))
                if not self.engagement_control_enabled:
                    raise ValueError(warning)
                self.get_logger().warn(warning + "; advisory G5 preview only")
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
            if self.engagement_control_enabled:
                response.message += (
                    "; hypothetical post-engagement shape preview; "
                    "model reachability bounds advisory; "
                    "takeup execution feasibility not certified")
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
        timestamp = stamp_ns(message.header)
        return timestamp if timestamp > 0 else self._now_ns()

    def _record_header_age(self, name: str, message):
        timestamp_ns = stamp_ns(message.header)
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
                # advanced only after estimator processing and remains MPPI's
                # processed-feedback freshness signal. AdapJ consumes raw ENC.
                previous_source_timestamp_ns = (
                    self.encoder_source_timestamp_ns)
                if (previous_source_timestamp_ns is None
                        or timestamp_ns >= previous_source_timestamp_ns):
                    self.encoder_source_timestamp_ns = timestamp_ns
                if (previous_source_timestamp_ns is None
                        or timestamp_ns > previous_source_timestamp_ns):
                    self.encoder_receive_time = now
                    if getattr(self, 'adapj', None) is not None:
                        if np.any(counts != np.rint(counts)) or np.any(np.abs(counts) > 2147483647):
                            self._fault_locked('malformed_adapj_encoder_feedback')
                            return
                        self._adapj_encoder_history.append((int(timestamp_ns), counts.copy()))
                        if self.mppi_execution_mode == 'position_stream':
                            self.position_stream_encoder_counts = counts.copy()
                            self.adapj.confirm_position_standstill(self._adapj_encoder_history)
        if not valid:
            self._fault_if_active("encoder_feedback_out_of_range")
            return
        if getattr(self, "reduced_filter", False):
            # The reduced filter consumes every encoder sample in arrival order;
            # encoder_time advances only when its owner absorbs one.
            self.reduced_inbox.put(("encoder", timestamp_ns, counts.tolist(), now))
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

    def _planner_root_on_device(self, root):
        """Complete blocking handoff outside owner/exchange locks, once per solve."""
        if (root.motor_angle_rad.device == self.planner_runtime.device
                and root.motor_angle_rad.dtype == self.planner_runtime.dtype):
            return root
        started = self._steady()
        try:
            if root.motor_angle_rad.dtype != self.planner_runtime.dtype:
                return root.clone_to(self.planner_runtime.device, dtype=self.planner_runtime.dtype)
            return root.clone_to(self.planner_runtime.device)
        finally:
            self._timing.record_seconds("plan_snapshot_transfer", self._steady()-started)

    def _publish_planner_snapshot(
            self, state, source_time, transmission_state=None):
        if state is None or source_time is None:
            return
        with self._timed_lock(self._snapshot_lock, "snapshot_publish"):
            self._planner_snapshot = state
            self._planner_snapshot_source_time = source_time
            self._planner_transmission_snapshot = transmission_state
            self._planner_tip_readout_offset_m = getattr(
                self, "_tip_readout_offset_m", None)

    def _process_encoder_sample(self, sample):
        if sample is None:
            return False
        timestamp_ns, counts, arrival = sample
        self._timing.record_seconds(
            "encoder_pending_age", self._steady()-arrival)
        started = self._steady()
        backlash_snapshot = None
        with self._lock:
            control_owned = bool(self.armed)
        try:
            if (self.runtime.state is not None
                    and timestamp_ns <= self.runtime.state.timestamp_ns):
                return False
            raw_counts = np.asarray(counts, dtype=np.float64).copy()
            self._raw_encoder_history.append(
                (int(timestamp_ns), raw_counts.copy()
                 if (self.engagement_control_enabled or getattr(self, "adapj", None) is not None
                     or getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream')
                 else raw_counts[:3].copy()))
            if (getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream'
                    and getattr(self, 'adapj', None) is None):
                self.position_stream_encoder_counts = raw_counts.copy()
            self.latest_raw_encoder_counts = raw_counts[:3].copy()
            effective_motor = None
            if self.backlash_compensation_enabled:
                raw_motor = (
                    raw_counts[:3]*ENCODER_RADIANS_PER_COUNT)
                advance_options = dict(external_motion=not control_owned)
                adapj = getattr(self, "adapj", None)
                if (not control_owned and adapj is not None and adapj.capturing
                        and adapj.exclude_takeup_from_initialization):
                    advance_options["track_external_response"] = True
                self.backlash_estimator.advance_motor(raw_motor, **advance_options)
                effective_motor = self.backlash_estimator.interface_motor_angles(raw_motor)
                self._backlash_history.append([
                    int(timestamp_ns), raw_motor.copy(),
                    self.backlash_estimator.clone_state(), control_owned])
                backlash_snapshot = self.backlash_estimator.snapshot()
            if self.engagement_control_enabled:
                effective_motor = None  # Joint runtime owns interface transmission.
                if self.runtime.state is None:
                    self.encoder_time = arrival
                    return False  # Initialization awaits causally bracketed markers.
            if self.runtime.state is None:
                state = self.runtime.initialize(
                    timestamp_ns, raw_counts,
                    interface_motor_angle_rad=effective_motor)
            else:
                if self.engagement_control_enabled:
                    self.runtime.advance_encoder(timestamp_ns,raw_counts,return_snapshot=False,
                        tendon_confirmation=tendon_confirmation(self.runtime,backlash_snapshot))
                    state = self.runtime.clone_state(include_history=False)
                else:
                    state = self.runtime.advance_encoder(timestamp_ns,raw_counts,
                        interface_motor_angle_rad=effective_motor)
            model_diagnostics = self.runtime.diagnostics()
            if self.engagement_control_enabled:
                backlash_snapshot = reconcile_engagement(self.runtime,backlash_snapshot,
                    backlash_config=self.backlash_config)
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
            nominal_distal_lambda=None, observed_distal_lambda=None, *,
            observed_interface_position_m, observed_marker_points_m):
        """Sole estimator-owner composition of shared source-time confirmation."""
        bending_mode = None if strain is None else (
            self.runtime.mechanics.mode if self.engagement_control_enabled else self.runtime.port.mode).detach().cpu().numpy()
        snapshot, history = self.backlash_estimator.observe_at_timestamp(
            self._backlash_history, timestamp_ns, interface_pose, model,
            control_owned=self.armed, observed_interface_position_m=observed_interface_position_m,
            observed_marker_points_m=observed_marker_points_m,
            distal_strain=strain, distal_bending_mode=bending_mode,
            nominal_distal_lambda=nominal_distal_lambda, observed_distal_lambda=observed_distal_lambda)
        self._backlash_history = deque(history, maxlen=256)
        return snapshot

    def _engagement_confirmation_at_source(self, timestamp_ns, estimator, points):
        """Use the shared policy before the model's existing source-time replay.

        The transient callback stays inside the sole estimator owner. It never
        enters a runtime checkpoint or planner IPC payload.
        """
        self.backlash_snapshot, history, decisions = confirm_at_source(self.runtime,
            self.backlash_estimator, self._backlash_history, timestamp_ns, estimator, points,
            control_owned=self.armed)
        self._backlash_history = deque(history, maxlen=256)
        return decisions

    def _marker_cb(self, message: PointCloud):
        try:
            timestamp_ns, points, quality = marker_measurement(
                message, self.frame_id)
        except ValueError as error:
            self._fault_if_active(str(error))
            return
        self._record_header_age("marker", message)
        arrival = self._steady()
        if getattr(self, "reduced_filter", False):
            self.reduced_inbox.put(("marker", timestamp_ns, points*1000.0, arrival))
            return
        quality = dict(quality, callback_arrival_ros_ns=self.get_clock().now().nanoseconds)
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
        if self.engagement_control_enabled and state_timestamp_ns is None and self._raw_encoder_history:
            state_timestamp_ns = self._raw_encoder_history[-1][0]
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
            self._stop_on_raw_tendon_onset(timestamp_ns, points, quality, arrival)
            if self.engagement_control_enabled:
                quality = dict(visible=(np.asarray(quality['confidence']) > 0)
                    & (np.asarray(quality['source_rig_count']) >= 1)
                    & (np.asarray(quality['reprojection_error_px']) >= 0),
                    point_std_m=np.full(4,self.runtime.config.point_std_m))
                if self.runtime.state is None:
                    earlier = [item for item in self._raw_encoder_history if item[0] <= timestamp_ns]
                    later = [item for item in self._raw_encoder_history if item[0] >= timestamp_ns]
                    if not earlier or not later or later[0][0]-earlier[-1][0] > 250_000_000:
                        return False
                    a,b = earlier[-1],later[0]
                    counts = a[1] if a[0]==b[0] else a[1]+(timestamp_ns-a[0])/(b[0]-a[0])*(b[1]-a[1])
                    checkpoint = next(entry[2] for entry in reversed(self._backlash_history) if entry[0] <= timestamp_ns)
                    self.runtime.initialize(timestamp_ns,counts,points,quality=quality,
                        tendon_confirmation=confirmation_from_checkpoint(self.runtime,checkpoint))
                    for stamp,counts in self._raw_encoder_history:
                        if stamp > timestamp_ns:
                            checkpoint = next(entry[2] for entry in reversed(self._backlash_history) if entry[0] <= stamp)
                            self.runtime.advance_encoder(stamp,counts,return_snapshot=False,
                                tendon_confirmation=confirmation_from_checkpoint(self.runtime,checkpoint))
                    from cr_meta_lnn.deployment.v171_streaming_runtime import MarkerUpdateResult
                    result = MarkerUpdateResult(True,'initialized',self.runtime.state.health,4)
                else:
                    result = self.runtime.observe_markers_for_control(timestamp_ns,points,quality,
                        confirmation_observer=lambda estimator:self._engagement_confirmation_at_source(timestamp_ns,estimator,points))
            if self.runtime.state is None:
                return False
            if not self.engagement_control_enabled:
                result = self.runtime.observe_markers(timestamp_ns, points, quality)
            state = self.runtime.state
            accepted = state.accepted_observations
            rejected = state.consecutive_rejections
            health = state.health
            model_diagnostics = self.runtime.diagnostics()
            snapshot = (self.runtime.clone_state(include_history=False)
                        if self.engagement_control_enabled else self.runtime.clone_state())
            observation_snapshot = (
                (self.runtime.clone_state_at_or_before(timestamp_ns,include_history=False)
                 if self.engagement_control_enabled else self.runtime.clone_state_at_or_before(timestamp_ns))
                if result.accepted else None)
            if observation_snapshot is not None:
                # Pair raw and posterior at image source time, before encoder
                # replay. Only the tip readout is anchored; belief is untouched.
                source_markers = self.runtime.markers_for_state(observation_snapshot)
                if torch.is_tensor(source_markers):
                    source_markers = source_markers.detach().cpu().numpy()
                self._tip_readout_offset_m = (
                    np.asarray(points[-1], dtype=np.float64)
                    - np.asarray(source_markers[-1], dtype=np.float64)).copy()
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
                timing = getattr(self, "takeup_onset_timing", None)
                if timing and timing['takeup_onset_source_ns'] == timestamp_ns:
                    timing['takeup_onset_correction_commit_ros_ns'] = self.get_clock().now().nanoseconds
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
                        gain_coordinates = (
                            self.runtime.engaged_input_observation(observation_snapshot)
                            if self.takeup_transaction_enabled
                            and not self.engagement_control_enabled else
                            (result.distal_lambda_prior, result.distal_lambda_posterior))
                        if not self.engagement_control_enabled:
                            self.backlash_snapshot = (
                                self._observe_backlash_at_timestamp(
                                    timestamp_ns,
                                    observation_snapshot.interface_pose.detach()
                                    .cpu().numpy(), model,
                                    observation_snapshot.strain.detach()
                                    .cpu().numpy(),
                                    *gain_coordinates,
                                    observed_interface_position_m=points[0],
                                    observed_marker_points_m=points))
                        if self.engagement_control_enabled:
                            self.backlash_snapshot = reconcile_engagement(self.runtime,self.backlash_snapshot,
                                backlash_config=self.backlash_config)
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
                if getattr(self, "adapj", None) is not None:
                    try:
                        # Disarmed initialization probes are external position
                        # moves: their corrected source-time history, not the
                        # idle control-transaction arbiter, identifies takeup.
                        self.adapj.observe(timestamp_ns, observed_tip,
                            self._adapj_encoder_history,
                            initialization_history=self._backlash_history,
                            update_allowed=self.armed and not self.fault_latched
                                and self.estimator_catchup_started_at is None
                                and (not self.takeup_transaction_enabled or
                                     self.takeup_arbiter.state == TakeupTransactionArbiter.READY_TO_PLAN))
                    except (ValueError, RuntimeError) as error:
                        self._fault_locked(f"adapj_observation:{error}")
                        return False
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

    def _stop_on_raw_tendon_onset(self, timestamp_ns, points, quality, arrival):
        """Estimator-owner stop preview; no posterior or replay-state commit."""
        if not self.takeup_transaction_enabled:
            return
        with self._lock:
            if (not self.armed or self.fault_latched or not self.mode_claimed
                    or self.state != ControllerState.ACTIVE
                    or not self.takeup_arbiter.pending_mask[2]
                    or self.takeup_arbiter.response_onset_seen):
                return
        confidence = np.asarray(quality['confidence'])
        rigs = np.asarray(quality['source_rig_count'])
        reprojection = np.asarray(quality['reprojection_error_px'])
        if not np.all((confidence > 0) & (rigs >= 1) & (reprojection >= 0)):
            return
        preview = self.backlash_estimator.preview_tendon_onset(
            self._backlash_history, timestamp_ns, points, control_owned=self.armed)
        if preview is None:
            return
        with self._lock:
            if (not self.armed or self.fault_latched or not self.mode_claimed
                    or self.state != ControllerState.ACTIVE
                    or not self.takeup_arbiter.note_tendon_onset(preview)):
                return
            detected = self._steady()
            self.takeup_onset_timing = {
                'takeup_onset_generation': self.takeup_arbiter.generation,
                'takeup_onset_source_ns': int(timestamp_ns),
                'takeup_onset_marker_arrival_ros_ns': int(quality.get('callback_arrival_ros_ns', 0)),
                'takeup_onset_detect_ros_ns': self.get_clock().now().nanoseconds,
                'takeup_onset_marker_arrival_steady_s': float(arrival),
                'takeup_onset_detect_steady_s': float(detected),
                'takeup_onset_arrival_to_detect_ms': (detected-arrival)*1000.,
            }
            self.last_command.fill(0.)
            self.last_command_time = detected
            self.state_reason = 'takeup_response_hold'
            # A zero-only stop can precede expensive shape correction safely.
            # The arbiter still owns the transaction; planner belief stays intact.
            self._publish_velocity_locked(np.zeros(6))

    @gc_work
    @instrument_timer("estimator")
    def _estimator_tick(self):
        # This callback is the only owner of mutable runtime state. Catch up to
        # the newest encoder before considering a marker, then drain once more
        # after the expensive correction so the published snapshot is current.
        self._process_encoder_sample(self._take_encoder_sample())
        marker = self._take_causal_marker_sample(self._steady())
        self._process_marker_sample(marker)
        self._process_encoder_sample(self._take_encoder_sample())

    @instrument_timer("estimator")
    def _reduced_owner_tick(self):
        """Sole owner of the reduced filter (reduced_filter backend only).

        Absorbs encoder and marker inputs in arrival order, advances at most one image, maps the owner's
        health onto the existing readiness gates, and plans right after a committed image when the plan
        is due (image-stamp cadence) and the owner is caught up. Each gate field is written only by the
        event it certifies: encoder_time when an encoder sample is absorbed, marker_time and
        accepted_observations when an image commits as tracking.
        """
        from cr_meta_lnn.deployment.v171_streaming_runtime import MarkerUpdateResult
        owner = self.reduced_owner
        events, overflowed = self.reduced_inbox.drain()
        if overflowed:
            owner.mark_history_lost()
            self._fault_if_active("reduced_filter_input_overflow")
        absorbed_encoder_at = None
        for kind, stamp, values, arrival in events:
            try:
                if kind == "encoder":
                    owner.add_encoder(stamp, values)
                    absorbed_encoder_at = arrival
                else:
                    owner.add_image(stamp, values)
            except ValueError as error:
                self._fault_if_active(f"reduced_filter_input:{error}"[:120])
        if absorbed_encoder_at is not None:
            with self._lock:
                self.encoder_time = absorbed_encoder_at
        result = owner.process()
        health = owner.health()
        with self._lock:
            self.consecutive_rejections = int(health.consecutive_rejections)
            self.estimator_health = REDUCED_GATE_HEALTH.get(
                health.estimator_health, health.estimator_health)
            self.model_valid = health.estimator_health != "FAULT"
            if result is not None:
                accepted = result.status in ("initialized", "tracking")
                self.last_marker_result = MarkerUpdateResult(
                    accepted, str(result.reason or result.status), self.estimator_health, 4)
                if result.status == "tracking":
                    self.accepted_observations += 1
                    self.marker_time = self._steady()
                    self.runtime_initialized = True
                elif (result.status == "holding" and self.armed
                        and not self.fault_latched
                        and self.state == ControllerState.ACTIVE):
                    # A rejected image never authorizes motion: hold zero until
                    # an image commits again (persisting holds fail freshness).
                    self.last_command.fill(0.0)
                    self.last_effective_command.fill(0.0)
                    self.last_command_time = self._steady()
                    self.state_reason = "reduced_filter_holding_zero"
        if (result is not None and result.status == "tracking" and health.ready
                and plan_due(self.reduced_last_plan_image_ns, int(result.source_ns),
                             1.0/self.plan_rate_hz)):
            self.reduced_plan_attempted = False
            self._plan_tick()
            if self.reduced_plan_attempted:
                self.reduced_last_plan_image_ns = int(result.source_ns)

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
                if not self._pause_for_estimator_catchup_locked(
                        self._steady(), "marker_diagnostic_error"):
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
            if getattr(self, "adapj", None) is not None:
                self.adapj.invalidate()
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
                        self.planner.reset_capture_target()
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
                source_timestamp_ns=stamp_ns(message.header),
                received_at_s=self._steady(),
                sample_period_s=float(message.sample_period_s),
                positions_m=positions,
                tangents=tangents,
                x_only=bool(message.x_only),
                xz_only=bool(message.xz_only),
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
            new_path = prior is None or reference.path_id != prior.path_id
            policy_changed = new_path or (
                reference.final_hold != prior.final_hold
                or reference.progress_paused != prior.progress_paused
                or reference.x_only != prior.x_only
                or reference.xz_only != prior.xz_only)
            if new_path:
                self.reversal_scheduler.reset_intent()
            self.path_reference = reference
            self.reference_source = "PATH"
            offset_s = max(
                0.0, (self._now_ns()-reference.source_timestamp_ns)/1e9)
            index = min(
                len(reference.positions_m)-1,
                int(round(offset_s/reference.sample_period_s)))
            self.target = reference.positions_m[index].copy()
            # Rolling samples describe the same task, not a new capture target.
            # Resetting every 30 Hz preview invalidates in-flight worker solves.
            if policy_changed:
                self.planner.reset_capture_target()
                self.tip_forecast_monitor.clear()
                self.last_tip_forecast_result = None

    def _collection_event_cb(self, _message: String):
        with self._lock:
            self.collection_present = True
            self._fault_locked("collection_event_detected")

    def _adapj_initialization_cb(self, request, response):
        with self._lock:
            if self.armed or self.mode_claimed or self.fault_latched:
                response.success = False
                response.message = "initialization_requires_disarmed_nonfaulted_controller"
                return response
            directory = str(self.get_parameter("adapj_initialization_dir").value)
            if not directory:
                response.success = False
                response.message = "adapj_initialization_dir_required"
                return response
            try:
                if request.data:
                    self.adapj.begin()
                    response.message = "adapj_initialization_recording"
                else:
                    response.message = str(self.adapj.finish(directory))
                response.success = True
            except (ValueError, OSError) as error:
                response.success = False
                response.message = str(error)
            return response

    def _adapj_cancel_initialization_cb(self, request, response):
        with self._lock:
            if self.armed or self.mode_claimed:
                response.success = False
                response.message = "refusing to discard armed AdapJ state"
            else:
                self.adapj.capturing = False
                self.adapj.controller = None
                self.adapj.invalidate()
                response.success = True
                response.message = "initialization_canceled"
            return response

    def _arm_cb(self, request: SetBool.Request, response: SetBool.Response):
        # Never let a service request wait indefinitely behind recurrent
        # sensor work. Collection coexistence is checked again at the start of
        # every plan tick, before a manager mode can be claimed.
        if not self._lock.acquire(timeout=0.25):
            response.success = False
            response.message = "controller_busy"
            return response
        try:
            if self._shutting_down:
                response.success = False
                response.message = "controller_shutdown"
                return response
            if not request.data:
                self._release_locked()
                self.armed = False
                self._task_gc.stop()
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
            if getattr(self, "adapj", None) is not None and not self.adapj.initialized:
                response.success = False
                response.message = "adapj_initialization_required"
                return response
            try:
                gc_ready = self._task_gc.begin()
            except (OSError, ValueError, IndexError):
                response.success = False
                response.message = "task_gc_rss_unavailable"
                return response
            if not gc_ready:
                response.success = False
                response.message = "idle_gc_maintenance_pending"
                return response
            self._release_locked()
            self.reset_pending = True
            self._planner_warmup_supersessions = 0
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
        if getattr(self, "reduced_filter", False):
            response.success = False
            response.message = "reduced_filter_backend_has_no_simulation_reset"
            return response
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
                self._adapj_encoder_history.clear()
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
                self._planner_warmup_supersessions = 0
                self.consecutive_deadline_misses = 0
                self.estimator_catchup_started_at = None
                self.estimator_catchup_pause_count = 0
                with self._snapshot_lock:
                    self._planner_snapshot = None
                    self._planner_snapshot_source_time = None
                    self._tip_readout_offset_m = None
                    self._planner_tip_readout_offset_m = None
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
            encoder_age_s=age(self.encoder_receive_time if getattr(self, 'adapj', None) is not None
                              else self.encoder_time),
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
        """Hold zero through a bounded interruption and fresh-decision barrier."""
        new_pause = self.estimator_catchup_started_at is None
        if new_pause:
            self.estimator_catchup_started_at = now
            self.estimator_catchup_observations = self.accepted_observations
        elapsed = now-self.estimator_catchup_started_at
        inputs = self._gate_inputs_locked(now)
        if not recoverable_runtime_interruption(
                inputs, self.limits, reason, elapsed,
                self.estimator_catchup_timeout_s):
            return False
        if new_pause:
            self.estimator_catchup_pause_count += 1
        self.last_command.fill(0.0)
        self.last_effective_command.fill(0.0)
        self.last_command_time = now
        self.state = ControllerState.ACTIVE
        self.state_reason = "runtime_pause_zero:"+reason
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
        if released and getattr(self, "adapj", None) is not None:
            self.adapj.resume_after(self.get_clock().now().nanoseconds)
        if (released and transaction_state
                == TakeupTransactionArbiter.SATURATED_REPLAN):
            self.planner.reset()
            self.last_effective_command.fill(0.0)
        return released, transaction_state

    def _planner_reference_matches_locked(self, source, target, reference,
                                          root_timestamp_ns, now):
        if self.reference_source != source:
            return False
        if source == "POINT":
            return np.array_equal(self.target, target)
        if source == "PATH":
            return reference is not None and reference.compatible_with(
                self.path_reference, root_timestamp_ns,
                self.planner.config.horizon_steps, self.planner.config.step_s,
                now, bounded_refresh=True, bounded_tangent_tolerance=2e-3)
        return False

    def _complete_planner_warmup_locked(self, plan):
        """Discard cold decisions; retry supersession without motion authority."""
        if plan.reason == "worker_state_changed":
            self._planner_warmup_supersessions += 1
            self.last_command.fill(0.0)
            self.last_effective_command.fill(0.0)
            self.last_command_time = self._steady()
            if self._planner_warmup_supersessions >= self.maximum_deadline_misses:
                self._fault_locked("planner_warmup:repeated_state_change")
            else:
                self.state = ControllerState.READY
                self.state_reason = "planner_warmup_superseded_retry"
            return
        if plan.reason not in ("ok", "deadline_missed"):
            self._fault_locked(f"planner_warmup:{plan.reason}")
            return
        self._planner_warmup_supersessions = 0
        self.planner_warmed = True
        self.state = ControllerState.READY
        self.state_reason = "planner_warmed"
        self.get_logger().info(
            "MPPI warm-up complete; executable plans remain "
            "subject to the configured deadline")

    @gc_work
    @instrument_timer("plan")
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
                if self.estimator_catchup_started_at is not None:
                    if not recoverable_runtime_interruption(
                            self._gate_inputs_locked(now), self.limits, "ready",
                            now-self.estimator_catchup_started_at,
                            self.estimator_catchup_timeout_s):
                        self._fault_locked("runtime_recovery_timeout")
                        return
                    if self.accepted_observations <= self.estimator_catchup_observations:
                        self._pause_for_estimator_catchup_locked(now, "ready")
                        return
                if (self.engagement_control_enabled
                        and not self.model_diagnostics.get('prediction_ready', False)):
                    self.last_command.fill(0.0)
                    self.last_effective_command.fill(0.0)
                    self.last_command_time = now
                    self.state_reason = 'model_posterior_not_ready'
                    return
                if not self.planner_warmed and getattr(self, "control_method", "mppi") != "adapj":
                    # Run the framework's lazy first rollout before claiming
                    # a control mode. Its command is discarded below.
                    warmup = True
                    self.state = ControllerState.READY
                    self.state_reason = "warming_planner"
                elif not self.mode_claimed:
                    if (getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream'
                            and not self._position_stream_ready_locked(now)):
                        return
                    self._send_mode_locked(ManagerEvent.JOINT_VEL)
                    self.mode_claimed = True
                    self.mode_requested_at = now
                    self.state = ControllerState.READY
                    self.state_reason = "waiting_for_joint_velocity_mode"
                    return
                if (not warmup and self.takeup_transaction_enabled
                        and self.takeup_arbiter.state
                        in (TakeupTransactionArbiter.TAKEUP_ACTIVE,
                            TakeupTransactionArbiter.CONFIRMATION_HOLD,
                            TakeupTransactionArbiter.RESPONSE_HOLD)):
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
                    # A fresh observation and current arbiter decision, not
                    # a cached burst, authorize resuming the owned transaction.
                    self._clear_estimator_catchup_locked()
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
                    if getattr(self, "control_method", "mppi") == "adapj":
                        self._adapj_plan_locked(now)
                        return
                    # Do not advertise ACTIVE until a first valid command has
                    # actually been committed. The heartbeat runs at 100 Hz;
                    # marking ACTIVE before the first plan completes lets it
                    # observe no command and latch a spurious stale fault.
                    if self.last_command_time is None:
                        self.state = ControllerState.READY
                        self.state_reason = "planning_first_command"
                    if (self.reference_source == "POINT"
                            and self.estimator_catchup_started_at is None
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
                if (not warmup and getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream'
                        and position_execution_in_flight(self.position_execution, self.get_clock().now().nanoseconds)):
                    # Input, interruption, take-up and capture-hold gates above
                    # retain priority. Fresh owner heartbeats do not renew the
                    # immutable device deadline. Execute one native first-step
                    # window before committing another stop-and-replace move.
                    self.last_command_time = now
                    self.state_reason = 'driver_native_command_in_flight'
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
                recovery_started_at = self.estimator_catchup_started_at
                observed_tip = (
                    None if self.observed_tip is None else
                    self.observed_tip.copy())

            if getattr(self, "reduced_filter", False):
                # Same thread as the owner (commit-triggered): the root is the
                # filter's committed state plus the measured encoder prefix.
                rooted = self.reduced_owner.plan_root()
                if rooted.root is None:
                    with self._lock:
                        if self.state == ControllerState.ACTIVE:
                            self.last_command.fill(0.0)
                            self.last_effective_command.fill(0.0)
                            self.last_command_time = self._steady()
                            self.state_reason = "reduced_filter_root_refused_zero"
                    return
                if reference_source == "PATH":
                    with self._lock:
                        self._fault_locked("reduced_filter_path_reference_unsupported")
                    return
                root = rooted.root
                position = np.asarray(rooted.joint_position, dtype=np.float64)
                with self._lock:
                    snapshot_source_time = self.encoder_time
                tip_readout_offset = None
                transmission_snapshot = None
            else:
                # The estimator owner publishes complete replace-only snapshots.
                # Reading one never waits for marker correction or rewind/replay.
                with self._timed_lock(self._snapshot_lock, "plan_snapshot"):
                    root = self._planner_snapshot
                    snapshot_source_time = self._planner_snapshot_source_time
                    tip_readout_offset = self._planner_tip_readout_offset_m
                    transmission_snapshot = self._planner_transmission_snapshot
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
            root_timestamp_ns = (
                root.plan_ns if getattr(self, "reduced_filter", False) else root.timestamp_ns)
            if reference_source == "PATH":
                if path_reference is None:
                    with self._lock:
                        self._fault_locked("path_reference_missing")
                    return
                try:
                    target, target_tangent = path_reference.sample(
                        root_timestamp_ns,
                        self.planner.config.horizon_steps,
                        self.planner.config.step_s, self._steady())
                except ValueError as error:
                    with self._lock:
                        self._fault_locked(str(error))
                    return
            capture_tip = None
            if tip_readout_offset is not None:
                root_markers = self.runtime.markers_for_state(root)
                if torch.is_tensor(root_markers):
                    root_markers = root_markers.detach().cpu().numpy()
                capture_tip = (np.asarray(root_markers[-1], dtype=np.float64)
                    + tip_readout_offset)
            if getattr(self, "reduced_filter", False):
                # The dry run's validated call: the filter alone owns play,
                # take-up and response, so no October transmission inputs.
                self.reduced_plan_attempted = True
                try:
                    plan = self.planner.plan(
                        root, position, target, deadline_started_s=callback_started)
                except ValueError as error:
                    with self._lock:
                        if self.state == ControllerState.ACTIVE:
                            self.last_command.fill(0.0)
                            self.last_effective_command.fill(0.0)
                            self.last_command_time = self._steady()
                            self.state_reason = "reduced_filter_planner_error_zero"
                    self.get_logger().warn(f"reduced-filter plan refused, commanding zero: {error}")
                    return
            else:
                if not self.planner_process_enabled:
                    root = self._planner_root_on_device(root)
                plan = self.planner.plan(
                    root, position, target,
                    x_only=(reference_source == "PATH" and path_reference.x_only),
                    xz_only=(reference_source == "PATH" and path_reference.xz_only),
                    target_tangent_base=(
                        None if reference_source != "PATH"
                        or path_reference.final_hold else target_tangent),
                    observed_tip_base_m=observed_tip,
                    tip_readout_offset_m=tip_readout_offset,
                    capture_tip_base_m=capture_tip,
                    previous_logical_velocity=previous,
                    deadline_started_s=callback_started,
                    transmission_state=(transmission_snapshot if self.engagement_control_enabled else self.backlash_snapshot),
                    takeup_motor_radians_per_second=(
                        self.backlash_compensator.motor_radians_per_second(
                            self.contract)),
                    blocked_motor_direction=blocked_motor_direction,
                    direction_lease=reversal_state.lease_direction,
                    approved_reversal_direction=(
                        reversal_state.approved_direction))
            if self.planner_process_enabled:
                for name, seconds in self.planner.process_timings_s.items():
                    self._timing.record_seconds(name, seconds)
            if warmup:
                # Warm-start control values must not inherit the discarded
                # rollout. Cold deadline misses are expected; superseded work
                # retries below. Real worker/transport failures remain faults.
                self.planner.reset()
                with self._lock:
                    plan = self.planner.enforce_deadline(
                        plan, callback_started)
                    self._publish_control_cycle_timing_locked(
                        plan, callback_started, root_timestamp_ns)
                    if not self.armed or self.fault_latched:
                        return
                    self._complete_planner_warmup_locked(plan)
                return
            # Diagnostic readout from the same immutable estimator root used
            # by the solve. Keep root fit offset separate from predicted motion.
            model_start_tip = None
            if (plan.valid and plan.command_tip_sequence_m is not None
                    and not getattr(self, "reduced_filter", False)):
                root_points = self.runtime.markers_for_state(root)
                if torch.is_tensor(root_points):
                    root_points = root_points.detach().cpu().numpy()
                model_start_tip = np.asarray(root_points[-1], dtype=np.float64)
                if tip_readout_offset is not None:
                    model_start_tip = capture_tip
            with self._lock:
                plan = self.planner.enforce_deadline(
                    plan, callback_started)
                self._publish_control_cycle_timing_locked(
                    plan, callback_started, root_timestamp_ns)
                if not self.armed or self.fault_latched:
                    return
                if (self.estimator_catchup_started_at is not None
                        and (self.estimator_catchup_started_at != recovery_started_at
                             or snapshot_source_time < self.estimator_catchup_started_at)):
                    # Never resume with a solve started before the interruption,
                    # even if readiness recovered while the worker was running.
                    if not self._pause_for_estimator_catchup_locked(
                            self._steady(), "ready"):
                        self._fault_locked("runtime_recovery_timeout")
                    return
                if (self.planner_process_enabled
                        and self._steady()-snapshot_source_time > self.limits.feedback_s):
                    if self._pause_for_estimator_catchup_locked(
                            self._steady(), "planner_snapshot_stale_at_commit"):
                        return
                    self._fault_locked("planner_snapshot_stale_at_commit")
                    return
                try:
                    reference_matches = (
                        not self.planner_process_enabled
                        or self._planner_reference_matches_locked(
                            reference_source, target, path_reference,
                            root_timestamp_ns, self._steady()))
                except ValueError as error:
                    self._fault_locked(str(error))
                    return
                if not reference_matches:
                    self.last_command.fill(0.0)
                    self.last_effective_command.fill(0.0)
                    self.last_command_time = self._steady()
                    self.state_reason = "planner_reference_changed_zero"
                    return
                gate_state, reason = readiness(
                    self._gate_inputs_locked(self._steady()), self.limits)
                if gate_state != ControllerState.READY or reason != "ready":
                    if self._pause_for_estimator_catchup_locked(
                            self._steady(), reason):
                        return
                    self._fault_locked(reason)
                    return
                if self.engagement_control_enabled and not self.model_diagnostics.get('prediction_ready',False):
                    self.last_command.fill(0.)
                    self.last_effective_command.fill(0.)
                    self.last_command_time = self._steady()
                    self.state_reason = 'model_posterior_changed_during_plan_zero'
                    return
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
                    if not self._pause_for_estimator_catchup_locked(
                            self._steady(), "deadline_missed"):
                        self._fault_locked("runtime_recovery_timeout")
                        return
                    self._publish_plan_locked(plan)
                    self.get_logger().warn(
                        "MPPI deadline missed; commanding zero (%d/%d)" % (
                            self.consecutive_deadline_misses,
                            self.maximum_deadline_misses))
                    return
                if not plan.valid:
                    if plan.reason == "worker_state_changed":
                        # A newer target/reset/capture observation won the
                        # race. Never commit the stale worker decision.
                        self.last_command.fill(0.0)
                        self.last_effective_command.fill(0.0)
                        self.last_command_time = self._steady()
                        self.state_reason = "planner_state_changed_zero"
                        return
                    self._fault_locked(f"planner:{plan.reason}")
                    return
                first_active_plan = self.last_command_time is None
                self._clear_estimator_catchup_locked()
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
                                TakeupTransactionArbiter.CONFIRMATION_HOLD,
                                TakeupTransactionArbiter.RESPONSE_HOLD)):
                        self.reversal_scheduler.mark_transaction_started(
                            decision.direction)
                else:
                    self.last_command = self._compensated_command_locked(
                        self.last_effective_command, position)
                    self.state_reason = "active"
                self.last_command_time = self._steady()
                self.state = ControllerState.ACTIVE
                self.position_execution = None
                compensating_backlash = not np.allclose(
                    self.last_command, self.last_effective_command,
                    rtol=0.0, atol=1e-12)
                forecast_matches_command = (
                    execute_plan and not compensating_backlash
                    and not np.any(
                        self.backlash_snapshot.taking_up
                        & self.takeup_response_free_mask))
                if forecast_matches_command:
                    if getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream':
                        try:
                            self.position_stream_command_id = (self.position_stream_command_id % 0xffffffff)+1
                            position_anchor_counts = root_encoder_counts(self._raw_encoder_history, root_timestamp_ns)
                            dt = (self.planner.config.point_rollout_step_s
                                  if self.reference_source == 'POINT' and self.planner.config.point_rollout_step_s > 0
                                  else self.planner.config.step_s)
                            self.position_execution = make_position_execution(
                                self.contract, self.position_stream_command_id, position_anchor_counts,
                                self.position_stream_encoder_counts, self.last_command, dt,
                                self.get_clock().now().nanoseconds)
                        except ValueError as error:
                            self._fault_locked(f'position_execution:{error}')
                            return
                    self.planner.note_capture_control_executed(
                        self.last_command)
                if (forecast_matches_command
                        and plan.command_tip_sequence_m is not None
                        and self.observed_tip is not None
                        and self.observed_tip_timestamp_ns is not None):
                    self.tip_forecast_monitor.add(
                        root_timestamp_ns,
                        self.observed_tip_timestamp_ns,
                        (self.planner.config.point_rollout_step_s
                         if reference_source != "PATH"
                         and self.planner.config.point_rollout_step_s > 0.0
                         else self.planner.config.step_s),
                        self.observed_tip,
                        plan.command_tip_sequence_m[0],
                        model_start_tip_m=model_start_tip,
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

    def _adapj_plan_locked(self, now):
        if self.reference_source != "POINT":
            self._fault_locked("adapj_point_reference_required")
            return
        if not self.mode_claimed:
            self._send_mode_locked(ManagerEvent.JOINT_VEL)
            self.mode_claimed = True
            self.mode_requested_at = now
            self.state = ControllerState.READY
            self.state_reason = "waiting_for_joint_velocity_mode"
            return
        if now-self.mode_requested_at < self.mode_settle_s:
            return
        if (getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream'
                and position_execution_in_flight(self.position_execution, self.get_clock().now().nanoseconds)):
            self.last_command_time = now
            self.state_reason = 'adapj_driver_native_command_in_flight'
            return
        if self.estimator_catchup_started_at is not None:
            self.adapj.invalidate()
        try:
            command = self.adapj.command(self.target, self.position,
                                        self.get_clock().now().nanoseconds)
        except (ValueError, RuntimeError) as error:
            self._fault_locked(f"adapj:{error}")
            return
        if command is None:
            if self.adapj.interval_active(self.get_clock().now().nanoseconds):
                # The heartbeat bounds the pulse independently of planner ticks.
                # A camera sample arriving early must not shorten/replan it.
                return
            # One bounded command interval, then zero until a causally later
            # camera/encoder pair arrives. Never prolong an old velocity while
            # waiting for the visual pipeline to report its response.
            self.last_command.fill(0.)
            self.last_effective_command.fill(0.)
            self.last_command_time = now
            self.state_reason = "adapj_response_hold"
            return
        self._clear_estimator_catchup_locked()
        native = getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream'
        endpoint = None
        if native:
            try:
                self.position_stream_command_id = (self.position_stream_command_id % 0xffffffff)+1
                counts = self.position_stream_encoder_counts
                endpoint = make_position_execution(
                    self.contract, self.position_stream_command_id, counts, counts,
                    command, self.adapj.step_s, self.get_clock().now().nanoseconds,
                    integer_degrees=True, authority_s=.25, preserve_displacement=True)
            except ValueError as error:
                self.adapj.invalidate()
                self._fault_locked(f'adapj_position_execution:{error}')
                return
            if not endpoint.command.active_mask:
                self.adapj.invalidate()
                self.position_execution = None
                self.last_command.fill(0.)
                self.last_effective_command.fill(0.)
                self.last_command_time = now
                self.state_reason = 'adapj_position_below_driver_resolution'
                return
            # Reflect the quantized, shaft-masked endpoint in control status
            # and response monitoring as well as on the native wire.
            command = self.contract.motor_axis_to_logical_velocity(
                (np.asarray(endpoint.command.target_counts)-counts)
                * self.contract.encoder_counts_to_motor_axis_position(np.ones(6))
                / self.adapj.step_s)
            endpoint = replace(endpoint, velocity=tuple(command))
        self.last_effective_command = command.copy()
        if self.takeup_transaction_enabled:
            options = {}
            if endpoint is not None:
                options['physical_direction'] = np.sign(
                    np.asarray(endpoint.command.target_counts)[:3]-counts[:3])
            decision = self.takeup_arbiter.begin(command, self.position,
                                                self.contract, self.backlash_snapshot, **options)
            if decision.state == TakeupTransactionArbiter.FAILED:
                self.adapj.invalidate()
                self._fault_locked(decision.reason)
                return
            if decision.state == TakeupTransactionArbiter.SATURATED_REPLAN:
                self._record_takeup_saturation_locked(decision, self.position)
            if not decision.execute_plan:
                self.adapj.invalidate()
            self.last_command = decision.command_logical_velocity
            self.state_reason = decision.reason
        else:
            self.last_command = command
            self.state_reason = "active"
        self.position_execution = None
        if native and (not self.takeup_transaction_enabled or decision.execute_plan):
            self.position_execution = endpoint
            self.last_command = np.asarray(endpoint.velocity).copy()
        self.last_command_time = now
        self.state = ControllerState.ACTIVE
        message = ControlStream()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_vel = command.tolist()
        self.plan_pub.publish(message)

    @instrument_timer("heartbeat")
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
            if self.estimator_catchup_started_at is not None:
                if not self._pause_for_estimator_catchup_locked(now, "ready"):
                    self._fault_locked("runtime_recovery_timeout")
                return
            if self.state != ControllerState.ACTIVE:
                self._publish_velocity_locked(np.zeros(6))
                return
            if (self.takeup_transaction_enabled
                    and self.takeup_arbiter.state
                    in (TakeupTransactionArbiter.TAKEUP_ACTIVE,
                        TakeupTransactionArbiter.CONFIRMATION_HOLD,
                        TakeupTransactionArbiter.RESPONSE_HOLD,
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
            native_adapj = (getattr(self, 'adapj', None) is not None
                           and getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream')
            if not self.takeup_transaction_enabled and not native_adapj:
                self.last_command = self._compensated_command_locked(
                    self.last_effective_command, self.position)
            command = (self.last_command if native_adapj else
                       self.contract.project_publishable_velocity(
                           self.last_command, self.position))
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
        if getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream':
            taking_up = (self.takeup_transaction_enabled and
                         self.takeup_arbiter.state != TakeupTransactionArbiter.READY_TO_PLAN)
            native_takeup = taking_up and getattr(self, 'adapj', None) is not None
            if native_takeup and np.any(values):
                execution = self.position_execution
                now_ns = self.get_clock().now().nanoseconds
                if (execution is not None and now_ns < execution.deadline_ros_ns
                        and not np.allclose(values, execution.velocity, rtol=0, atol=1e-12)):
                    # Cancel, then prove standstill before changing shaft set.
                    # This shortened local barrier is never sent as a replacement.
                    self.position_execution = execution = replace(
                        execution, deadline_ros_ns=now_ns)
                if execution is None or (now_ns >= execution.deadline_ros_ns
                        and measured_standstill(self._adapj_encoder_history,
                                               execution.deadline_ros_ns)):
                    try:
                        self.position_stream_command_id = (self.position_stream_command_id % 0xffffffff)+1
                        counts = self.position_stream_encoder_counts
                        self.position_execution = make_position_execution(
                            self.contract, self.position_stream_command_id, counts, counts,
                            values, self.adapj.step_s, now_ns, integer_degrees=True,
                            authority_s=.25, preserve_displacement=True)
                    except ValueError as error:
                        self._fault_locked(f'adapj_takeup_position_execution:{error}')
                        return
            if not np.any(values):
                self.position_execution = None
            elif not taking_up or native_takeup:
                execution = self.position_execution
                now_ns = self.get_clock().now().nanoseconds
                if (execution is not None and now_ns < execution.deadline_ros_ns
                        and np.allclose(values, execution.velocity, rtol=0, atol=1e-12)):
                    packet = StreamedPositionControl()
                    packet.header = message.header
                    packet.session_id = self.position_stream_session
                    packet.command_id = execution.command.command_id
                    packet.duration_us = execution.command.duration_us
                    packet.deadline_ros_ns = execution.deadline_ros_ns
                    packet.active_mask = execution.command.active_mask
                    packet.target_counts = list(execution.command.target_counts)
                    packet.speed_rpm = list(execution.command.speed_rpm)
                    self.position_control_pub.publish(packet)
                    if native_takeup:
                        self.takeup_arbiter.note_command_publication(now_ns, bool(packet.active_mask & 4))
                    elif getattr(self, 'adapj', None) is not None:
                        self.adapj.note_position_execution(execution.deadline_ros_ns, now_ns)
                    return
                # Missing/expired/mutated endpoints never fall back to motion.
                if not native_takeup:
                    self.position_execution = None
                values = np.zeros(6)
        if (getattr(self, "adapj", None) is not None
                and getattr(self, 'mppi_execution_mode', 'velocity') != 'position_stream'
                and (not getattr(self, "takeup_transaction_enabled", False)
                     or self.takeup_arbiter.state == TakeupTransactionArbiter.READY_TO_PLAN)):
            values = self.adapj.execution_velocity(
                values, message.header.stamp.sec*1_000_000_000
                + message.header.stamp.nanosec)
        message.joint_vel = [float(value) for value in values]
        if self.takeup_transaction_enabled:
            publication_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            self.takeup_arbiter.note_command_publication(publication_ns, bool(values[2]))
            timing = getattr(self, 'takeup_onset_timing', None)
            if timing and not np.any(values) and 'takeup_onset_zero_publish_ros_ns' not in timing:
                timing['takeup_onset_zero_publish_ros_ns'] = publication_ns
        self.tip_forecast_monitor.note_command(
            message.header.stamp.sec*1_000_000_000
            + message.header.stamp.nanosec, values)
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

    def _position_stream_ready_locked(self, now):
        if getattr(self, 'mppi_execution_mode', 'velocity') != 'position_stream':
            return True
        if self.position_stream_session:
            return True
        self.state_reason = 'waiting_for_position_stream_session'
        if self.position_stream_future is None:
            if not self.position_stream_client.service_is_ready():
                self._fault_locked('position_stream_service_unavailable')
                return False
            request = OpenPositionStream.Request()
            request.source = self.source
            self.position_stream_future = self.position_stream_client.call_async(request)
            self.position_stream_open_started = now
            return False
        if not self.position_stream_future.done():
            if now-self.position_stream_open_started > 2.0:
                self._fault_locked('position_stream_session_timeout')
            return False
        try:
            response = self.position_stream_future.result()
            if not response.success or not response.session_id:
                raise ValueError(response.message)
            self.position_stream_session = response.session_id
            return True
        except Exception as error:
            self._fault_locked(f'position_stream_session:{error}')
            return False

    def _release_locked(self):
        self.position_execution = None
        self.position_stream_session = 0
        self.position_stream_future = None
        if getattr(self, "adapj", None) is not None:
            self.adapj.invalidate()
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
        policy = getattr(self, '_task_gc', None)
        if policy is not None:
            policy.stop()
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
        if (not result.comparison_valid
                or not result.capture_hold or result.target_tip_m is None
                or result.start_tip_m is None
                or result.predicted_terminal_tip_m is None
                or result.observed_tip_m is None):
            return False
        target = np.asarray(result.target_tip_m, dtype=np.float64)
        start_error = 1e3*float(np.linalg.vector_norm(
            target-result.start_tip_m))
        predicted_error = 1e3*float(np.linalg.vector_norm(
            target-(result.start_tip_m+result.predicted_delta_mm/1e3)))
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
        aligned = MppiAlignedResponseTrace()
        aligned.header.stamp.sec = result.source_observation_timestamp_ns // 1_000_000_000
        aligned.header.stamp.nanosec = result.source_observation_timestamp_ns % 1_000_000_000
        aligned.header.frame_id = self.frame_id
        aligned.source_observation_timestamp_ns = result.source_observation_timestamp_ns
        aligned.source_start_observation_timestamp_ns = (
            result.source_start_observation_timestamp_ns)
        aligned.comparison_valid = result.comparison_valid
        aligned.comparison_reason = result.comparison_reason
        aligned.model_start_tip_m = (
            [math.nan]*3 if result.model_start_tip_m is None
            else result.model_start_tip_m.tolist())
        aligned.root_readout_error_mm = (
            [math.nan]*3 if result.root_readout_error_mm is None
            else result.root_readout_error_mm.tolist())
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
        aligned.comparison = message
        self.aligned_response_trace_pub.publish(aligned)

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
            raw_encoder_counts, dtype=np.float64)[:3].tolist()
        message.downstream = state.downstream.detach().cpu().numpy().tolist()
        message.interface_pose = (
            state.interface_pose.detach().cpu().numpy().reshape(-1).tolist())
        message.distal_strain = (
            state.strain.detach().cpu().numpy().reshape(-1).tolist())
        estimated_markers = self.runtime.markers_for_state(state)
        if torch.is_tensor(estimated_markers):
            estimated_markers = estimated_markers.detach().cpu().numpy()
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
                "mppi_execution_mode": getattr(self, 'mppi_execution_mode', 'velocity'),
                "position_execution_backend": ('driver_native' if getattr(self, 'mppi_execution_mode', 'velocity') == 'position_stream' else 'velocity'),
                "position_stream_session": str(getattr(self, 'position_stream_session', 0)),
                "position_stream_command_id": str(getattr(self, 'position_stream_command_id', 0)),
                "control_method": self.control_method,
                "adapj_initialized": str(self.adapj is not None and self.adapj.initialized),
                "adapj_update_count": str(0 if self.adapj is None else self.adapj.updates),
                "adapj_reason": "disabled" if self.adapj is None else self.adapj.last_reason,
                "adapj_decision_timestamp_ns": "none" if self.adapj is None else str(self.adapj.pending_command_ns),
                "adapj_output_start_timestamp_ns": "none" if self.adapj is None else str(self.adapj.execution_started_ns),
                "adapj_output_stop_timestamp_ns": "none" if self.adapj is None else str(self.adapj.execution_stopped_ns),
                "adapj_parameters": "none" if self.adapj is None or not self.adapj.initialized else
                    json.dumps({key: value.tolist() for key, value in
                        self.adapj.controller.matrices.items()}, separators=(",", ":")),
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
                "encoder_gate_basis": (
                    "raw_receipt" if self.adapj is not None else "estimator_processed"),
                "encoder_receive_age_ms": (
                    "none" if inputs.encoder_receive_age_s is None
                    else f"{1e3*inputs.encoder_receive_age_s:.3f}"),
                "estimator_catchup_timeout_ms": format(
                    1e3*self.estimator_catchup_timeout_s, ".6g"),
                "estimator_catchup_pause_count": str(
                    self.estimator_catchup_pause_count),
                "runtime_recovery_active": str(
                    self.estimator_catchup_started_at is not None),
                "runtime_recovery_elapsed_ms": (
                    "0" if self.estimator_catchup_started_at is None else
                    f"{1e3*(now-self.estimator_catchup_started_at):.3f}"),
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
            }
            values.update(marker_diagnostic_values(
                self.last_marker_result))
            values["task_gc"] = json.dumps(self._task_gc.summary())
            values.update(transmission_diagnostic_values(
                self.last_command, self.last_effective_command,
                self.backlash_compensation_enabled,
                self.latest_raw_encoder_counts, self.backlash_snapshot,
                self.takeup_transaction_enabled, self.takeup_arbiter))
            values.update(planner_policy_diagnostic_values(
                self.planner.config, capture_diagnostics,
                self.reversal_scheduler, self.takeup_arbiter))
            values.update(runtime_diagnostic_values(
                blocked_motor_direction=self.blocked_motor_direction,
                takeup_saturation_position=self.takeup_saturation_position,
                takeup_saturation_position_timestamp_ns=(
                    self.takeup_saturation_position_timestamp_ns),
                takeup_saturation_release_reason=(
                    self.takeup_saturation_release_reason),
                position_valid=self.position_valid,
                encoder_valid=self.encoder_valid,
                planner_snapshot_source_time=(
                    self._planner_snapshot_source_time),
                now=now,
                torch_intraop_threads=self.torch_intraop_threads,
                torch_interop_threads=self.torch_interop_threads,
                contract=self.contract,
                raw_response_during_interface_takeup=(
                    self.raw_response_during_interface_takeup),
                takeup_response_free_mask=self.takeup_response_free_mask))
            values.update(compute_device_diagnostics(self.compute_device))
            values["planner_process_enabled"] = str(self.planner_process_enabled)
            if self.planner_process_enabled:
                values.update(self.planner.worker_diagnostics())
            values["estimator_compute_device"] = str(self.estimator_compute_device.device)
            values["planner_compute_device"] = str(self.compute_device.device)
            if getattr(self, "reduced_filter", False):
                values["estimator_backend"] = REDUCED_BACKEND
                values["estimator_compute_dtype"] = "float64"   # the filter runs in numpy
                values["planner_compute_dtype"] = str(self.reduced_backend.dtype)
                values["reduced_filter_kernel_preparations_ms"] = json.dumps(
                    {str(laws): round(seconds*1e3, 1)
                     for laws, seconds in self.reduced_owner.preparations})
                values["reduced_filter_images_dropped"] = str(
                    self.reduced_stream.aligner.images_dropped)
            else:
                values["estimator_compute_dtype"] = str(self.runtime.dtype)
                values["planner_compute_dtype"] = str(self.planner_runtime.dtype)
            values.update({
                key: (str(value) if isinstance(value, int)
                      else f"{value:.6g}")
                for key, value in self._timing.snapshot_and_reset().items()
            })
            plan_terminal_error_mm = None
            if (self.last_plan is not None
                    and self.last_plan.command_tip_sequence_m is not None
                    and self.target is not None):
                terminal = self.last_plan.command_tip_sequence_m[-1]
                _, plan_terminal_error_mm = tip_tracking_error_mm(
                    self.target, terminal)
            values.update(plan_diagnostic_values(
                self.last_plan, plan_terminal_error_mm))
            values.update(response_diagnostic_values(
                self.last_tip_forecast_result,
                self.tip_forecast_monitor.pending_count))
            values.update(model_diagnostic_values(
                self.model_diagnostics, self.model_valid))
            values['engagement_control_enabled'] = str(self.engagement_control_enabled)
            values.update({key: str(value) for key, value in
                           getattr(self, 'takeup_onset_timing', {}).items()})
            if self.engagement_control_enabled:
                values['command_owner'] = ('takeup' if self.takeup_arbiter.state in (
                    self.takeup_arbiter.TAKEUP_ACTIVE,self.takeup_arbiter.CONFIRMATION_HOLD,
                    self.takeup_arbiter.RESPONSE_HOLD)
                    else 'mppi' if self.state == ControllerState.ACTIVE and self.last_command.any() else 'hold')
                values['model_widths'] = json.dumps(self.model_diagnostics.get('widths',{}))
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

    def _gc_maintenance_tick(self):
        # Output teardown precedes reservation. Arming cannot race a reserved
        # collection; planner/estimator admission is closed until restoration.
        with self._lock:
            reason = self._task_gc.guard_reason()
            if reason:
                self._fault_locked(reason)
            collect = (not self.armed and not self.mode_claimed
                       and self._task_gc.reserve_collection())
        if collect:
            self._task_gc.collect_reserved()

    def shutdown(self):
        with self._lock:
            self._shutting_down = True
            self._release_locked()
            self.armed = False
            self._task_gc.stop()
            self.state = ControllerState.DISARMED
            self.state_reason = "shutdown"
        if isinstance(self.planner, ProcessPlanner):
            self.planner.close()

    def restore_gc_after_executor_shutdown(self):
        if self._task_gc.reserve_collection():
            self._task_gc.collect_reserved()


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
        try:
            if node is not None:
                node.shutdown()
                executor.remove_node(node)
        finally:
            # Wait for recurrent callbacks before final policy restoration,
            # even if closing the planner transport raised during teardown.
            executor.shutdown()
            if node is not None:
                node.restore_gc_after_executor_shutdown()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
