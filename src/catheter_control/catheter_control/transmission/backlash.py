"""Persistent backlash state estimation and bounded feedforward take-up."""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from time import monotonic

import copy
import numpy as np

from ..safety.hardware_contract import (
    MOTOR_AXIS_UNITS_PER_ENCODER_COUNT, knob_interface_translation_m)
from .engaged_gain import (
    EngagedGainConfig, EngagedGainEstimator, EngagedGainSnapshot)


CONTROL_AXES = 3


def _vector(name, value, size=CONTROL_AXES, *, nonnegative=False):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} must be nonnegative")
    return result


def _se3_log_components(transform):
    """Return rotation logarithm and its translation linear map."""
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("interface pose must be a finite 4x4 transform")
    rotation, translation = value[:3, :3], value[:3, 3]
    cosine = np.clip((np.trace(rotation)-1.0)/2.0, -1.0, 1.0)
    theta = float(np.arccos(cosine))
    if theta < 1e-7:
        omega_hat = 0.5*(rotation-rotation.T)
        omega = np.array([
            omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
        v_inverse = np.eye(3)-0.5*omega_hat+(omega_hat@omega_hat)/12.0
    else:
        omega_hat = theta/(2.0*np.sin(theta))*(rotation-rotation.T)
        omega = np.array([
            omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
        coefficient = (
            1.0/theta**2
            - (1.0+np.cos(theta))/(2.0*theta*np.sin(theta)))
        v_inverse = np.eye(3)-0.5*omega_hat+coefficient*(omega_hat@omega_hat)
    return omega, v_inverse, translation


def _se3_log(transform):
    """Return [rotation, translation] logarithm for one SE(3) transform."""
    omega, v_inverse, translation = _se3_log_components(transform)
    return np.r_[omega, v_inverse@translation]


@dataclass(frozen=True)
class BacklashConfig:
    width_rad: tuple[float, float, float]
    width_positive_rad: tuple[float, float, float] | None = None
    width_negative_rad: tuple[float, float, float] | None = None
    takeup_velocity: tuple[float, float, float] = (8.0, 40.0, 4.5)
    minimum_motor_increment_rad: float = 0.01
    minimum_transmitted_increment_rad: float = 0.10
    # Retained for launch/profile compatibility. Joint response attribution
    # supersedes the former dominant-axis purity gate.
    directional_purity: float = 0.90
    # Retained for configuration compatibility, not engagement decisions.
    response_direction_cosine: float = 0.50
    minimum_response_evidence: float = 0.50
    # Raw marker-0 *difference* covariance in robot-base coordinates, m^2.
    # Qualified from 703 stationary source pairs (20260919 causal_proximal);
    # same-session post-fault directional noise agrees with this scale.
    # This is not UKF R or posterior covariance. See the path-commit audit.
    marker_increment_covariance_m2: tuple[float, ...] = (
        2.246433668e-9, -8.407024420e-10, -2.010902684e-10,
        -8.407024420e-10, 8.844941085e-9, 1.152412296e-10,
        -2.010902684e-10, 1.152412296e-10, 8.245305540e-10)
    # Raw four-marker distance deformation is the production tendon evidence.
    # The strain threshold is retained for strain-only research/test callers.
    tendon_axis: int = 2
    minimum_distal_bending_increment: float = 0.05
    minimum_distal_marker_deformation_mm: float = 0.3
    # Unitless persistence/onset ratio. Applied only after credible raw onset;
    # 1.0 restores the strict single-floor policy. No additional state owner.
    distal_confirmation_release_ratio: float = 0.75
    # Distal bending proves tendon transmission, not chassis insertion.
    # Disable this for an interface-only observer rather than tendon take-up.
    # The knob's direct interface translation is independent of this flag.
    distal_confirmation_enabled: bool = True
    width_learning_rate: float = 0.05
    minimum_width_gain: float = 0.50
    maximum_width_gain: float = 1.50
    # Explicit experimental travel authority, not a learned width. Zero keeps
    # the established prior-derived budget. Supplied from joint span by ROS.
    tendon_unconfirmed_travel_limit_rad: float = 0.0
    engagement_confirmation_observations: int = 1
    provisional_rejection_observations: int = 2
    response_window_max_s: float = 1.0
    minimum_interface_rotation_rad: float = np.deg2rad(.1)

    def __post_init__(self):
        _vector("width_rad", self.width_rad, nonnegative=True)
        if ((self.width_positive_rad is None)
                != (self.width_negative_rad is None)):
            raise ValueError(
                "positive and negative backlash widths must be supplied "
                "together")
        if self.width_positive_rad is not None:
            _vector("width_positive_rad", self.width_positive_rad,
                    nonnegative=True)
            _vector("width_negative_rad", self.width_negative_rad,
                    nonnegative=True)
            if np.any(_vector(
                    "width_rad", self.width_rad, nonnegative=True) > 0.0):
                raise ValueError(
                    "directional backlash widths cannot be combined with "
                    "the legacy symmetric width")
        _vector("takeup_velocity", self.takeup_velocity, nonnegative=True)
        positive = (self.minimum_motor_increment_rad,
                    self.minimum_transmitted_increment_rad)
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("backlash increment floors must be positive")
        if not 0.0 < self.directional_purity <= 1.0:
            raise ValueError("directional_purity must be in (0,1]")
        if not -1.0 <= self.response_direction_cosine <= 1.0:
            raise ValueError("response_direction_cosine must be in [-1,1]")
        if not 0.0 <= self.minimum_response_evidence <= 1.0:
            raise ValueError("minimum_response_evidence must be in [0,1]")
        covariance = np.asarray(self.marker_increment_covariance_m2)
        if (covariance.shape != (9,) or not np.isfinite(covariance).all()
                or not np.allclose(covariance.reshape(3, 3),
                                   covariance.reshape(3, 3).T, rtol=1e-8, atol=1e-18)
                or np.min(np.linalg.eigvalsh(covariance.reshape(3, 3))) <= 0.0):
            raise ValueError("marker_increment_covariance_m2 must be symmetric positive definite 3x3")
        if not 0 <= self.tendon_axis < CONTROL_AXES:
            raise ValueError("tendon_axis must select a controlled shaft")
        if not np.isfinite(self.minimum_interface_rotation_rad) or self.minimum_interface_rotation_rad <= 0:
            raise ValueError("minimum_interface_rotation_rad must be positive")
        if (not np.isfinite(self.minimum_distal_marker_deformation_mm)
                or self.minimum_distal_marker_deformation_mm <= 0.0):
            raise ValueError("minimum_distal_marker_deformation_mm must be positive")
        if (not np.isfinite(self.distal_confirmation_release_ratio)
                or not 0.0 < self.distal_confirmation_release_ratio <= 1.0):
            raise ValueError("distal_confirmation_release_ratio must be in (0,1]")
        if (not np.isfinite(self.minimum_distal_bending_increment)
                or self.minimum_distal_bending_increment <= 0.0):
            raise ValueError(
                "minimum_distal_bending_increment must be positive")
        if not 0.0 <= self.width_learning_rate <= 1.0:
            raise ValueError("width_learning_rate must be in [0,1]")
        if (self.minimum_width_gain <= 0.0
                or self.maximum_width_gain < self.minimum_width_gain):
            raise ValueError("invalid backlash width bounds")
        if (not np.isfinite(self.tendon_unconfirmed_travel_limit_rad)
                or self.tendon_unconfirmed_travel_limit_rad < 0.0):
            raise ValueError("tendon_unconfirmed_travel_limit_rad must be finite and nonnegative")
        if self.engagement_confirmation_observations < 1:
            raise ValueError(
                "engagement_confirmation_observations must be positive")
        if self.provisional_rejection_observations < 1:
            raise ValueError(
                "provisional_rejection_observations must be positive")
        if (not np.isfinite(self.response_window_max_s)
                or self.response_window_max_s <= 0):
            raise ValueError("response_window_max_s must be positive")

    def unconfirmed_travel_limit_rad(self, prior_width_rad, *, axis=None):
        """Baseline hard travel budget, distinct from a belief quantile.

        Callers supply a fixed directional prior in shaft radians, never an
        observation-updated width. An explicit tendon-only experimental
        authority may substitute the fixed joint-span budget. Neither budget
        grows during take-up; projection independently enforces headroom.
        """
        width = float(prior_width_rad)
        if not np.isfinite(width) or width < 0.0:
            raise ValueError("prior take-up width must be finite and nonnegative")
        if (axis == self.tendon_axis
                and self.tendon_unconfirmed_travel_limit_rad > 0.0):
            return self.tendon_unconfirmed_travel_limit_rad
        return self.maximum_width_gain * width


@dataclass(frozen=True)
class BacklashSnapshot:
    width_rad: np.ndarray
    width_positive_rad: np.ndarray
    width_negative_rad: np.ndarray
    remaining_rad: np.ndarray
    motion_direction: np.ndarray
    engaged_direction: np.ndarray
    confidence: np.ndarray
    confirmation_count: np.ndarray
    phase: tuple[str, str, str]
    inferred_transmitted_increment_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    response_evidence: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    joint_response_residual: float = 0.0
    distal_bending_increment: float = 0.0
    tendon_distal_response_evidence: float = 0.0
    tendon_distal_response_confirmed: bool = False
    response_classification: tuple[str, str, str] = (
        "NOT_EVALUATED", "NOT_EVALUATED", "NOT_EVALUATED")
    provisional_rejection_count: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.int32))
    width_positive_lower_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    width_positive_upper_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    width_negative_lower_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    width_negative_upper_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    remaining_lower_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    remaining_upper_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    reversal_start_motor_rad: np.ndarray = field(
        default_factory=lambda: np.full(CONTROL_AXES, np.nan))
    engagement_anchor_motor_rad: np.ndarray = field(
        default_factory=lambda: np.full(CONTROL_AXES, np.nan))
    accumulated_takeup_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    effective_motor_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    effective_motor_uncertainty_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    last_evidence_timestamp_ns: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.int64))
    engaged_gain: EngagedGainSnapshot | None = None
    # Noise-qualified early response: stops geometric take-up, never grants
    # engagement or changes the stronger confirmation predicate.
    tendon_response_onset: bool = False
    tendon_response_noise_mm: float = float('nan')
    tendon_response_onset_floor_mm: float = float('nan')
    tendon_response_deformation_mm: float = 0.0
    tendon_response_chord_support_mm: float = 0.0
    tendon_response_timestamp_ns: int = 0
    tendon_response_encoder_quiet: bool = False

    @property
    def taking_up(self):
        return np.asarray([value == "TAKEUP" for value in self.phase])

    @property
    def failed(self):
        return np.asarray([value == "FAILED" for value in self.phase])

    @property
    def uncertainty_rad(self):
        return np.maximum(
            0.0, np.asarray(self.remaining_upper_rad)
            - np.asarray(self.remaining_lower_rad))


@dataclass(frozen=True)
class BacklashRollout:
    """Candidate-local transmission result for one MPPI batch."""

    transmitted_motor_radians_per_second: np.ndarray
    compensated_motor_radians_per_second: np.ndarray
    remaining_rad: np.ndarray
    motion_direction: np.ndarray
    taking_up: np.ndarray
    takeup_delay_s: np.ndarray


@dataclass(frozen=True)
class RawInterfacePlayRollout:
    """Interface rates resulting from raw commands through measured play.

    Unlike :class:`BacklashRollout`, this route never boosts or replaces the
    requested motor command. It is used when raw motion has a separate modeled
    response while the interface coordinate is still inside play (motor 2's
    frozen v171 tendon-history branch).
    """

    interface_motor_radians_per_second: np.ndarray
    remaining_rad: np.ndarray
    motion_direction: np.ndarray


def rollout_raw_interface_play(raw_motor_radians_per_second, state, step_s,
                               filtered_axes=(False, False, True)):
    """Propagate raw commands through play only on ``filtered_axes``.

    The returned rate drives the interface Jacobian. The unmodified input must
    independently drive any raw-coordinate model branch, such as v171 tendon
    history. No take-up actuator command is synthesized here.
    """
    raw = np.asarray(raw_motor_radians_per_second, dtype=np.float64)
    axes = np.asarray(filtered_axes, dtype=bool)
    if (raw.ndim != 3 or raw.shape[-1] != CONTROL_AXES
            or not np.isfinite(raw).all()):
        raise ValueError("raw motor rates must have shape (K,H,3)")
    if axes.shape != (CONTROL_AXES,):
        raise ValueError("filtered_axes must contain three booleans")
    if not np.isfinite(step_s) or step_s <= 0.0:
        raise ValueError("step_s must be positive")

    count = raw.shape[0]
    positive = np.broadcast_to(
        _vector("width_positive_rad", state.width_positive_rad,
                nonnegative=True), (count, CONTROL_AXES)).copy()
    negative = np.broadcast_to(
        _vector("width_negative_rad", state.width_negative_rad,
                nonnegative=True), (count, CONTROL_AXES)).copy()
    remaining = np.broadcast_to(
        _vector("remaining_rad", state.remaining_rad, nonnegative=True),
        (count, CONTROL_AXES)).copy()
    motion_direction = np.broadcast_to(
        np.asarray(state.motion_direction, dtype=np.int8),
        (count, CONTROL_AXES)).copy()
    interface = raw.copy()

    for step in range(raw.shape[1]):
        requested = raw[:, step]
        direction = np.sign(requested).astype(np.int8)
        moving = (direction != 0) & axes[None]
        changed = moving & (
            (motion_direction == 0) | (direction != motion_direction))
        selected_width = np.where(direction > 0, positive, negative)
        remaining[changed] = selected_width[changed]
        motion_direction[moving] = direction[moving]

        travel = np.abs(requested)*float(step_s)
        blocked = np.minimum(remaining, travel)
        blocked[:, ~axes] = 0.0
        remaining -= blocked
        passed = np.maximum(0.0, travel-blocked)
        interface[:, step] = np.where(
            axes[None], np.sign(requested)*passed/float(step_s), requested)

    return RawInterfacePlayRollout(
        interface_motor_radians_per_second=interface,
        remaining_rad=remaining,
        motion_direction=motion_direction)


def rollout_backlash_state(desired_motor_radians_per_second, state,
                           takeup_motor_radians_per_second, step_s):
    """Propagate a measured backlash state through candidate motor commands.

    ``desired_motor_radians_per_second`` is the post-engagement motion MPPI
    wants. During an active gap, the physical shaft is boosted to the bounded
    take-up rate while only travel beyond the remaining gap reaches the
    learned interface model. Every candidate owns an independent copy of the
    measured state; this function never mutates the online estimator.
    """
    desired = np.asarray(desired_motor_radians_per_second, dtype=np.float64)
    takeup = _vector(
        "takeup_motor_radians_per_second",
        takeup_motor_radians_per_second, nonnegative=True)
    if (desired.ndim != 3 or desired.shape[-1] != CONTROL_AXES
            or not np.isfinite(desired).all()):
        raise ValueError("desired motor rates must have shape (K,H,3)")
    if not np.isfinite(step_s) or step_s <= 0.0:
        raise ValueError("step_s must be positive")

    count = desired.shape[0]
    positive = np.broadcast_to(
        _vector("width_positive_rad", state.width_positive_rad,
                nonnegative=True), (count, CONTROL_AXES)).copy()
    negative = np.broadcast_to(
        _vector("width_negative_rad", state.width_negative_rad,
                nonnegative=True), (count, CONTROL_AXES)).copy()
    remaining = np.broadcast_to(
        _vector("remaining_rad", state.remaining_rad, nonnegative=True),
        (count, CONTROL_AXES)).copy()
    motion_direction = np.broadcast_to(
        np.asarray(state.motion_direction, dtype=np.int8),
        (count, CONTROL_AXES)).copy()
    phase_takeup = np.broadcast_to(
        np.asarray(state.taking_up, dtype=bool),
        (count, CONTROL_AXES)).copy()

    transmitted = np.zeros_like(desired)
    compensated = desired.copy()
    taking_history = np.zeros_like(desired, dtype=bool)
    takeup_delay = np.zeros(count, dtype=np.float64)
    charged_active = np.zeros((count, CONTROL_AXES), dtype=bool)
    for step in range(desired.shape[1]):
        requested = desired[:, step]
        direction = np.sign(requested).astype(np.int8)
        moving = direction != 0
        changed = moving & (
            (motion_direction == 0) | (direction != motion_direction))
        selected_width = np.where(direction > 0, positive, negative)
        # MPPI evaluates the desired post-engagement response immediately.
        # Separately charge the time needed to reach that response.  Existing
        # TAKEUP is charged once, while every candidate-local reversal loads a
        # new directional gap.
        active_to_charge = (
            moving & phase_takeup & ~changed & ~charged_active)
        gap_to_charge = np.where(changed, selected_width, 0.0)
        gap_to_charge = np.where(
            active_to_charge, remaining, gap_to_charge)
        safe_takeup = np.maximum(takeup, 1e-12)
        # Shafts take up concurrently under the coordinated macro. Its delay
        # is set by the slowest active gap, not the sum of independent gaps.
        takeup_delay += np.max(gap_to_charge/safe_takeup, axis=1)
        charged_active |= active_to_charge | changed
        remaining[changed] = selected_width[changed]
        phase_takeup[changed] = selected_width[changed] > 0.0
        motion_direction[moving] = direction[moving]

        # Full-speed take-up is only justified while geometric play remains.
        # ``phase_takeup`` deliberately stays asserted until measured motion
        # confirms engagement, but using that epistemic state to select the
        # actuator magnitude creates an unmodelled full-speed burst after the
        # calibrated gap has already been traversed.  During that confirmation
        # interval retain the direction latch and pass the desired
        # post-engagement rate.
        taking = moving & (remaining > 0.0)
        taking_history[:, step] = taking
        compensated_step = requested.copy()
        compensated_step[taking] = (
            direction[taking]
            * np.maximum(np.abs(requested[taking]),
                         np.broadcast_to(takeup, requested.shape)[taking]))
        compensated[:, step] = compensated_step

        travel = np.abs(compensated_step)*float(step_s)
        blocked = np.minimum(remaining, travel)
        remaining -= blocked
        passed = np.maximum(0.0, travel-blocked)
        transmitted[:, step] = np.sign(compensated_step)*passed/float(step_s)
        # Exhausting geometric play permits transmission, but it does not
        # prove engagement. Keep the candidate phase in TAKEUP for direction
        # reasoning; only the high-speed magnitude boost ends here.

    return BacklashRollout(
        transmitted_motor_radians_per_second=transmitted,
        compensated_motor_radians_per_second=compensated,
        remaining_rad=remaining,
        motion_direction=motion_direction,
        taking_up=taking_history,
        takeup_delay_s=takeup_delay)


class BacklashStateEstimator:
    """Estimate engagement from causal raw-shaft and UKF interface motion."""

    def __init__(self, config: BacklashConfig,
                 engaged_gain_config: EngagedGainConfig | None = None):
        self.config = config
        self.engaged_gain_estimator = EngagedGainEstimator(
            EngagedGainConfig() if engaged_gain_config is None
            else engaged_gain_config)
        symmetric = _vector(
            "width_rad", config.width_rad, nonnegative=True).copy()
        self.prior_width_positive = (
            symmetric.copy() if config.width_positive_rad is None else
            _vector("width_positive_rad", config.width_positive_rad,
                    nonnegative=True).copy())
        self.prior_width_negative = (
            symmetric.copy() if config.width_negative_rad is None else
            _vector("width_negative_rad", config.width_negative_rad,
                    nonnegative=True).copy())
        self.width_positive = self.prior_width_positive.copy()
        self.width_negative = self.prior_width_negative.copy()
        self.width_positive_lower = (
            config.minimum_width_gain*self.prior_width_positive)
        self.width_positive_upper = (
            config.maximum_width_gain*self.prior_width_positive)
        self.width_negative_lower = (
            config.minimum_width_gain*self.prior_width_negative)
        self.width_negative_upper = (
            config.maximum_width_gain*self.prior_width_negative)
        # At process start the loaded encoder position is known but the side
        # of each transmission gap is not. The first observed motor direction
        # selects its directional width and starts one bounded take-up budget.
        self.remaining = np.zeros(CONTROL_AXES)
        self.remaining_lower = np.zeros(CONTROL_AXES)
        self.remaining_upper = np.zeros(CONTROL_AXES)
        self.motion_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.engaged_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.confidence = np.zeros(CONTROL_AXES)
        self.confirmation_count = np.zeros(CONTROL_AXES, dtype=np.int32)
        self.confirmation_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.provisional_rejection_count = np.zeros(
            CONTROL_AXES, dtype=np.int32)
        self.phase = ["UNKNOWN"]*CONTROL_AXES
        self.external_motion = False
        self.track_external_response = False
        self.accumulated_since_reversal = np.zeros(CONTROL_AXES)
        self.reversal_start_motor = np.full(CONTROL_AXES, np.nan)
        self.engagement_anchor_motor = np.full(CONTROL_AXES, np.nan)
        self.last_evidence_timestamp_ns = np.zeros(
            CONTROL_AXES, dtype=np.int64)
        self.reversal_observed = np.zeros(CONTROL_AXES, dtype=bool)
        self.previous_raw_motor = None
        self.pending_raw_delta = np.zeros(CONTROL_AXES)
        self.effective_motor = None
        self.previous_response_motor = None
        self.previous_pose = None
        self.previous_strain = None
        self.distal_marker_anchor = None
        self.distal_marker_history = []
        self.distal_marker_motor = None
        self.distal_marker_direction = 0
        self.distal_marker_timestamp_ns = None
        self.distal_response_encoder_quiet = False
        self.distal_marker_motor_anchor = None
        self.distal_quiet_history = []
        self.distal_onset_noise = None
        self.distal_noise_observation_timestamp_ns = None
        self.distal_response_deformation_mm = 0.0
        self.distal_response_chord_support_mm = 0.0
        self.tendon_response_onset = False
        self.response_window_timestamp_ns = None
        self.response_observation_timestamp_ns = None
        self.response_sample_motor = None
        self.response_sample_pose = None
        self.response_sample_strain = None
        self.marker_anchor_history = []
        self.marker_response_anchor = None
        self.measured_response_direction = np.zeros((CONTROL_AXES, 3))
        self.response_window_reset_required = False
        self.inferred_transmitted_increment = np.zeros(CONTROL_AXES)
        self.response_evidence = np.zeros(CONTROL_AXES)
        self.joint_response_residual = 0.0
        self.distal_bending_increment = 0.0
        self.tendon_distal_response_evidence = 0.0
        self.tendon_distal_response_confirmed = False
        self.response_classification = ["NOT_EVALUATED"]*CONTROL_AXES

    def reset_observation_history(self, *, preserve_noise_calibration=False):
        """Reset transaction evidence; sensor invalidation also resets noise.

        Homing/command ownership is not a camera calibration change. Only
        those callers preserve the already-qualified noise scale, never the
        old direction, response anchor, or pending confirmation evidence.
        """
        noise = self.distal_onset_noise if preserve_noise_calibration else None
        noise_timestamp = (self.distal_noise_observation_timestamp_ns
                           if preserve_noise_calibration else None)
        self.previous_response_motor = None
        self.previous_pose = None
        self.previous_strain = None
        self.distal_marker_anchor = None
        self.distal_marker_history = []
        self.distal_marker_motor = None
        self.distal_marker_direction = 0
        self.distal_marker_timestamp_ns = None
        self.distal_response_encoder_quiet = False
        self.distal_marker_motor_anchor = None
        self.distal_quiet_history = []
        self.distal_onset_noise = noise
        self.distal_noise_observation_timestamp_ns = noise_timestamp
        self.distal_response_deformation_mm = 0.0
        self.distal_response_chord_support_mm = 0.0
        self.tendon_response_onset = False
        self.response_window_timestamp_ns = None
        self.response_observation_timestamp_ns = None
        self.response_sample_motor = None
        self.response_sample_pose = None
        self.response_sample_strain = None
        self.marker_anchor_history = []
        self.marker_response_anchor = None
        self.response_window_reset_required = False
        self.measured_response_direction = np.zeros((CONTROL_AXES, 3))

    def clone_state(self):
        """Deep-copy the belief for timestamp-aligned rewind/replay."""
        return copy.deepcopy(self.__dict__)

    def _observe_distal_descriptor(self, descriptor, motor, timestamp_ns):
        """Bounded causal raw geometry, owned by the tendon transaction.

        Proximal window rebases and other shaft reversals cannot erase tendon
        evidence. Three current frames reject isolated spikes. A new direction
        anchors at its first source-time image/motor pair, never a median of
        pre-reversal motion. Up to five past frames smooth same-direction roots.
        """
        axis = self.config.tendon_axis
        direction = int(self.motion_direction[axis])
        previous_stamp = self.distal_marker_timestamp_ns
        gap = (timestamp_ns is not None and previous_stamp is not None
               and (timestamp_ns < previous_stamp or
                    timestamp_ns-previous_stamp >= self.config.response_window_max_s*1e9))
        changed = direction != self.distal_marker_direction
        pending = self.phase[axis] in ("TAKEUP", "PROVISIONAL")
        noise_stamp = self.distal_noise_observation_timestamp_ns
        noise_gap = (timestamp_ns is not None and noise_stamp is not None
                     and (timestamp_ns < noise_stamp or timestamp_ns-noise_stamp
                          >= self.config.response_window_max_s*1e9))
        if gap or noise_gap:
            self.distal_quiet_history = []
            self.distal_onset_noise = None
        self.distal_noise_observation_timestamp_ns = timestamp_ns
        if changed or gap or not pending:
            self.tendon_response_onset = False
        # Estimate raw distance noise only while the encoder is quiet. Freeze
        # it at a reversal; moving deformation must not inflate its own gate.
        quiet = (self.distal_marker_motor is not None and
                 abs(float(motor[axis])-self.distal_marker_motor)
                 < self.config.minimum_motor_increment_rad)
        self.distal_response_encoder_quiet = bool(quiet)
        if quiet and not pending and timestamp_ns is not None:
            self.distal_quiet_history.append((timestamp_ns, descriptor.copy()))
            self.distal_quiet_history = self.distal_quiet_history[-12:]
            history = self.distal_quiet_history
            if (len(history) >= 8
                    and history[-1][0]-history[0][0] >= 0.2e9):
                times = np.array([t for t, _ in history], dtype=np.int64)
                seconds = (times-times[0])*1e-9
                values = np.array([v for _, v in history])
                smooth = np.array([np.median(values[k-2:k+1], axis=0)
                                   for k in range(2, len(values))])
                centered = seconds[2:]-seconds[2:].mean()
                slope = centered@smooth/(centered@centered)
                # An encoder hold is not necessarily a shape hold. Qualify
                # only windows with drift smaller than the existing onset
                # resolution, and estimate scatter after removing that drift.
                # Resolved settling must neither qualify nor inflate measurement
                # noise. No model prediction or Jacobian enters this test.
                drift = np.max(np.abs(slope))*(seconds[-1]-seconds[0])
                if drift <= self.config.minimum_distal_marker_deformation_mm/3.:
                    residual = smooth-centered[:, None]*slope[None, :]
                    self.distal_onset_noise = float(np.max(np.std(residual, axis=0)))
        elif not quiet:
            self.distal_quiet_history = []
        if changed:
            self.distal_quiet_history = []
        # External homing can traverse slack during a marker gap. Preserve
        # only its same-direction transaction reference, not stale response
        # samples or confirmation counts. Ownership/reversal resets still
        # invalidate the reference, and rewinds never bridge a gap.
        retain_external_anchor = (
            gap and timestamp_ns > previous_stamp and self.external_motion
            and pending and not changed and direction != 0
            and self.distal_marker_anchor is not None
            and self.distal_marker_motor is not None
            and direction*(float(motor[axis])-self.distal_marker_motor)
                >= self.config.minimum_motor_increment_rad)
        if gap:
            self.distal_marker_history = []
            if not retain_external_anchor:
                self.distal_marker_anchor = None
                self.distal_marker_motor_anchor = None
                self.distal_marker_motor = None
            self.confirmation_count[axis] = 0
        if self.distal_marker_anchor is None or changed or not pending:
            self.distal_marker_anchor = (
                np.median(self.distal_marker_history, axis=0)
                if self.distal_marker_history and not changed else descriptor.copy())
            self.distal_marker_motor_anchor = (
                float(motor[axis]) if self.distal_marker_motor is None or changed
                else self.distal_marker_motor)
        if changed:
            self.distal_marker_history = []
        self.distal_marker_history.append(descriptor.copy())
        self.distal_marker_history = self.distal_marker_history[-5:]
        self.distal_marker_direction = direction
        self.distal_marker_motor = float(motor[axis])
        self.distal_marker_timestamp_ns = timestamp_ns
        deformation = (0.0 if len(self.distal_marker_history) < 3 else
            float(np.sqrt(np.mean((np.median(self.distal_marker_history[-3:], axis=0)
                                   - self.distal_marker_anchor)**2))))
        return deformation, float(motor[axis])-self.distal_marker_motor_anchor

    def restore_state(self, checkpoint):
        """Restore a checkpoint produced by clone_state."""
        if not isinstance(checkpoint, dict):
            raise ValueError("backlash checkpoint must be a dictionary")
        config = self.config
        self.__dict__.clear()
        self.__dict__.update(copy.deepcopy(checkpoint))
        self.config = config

    def _raw_tendon_evidence(self, deformation, motor_delta):
        """One raw-geometry predicate for stop previews and full confirmation."""
        axis = self.config.tendon_axis
        floor = self.config.minimum_distal_marker_deformation_mm
        pending = self.phase[axis] in ("TAKEUP", "PROVISIONAL")
        moving = abs(motor_delta) >= self.config.minimum_motor_increment_rad
        persistence = (self.phase[axis] == "PROVISIONAL"
                       and self.confirmation_count[axis] > 0
                       and self.confirmation_direction[axis] == self.motion_direction[axis])
        confirmation_floor = floor*self.config.distal_confirmation_release_ratio if persistence else floor
        chord_delta = (0.0 if len(self.distal_marker_history) < 3 else
            float(np.median(self.distal_marker_history[-3:], axis=0)[2]
                  - self.distal_marker_anchor[2]))
        chord_support = (-np.sign(MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[axis])
                         * self.motion_direction[axis] * chord_delta)
        self.distal_response_deformation_mm = float(deformation)
        self.distal_response_chord_support_mm = float(chord_support)
        if self.distal_onset_noise is not None:
            early_floor = max(floor/3., 3.*self.distal_onset_noise)
            self.tendon_response_onset |= bool(
                self.config.distal_confirmation_enabled and pending and moving
                and deformation >= early_floor and chord_support >= early_floor)
        return bool(self.config.distal_confirmation_enabled and pending and moving
                    and deformation >= confirmation_floor and chord_support >= confirmation_floor)

    @staticmethod
    def _source_motor(history, timestamp_ns, control_owned):
        entries = list(history)
        index = next((i for i in range(len(entries)-1, -1, -1)
                      if entries[i][0] <= int(timestamp_ns)), None)
        if index is None or bool(control_owned) != bool(entries[index][3]):
            return None
        motor = entries[index][1]
        if entries[index][0] != int(timestamp_ns):
            if index+1 >= len(entries):
                return None
            following = entries[index+1]
            span = following[0]-entries[index][0]
            if bool(following[3]) != bool(entries[index][3]) or not 0 < span <= 250_000_000:
                return None
            fraction = (int(timestamp_ns)-entries[index][0])/span
            motor = motor+fraction*(following[1]-motor)
        return entries, index, motor

    def preview_tendon_onset(self, history, timestamp_ns, points, *, control_owned):
        """Stop-only evidence on an isolated source checkpoint; never confirm.

        Invoked by the sole estimator owner before expensive shape correction.
        The live belief and replay history are untouched, including noise and
        persistence counts. Full correction consumes the observation once.
        """
        aligned = BacklashStateEstimator._source_motor(history, timestamp_ns, control_owned)
        if aligned is None:
            return None
        entries, index, motor = aligned
        scratch = copy.copy(self)
        scratch.restore_state(entries[index][2])
        if (scratch.distal_marker_timestamp_ns is not None
                and timestamp_ns <= scratch.distal_marker_timestamp_ns):
            return None
        points = np.asarray(points, dtype=np.float64)
        if points.shape != (4, 3) or not np.isfinite(points).all():
            return None
        descriptor = np.array([np.linalg.norm(points[i]-points[j])*1000.
                               for i in range(4) for j in range(i+1, 4)])
        deformation, delta = scratch._observe_distal_descriptor(descriptor, motor, timestamp_ns)
        scratch._raw_tendon_evidence(deformation, delta)
        return scratch.snapshot()

    def snapshot(self):
        active_width = np.where(
            self.motion_direction > 0, self.width_positive,
            np.where(self.motion_direction < 0, self.width_negative,
                     np.maximum(self.width_positive, self.width_negative)))
        return BacklashSnapshot(
            active_width.copy(), self.width_positive.copy(),
            self.width_negative.copy(), self.remaining.copy(),
            self.motion_direction.copy(), self.engaged_direction.copy(),
            self.confidence.copy(), self.confirmation_count.copy(),
            tuple(self.phase), self.inferred_transmitted_increment.copy(),
            self.response_evidence.copy(),
            float(self.joint_response_residual),
            float(self.distal_bending_increment),
            float(self.tendon_distal_response_evidence),
            bool(self.tendon_distal_response_confirmed),
            tuple(self.response_classification),
            self.provisional_rejection_count.copy(),
            self.width_positive_lower.copy(),
            self.width_positive_upper.copy(),
            self.width_negative_lower.copy(),
            self.width_negative_upper.copy(),
            self.remaining_lower.copy(), self.remaining_upper.copy(),
            self.reversal_start_motor.copy(),
            self.engagement_anchor_motor.copy(),
            self.accumulated_since_reversal.copy(),
            (np.zeros(CONTROL_AXES) if self.effective_motor is None else
             self.effective_motor.copy()),
            0.5*np.maximum(0.0, self.remaining_upper-self.remaining_lower),
            self.last_evidence_timestamp_ns.copy(),
            self.engaged_gain_estimator.snapshot(),
            bool(self.tendon_response_onset),
            (float('nan') if self.distal_onset_noise is None else
             float(self.distal_onset_noise)),
            (float('nan') if self.distal_onset_noise is None else
             max(self.config.minimum_distal_marker_deformation_mm/3.,
                 3.*self.distal_onset_noise)),
            float(self.distal_response_deformation_mm),
            float(self.distal_response_chord_support_mm),
            int(self.distal_marker_timestamp_ns or 0),
            bool(getattr(self, 'distal_response_encoder_quiet', False)))

    def _width_for_direction(self, axis, direction, *, prior=False):
        if direction > 0:
            return ((self.prior_width_positive if prior
                     else self.width_positive), axis)
        return ((self.prior_width_negative if prior
                 else self.width_negative), axis)

    def _width_interval_for_direction(self, axis, direction):
        if direction > 0:
            return (self.width_positive_lower, self.width_positive_upper, axis)
        return (self.width_negative_lower, self.width_negative_upper, axis)

    def _start_takeup(self, axis, direction, motor_position):
        if axis == self.config.tendon_axis:
            self.tendon_response_onset = False
        widths, index = self._width_for_direction(axis, direction)
        lower, upper, _ = self._width_interval_for_direction(axis, direction)
        self.remaining[axis] = widths[index]
        self.remaining_lower[axis] = lower[index]
        self.remaining_upper[axis] = upper[index]
        self.accumulated_since_reversal[axis] = 0.0
        self.reversal_start_motor[axis] = float(motor_position)
        self.engagement_anchor_motor[axis] = np.nan
        self.phase[axis] = "TAKEUP"
        self.confirmation_count[axis] = 0
        self.confirmation_direction[axis] = 0
        self.provisional_rejection_count[axis] = 0
        self.engaged_gain_estimator.start_takeup(axis, direction)

    def _learn_confirmed_boundary(self, axis):
        """Use confirmed first-response travel as an upper bound on the gap.

        Distal response need not move the interface. Neither response source
        identifies how much of the travel was already transmitted, so retain
        the full travel to the first response rather than subtracting an
        unrelated interface fit. Commit only on confirmed real reversals;
        startup direction selection does not identify a full directional gap.
        """
        rate = self.config.width_learning_rate
        start = self.reversal_start_motor[axis]
        anchor = self.engagement_anchor_motor[axis]
        if (not self.reversal_observed[axis] or rate == 0.0
                or not np.isfinite(start) or not np.isfinite(anchor)):
            return
        direction = int(self.confirmation_direction[axis])
        travel = direction*(anchor-start)
        if direction == 0 or travel < self.config.minimum_motor_increment_rad:
            return
        widths, index = self._width_for_direction(axis, direction)
        lower, upper, _ = self._width_interval_for_direction(axis, direction)
        # A prior floor must not override a confirmed smaller boundary. Slow
        # blending remains appropriate for increases, but a gap cannot exceed
        # the travel at which this same belief confirmed transmission.
        widths[index] = min((1.0-rate)*widths[index]+rate*travel, travel)
        slack = self.config.minimum_motor_increment_rad
        lower[index] = min(widths[index],
            (1.0-rate)*lower[index]+rate*max(0.0, travel-slack))
        upper[index] = max(widths[index], min(
            (1.0-rate)*upper[index]+rate*(travel+slack), travel+slack))

    def advance_motor(self, motor_angle_rad, *, external_motion=False,
                      track_external_response=False):
        """Advance raw shaft state and return estimated transmitted angles.

        The learned runtime must not integrate motion that is still consuming
        a known transmission gap. Raw encoder travel remains authoritative for
        gap bookkeeping; only travel beyond that gap advances the virtual
        motor angle supplied to the learned proximal/distal model.
        """
        motor = _vector("motor_angle_rad", motor_angle_rad)
        if track_external_response and not external_motion:
            raise ValueError("external response tracking requires external motion ownership")
        ownership_changed = bool(external_motion) != self.external_motion
        tracking_changed = bool(track_external_response) != getattr(self, "track_external_response", False)
        if ownership_changed or tracking_changed:
            # Start a fresh evidence window at either ownership boundary.
            # Physical homing evidence conditions inference, but never hands
            # an external transaction to the MPPI arbiter.
            axes = (0, self.config.tendon_axis)
            for axis in axes:
                if self.phase[axis] == "FAILED":
                    continue  # Explicit calibration cannot clear a failure.
                self.phase[axis] = "UNKNOWN"
                self.motion_direction[axis] = self.engaged_direction[axis] = 0
                self.confirmation_count[axis] = self.confirmation_direction[axis] = 0
                self.last_evidence_timestamp_ns[axis] = 0
                self.engagement_anchor_motor[axis] = np.nan
                self.reversal_start_motor[axis] = np.nan
                self.confidence[axis] = 0.
            self.reset_observation_history(preserve_noise_calibration=True)
        self.external_motion = bool(external_motion)
        self.track_external_response = bool(track_external_response)
        if self.previous_raw_motor is None:
            self.previous_raw_motor = motor.copy()
            self.effective_motor = motor.copy()
            return self.effective_motor.copy()

        self.pending_raw_delta += motor-self.previous_raw_motor
        self.previous_raw_motor = motor.copy()
        for axis in range(CONTROL_AXES):
            delta = float(self.pending_raw_delta[axis])
            if abs(delta) < self.config.minimum_motor_increment_rad:
                continue
            self.pending_raw_delta[axis] = 0.0
            direction = int(np.sign(delta))
            if external_motion:
                if axis == self.config.tendon_axis or (self.track_external_response and axis == 0):
                    if self.track_external_response and self.phase[axis] == "FAILED":
                        self.effective_motor[axis] += delta
                        continue
                    # Reuse signed distal confirmation during homing; explicit
                    # compensated calibration also tracks chassis response.
                    # TAKEUP here means unconfirmed physical transmission, not
                    # an MPPI-owned command transaction or travel budget.
                    if ownership_changed or tracking_changed or direction != self.motion_direction[axis]:
                        self._start_takeup(axis, direction, motor[axis]-delta)
                        self.reset_observation_history(preserve_noise_calibration=True)
                    self.motion_direction[axis] = direction
                    self.accumulated_since_reversal[axis] += abs(delta)
                    self.effective_motor[axis] += delta
                    self.remaining[axis] = self.remaining_lower[axis] = 0.
                    self.remaining_upper[axis] = 0.
                    continue
                # Externally owned position moves (homing) are not bounded
                # MPPI take-up transactions. Preserve coordinates and learned
                # widths, but do not carry transaction evidence into control.
                self.effective_motor[axis] += delta
                self.motion_direction[axis] = 0
                self.engaged_direction[axis] = 0
                self.phase[axis] = "UNKNOWN"
                self.confidence[axis] = 0.0
                self.confirmation_count[axis] = 0
                self.confirmation_direction[axis] = 0
                self.provisional_rejection_count[axis] = 0
                self.accumulated_since_reversal[axis] = 0.0
                self.reversal_start_motor[axis] = np.nan
                self.engagement_anchor_motor[axis] = np.nan
                self.last_evidence_timestamp_ns[axis] = 0
                self.reversal_observed[axis] = False
                self.inferred_transmitted_increment[axis] = 0.0
                self.response_evidence[axis] = 0.0
                self.remaining[axis] = 0.0
                self.remaining_lower[axis] = 0.0
                self.remaining_upper[axis] = max(
                    self.width_positive_upper[axis],
                    self.width_negative_upper[axis])
                self.response_classification[axis] = "NOT_EVALUATED"
                if ownership_changed:
                    self.reset_observation_history(preserve_noise_calibration=True)
                self.engaged_gain_estimator.start_takeup(axis, direction)
                continue
            # Zero prior chassis width is not proof of transmission after an
            # ownership handoff. The arbiter needs fresh directional response.
            if (self.motion_direction[axis] == 0
                    and self.engaged_direction[axis] == 0
                    and (self._width_for_direction(
                        axis, direction)[0][axis] > 0.0 or axis == 0)):
                self._start_takeup(axis, direction, motor[axis]-delta)
            elif (self.motion_direction[axis] != 0
                  and direction != self.motion_direction[axis]):
                self._start_takeup(axis, direction, motor[axis]-delta)
                self.reversal_observed[axis] = True
                self.confidence[axis] *= 0.5
                self.response_window_reset_required = True
            self.motion_direction[axis] = direction

            travel = abs(delta)
            passed = travel
            if self.phase[axis] in ("TAKEUP", "PROVISIONAL"):
                self.accumulated_since_reversal[axis] += travel
            if self.phase[axis] == "TAKEUP":
                blocked = min(self.remaining[axis], travel)
                self.remaining[axis] = max(
                    0.0, self.remaining[axis]-blocked)
                self.remaining_lower[axis] = max(
                    0.0, self.remaining_lower[axis]-travel)
                self.remaining_upper[axis] = max(
                    0.0, self.remaining_upper[axis]-travel)
                passed -= blocked
            self.effective_motor[axis] += direction*passed
        return self.effective_motor.copy()

    @staticmethod
    def _bounded_joint_fit(columns, measured, delta, moving,
                           minimum_transmitted_increment):
        """Infer simultaneous transmitted shaft increments.

        Each active coefficient has the sign of its raw encoder motion and is
        bounded by that motion. With only three proximal shafts an exhaustive
        active-set solve is cheap and deterministic. A small per-column cost
        prevents camera noise from becoming tiny motion on every shaft.
        """
        active = np.flatnonzero(moving)
        inferred = np.zeros(CONTROL_AXES, dtype=np.float64)
        evidence = np.zeros(CONTROL_AXES, dtype=np.float64)
        if active.size == 0:
            return inferred, evidence, float(np.linalg.norm(measured))

        signs = np.sign(delta[active])
        basis = columns[:, active]*signs[np.newaxis, :]
        bounds = np.abs(delta[active])
        column_norm = np.linalg.norm(basis, axis=0)
        noise = np.maximum(
            0.25*minimum_transmitted_increment*column_norm, 1e-9)
        candidates = []
        # 0: fixed at zero, 1: free, 2: fixed at the raw-motion upper bound.
        # Enumerating all 3^3 states gives bounded least squares without a
        # SciPy dependency.
        for states in product((0, 1, 2), repeat=active.size):
            q = np.zeros(active.size, dtype=np.float64)
            states = np.asarray(states)
            upper = states == 2
            free = states == 1
            q[upper] = bounds[upper]
            residual_target = measured-basis[:, upper]@q[upper]
            if np.any(free):
                solution, *_ = np.linalg.lstsq(
                    basis[:, free], residual_target, rcond=None)
                if (np.any(solution < -1e-10)
                        or np.any(solution > bounds[free]+1e-10)):
                    continue
                q[free] = np.clip(solution, 0.0, bounds[free])
            residual = measured-basis@q
            selected = q >= minimum_transmitted_increment
            score = float(residual@residual + np.sum(noise[selected]**2))
            candidates.append((score, q, residual))

        if not candidates:
            return inferred, evidence, float(np.linalg.norm(measured))
        score, q, residual = min(candidates, key=lambda item: item[0])
        inferred[active] = signs*q
        residual_norm = float(np.linalg.norm(residual))
        # Evidence is the normalized degradation of the best fit when an axis
        # is forbidden. It exposes ambiguous attribution between correlated
        # Jacobian columns instead of arbitrarily selecting the largest input.
        for local_axis, axis in enumerate(active):
            without = [item[0] for item in candidates
                       if item[1][local_axis]
                       < minimum_transmitted_increment]
            if not without:
                continue
            improvement = max(0.0, min(without)-score)
            evidence[axis] = 1.0-np.exp(
                -improvement/max(2.0*noise[local_axis]**2, 1e-18))
        return inferred, evidence, residual_norm

    def interface_motor_angles(self, raw_motor_angle_rad):
        """Separate direct knob/interface drive from distal tendon take-up.

        Chassis and rotation retain their estimated transmitted coordinates.
        Shaft2's tendon state must not suppress the near-backlash-free knob
        contribution to interface motion. Raw tendon history is unchanged.
        """
        raw = _vector("raw_motor_angle_rad", raw_motor_angle_rad)
        if self.effective_motor is None:
            raise RuntimeError("advance motor before requesting interface coordinates")
        interface = self.effective_motor.copy()
        interface[2] = raw[2]
        return interface

    def observe_response(self, raw_motor_angle_rad, interface_pose,
                         jacobian, state_scale, distal_strain=None,
                         distal_bending_mode=None, timestamp_ns=None,
                         nominal_distal_lambda=None,
                         observed_distal_lambda=None, *,
                         observed_interface_position_m=None,
                         observed_marker_points_m=None):
        """Confirm engagement from measured response, independently of J.

        ``raw_motor_angle_rad`` must be the physical encoder sample paired to
        the camera timestamp. Nominal widths predict virtual transmission in
        :meth:`advance_motor`; observed interface or distal response confirms it.
        When supplied, the measured interface-origin marker replaces fitted
        translation for response evidence only. Fitted orientation is retained.
        This does not modify the filter posterior or its planning state.
        Jacobian-derived coefficients/evidence are diagnostics only. Interface
        evidence uses interface-minus-direct-knob translation for chassis.
        Insertion/rotation simultaneity remains inconclusive. Tendon evidence
        requires distal bending independently of direct knob translation.
        Both ROS routes supply raw four-marker points; their six-distance RMS
        uses a millimeter threshold instead of model-dependent strain units.
        On the supported ordinary-bending branch, the interface-tip chord must
        also change in the physical pull/release direction before confirmation.
        """
        motor = _vector(
            "raw_motor_angle_rad", raw_motor_angle_rad)
        descriptor = None
        if observed_marker_points_m is not None:
            points = np.asarray(observed_marker_points_m, dtype=np.float64)
            if points.shape != (4, 3) or not np.all(np.isfinite(points)):
                raise ValueError("observed_marker_points_m must be finite 4x3 markers")
            descriptor = np.array([
                np.linalg.norm(points[i]-points[j])*1000.0
                for i in range(4) for j in range(i+1, 4)])
        pose = np.asarray(interface_pose, dtype=np.float64)
        matrix = np.asarray(jacobian, dtype=np.float64)
        scale = np.asarray(state_scale, dtype=np.float64)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("interface_pose must be a finite 4x4 transform")
        if observed_interface_position_m is not None:
            position = _vector(
                "observed_interface_position_m", observed_interface_position_m)
            # Marker 0 is at the modeled interface origin. A posterior origin
            # residual must not masquerade as transmitted translation. Use
            # the same measured position in cumulative and incremental windows,
            # including provisional persistence, rewind and clone/restore.
            pose = pose.copy()
            pose[:3, 3] = position
        if matrix.shape != (6, CONTROL_AXES):
            raise ValueError("jacobian must be 6x3")
        if scale.shape != (6,) or np.any(scale <= 0.0):
            raise ValueError("state_scale must contain six positive values")
        if (distal_strain is None) != (distal_bending_mode is None):
            raise ValueError(
                "distal strain and bending mode must be supplied together")
        if ((nominal_distal_lambda is None)
                != (observed_distal_lambda is None)):
            raise ValueError(
                "nominal and observed distal lambda must be supplied together")
        strain = None
        bending_mode = None
        if distal_strain is not None:
            strain = np.asarray(distal_strain, dtype=np.float64).reshape(-1)
            bending_mode = np.asarray(
                distal_bending_mode, dtype=np.float64).reshape(-1)
            if (strain.shape != bending_mode.shape or strain.size == 0
                    or not np.isfinite(strain).all()
                    or not np.isfinite(bending_mode).all()):
                raise ValueError(
                    "distal strain and bending mode must be equally sized "
                    "finite vectors")
            mode_norm = float(np.linalg.norm(bending_mode))
            if mode_norm <= 1e-12:
                raise ValueError("distal bending mode must be nonzero")
            bending_mode = bending_mode/mode_norm
        def observe_engaged_gain():
            if nominal_distal_lambda is None or self.external_motion:
                return
            axis = self.config.tendon_axis
            direction = int(self.engaged_direction[axis])
            if direction == 0:
                direction = int(self.motion_direction[axis])
            self.engaged_gain_estimator.observe(
                axis, direction, nominal_distal_lambda,
                observed_distal_lambda, timestamp_ns,
                engaged=(self.phase[axis] == "ENGAGED"
                         and not np.isin(self.phase, ["TAKEUP", "PROVISIONAL"]).any()),
                motor_position=motor[axis],
                minimum_motor_travel=self.config.minimum_transmitted_increment_rad)

        self.response_classification = ["NOT_EVALUATED"]*CONTROL_AXES
        if (timestamp_ns is not None
                and self.response_observation_timestamp_ns is not None
                and timestamp_ns == self.response_observation_timestamp_ns):
            return self.snapshot()
        raw_deformation, raw_tendon_delta = (0.0, 0.0)
        if descriptor is not None:
            raw_deformation, raw_tendon_delta = self._observe_distal_descriptor(
                descriptor, motor, timestamp_ns)
        # The age cap bounds the search for a first response, not the atomic
        # confirmation hold. Rebasing a provisional shaft onto its stopped
        # motor erases the very displacement whose persistence we must test.
        # The arbiter's existing confirmation timeout bounds that hold instead.
        rewound = (timestamp_ns is not None
                   and self.response_observation_timestamp_ns is not None
                   and timestamp_ns < self.response_observation_timestamp_ns)
        expired = (timestamp_ns is not None
                   and self.response_window_timestamp_ns is not None
                   and timestamp_ns-self.response_window_timestamp_ns
                   >= self.config.response_window_max_s*1e9
                   and "PROVISIONAL" not in self.phase)
        # Bound the anchor history, not the observations themselves. Only a
        # timed search rebase uses the five-frame median: never carry earlier
        # motion across a reversal, invalidation or clock rewind, or smooth
        # the current response / filter posterior. A provisional anchor stays
        # frozen through confirmation, exactly as before.
        if (self.previous_response_motor is None
                or self.response_window_reset_required or rewound):
            self.marker_anchor_history = []
        # Opposing knob/chassis travel can cancel at a static interface.
        # Subtract the fixed direct drive before accumulating robust anchors;
        # raw pose remains intact for rotation and filter diagnostics.
        chassis_position = pose[:3, 3]-knob_interface_translation_m(motor[2])
        self.marker_anchor_history.append(chassis_position.copy())
        self.marker_anchor_history = self.marker_anchor_history[-5:]
        if (self.previous_response_motor is None
                or self.response_window_reset_required or expired or rewound):
            if expired or rewound:
                for axis, phase in enumerate(self.phase):
                    if phase == "PROVISIONAL":
                        self.confirmation_count[axis] = 1
            self.previous_response_motor = motor.copy()
            self.previous_pose = pose.copy()
            self.marker_response_anchor = (
                np.median(self.marker_anchor_history, axis=0)
                if expired and len(self.marker_anchor_history) == 5
                else chassis_position.copy())
            self.previous_strain = (
                None if strain is None else strain.copy())
            self.response_window_reset_required = False
            self.response_window_timestamp_ns = timestamp_ns
            self.response_observation_timestamp_ns = timestamp_ns
            self.response_sample_motor = motor.copy()
            self.response_sample_pose = pose.copy()
            self.response_sample_strain = None if strain is None else strain.copy()
            self.measured_response_direction = np.zeros((CONTROL_AXES, 3))
            self.distal_bending_increment = 0.0
            self.tendon_distal_response_evidence = 0.0
            self.tendon_distal_response_confirmed = False
            observe_engaged_gain()
            if (descriptor is None or self.phase[self.config.tendon_axis]
                    not in ("TAKEUP", "PROVISIONAL")
                    or abs(raw_tendon_delta) < self.config.minimum_motor_increment_rad):
                return self.snapshot()
        self.response_observation_timestamp_ns = timestamp_ns
        sample_delta = motor-self.response_sample_motor
        sample_pose_anchor = self.response_sample_pose
        sample_relative = np.linalg.inv(sample_pose_anchor)@pose
        sample_response = _se3_log(
            sample_relative)/scale
        sample_distal = (0.0 if strain is None
                         or self.response_sample_strain is None else
                         float((strain-self.response_sample_strain)@bending_mode))
        self.response_sample_motor = motor.copy()
        self.response_sample_pose = pose.copy()
        self.response_sample_strain = None if strain is None else strain.copy()

        # Response evidence is intentionally accumulated across camera frames.
        # A per-frame shaft increment can be smaller than the response floor
        # even during continuous transmission, especially at the 20 Hz UKF
        # correction rate.  Resetting the pose/motor reference on every such
        # frame delays engagement until a scheduling gap happens to make one
        # increment large enough. Keep a common SE(3) window until at least
        # one pending shaft has enough causal evidence to be observable.
        delta = motor-self.previous_response_motor
        if descriptor is not None:
            delta[self.config.tendon_axis] = raw_tendon_delta
        relative = np.linalg.inv(self.previous_pose)@pose
        response = _se3_log(relative)
        moving = np.abs(delta) >= self.config.minimum_motor_increment_rad
        interface_moving = moving.copy()
        interface_moving[self.config.tendon_axis] = False
        isolated = np.count_nonzero(interface_moving) == 1
        pending = np.asarray(
            [phase in ("TAKEUP", "PROVISIONAL")
             for phase in self.phase], dtype=bool)
        # Retain the pre-response anchor during confirmation hold: each new
        # shape must still support the observed displacement. Stationary
        # encoders alone cannot turn a transient shape spike into engagement.

        distal_increment = 0.0
        distal_evidence = 0.0
        distal_confirmed = False
        if strain is not None and self.previous_strain is not None:
            distal_increment = float(
                (strain-self.previous_strain)@bending_mode)
            distal_evidence = float(np.clip(
                abs(distal_increment)
                / self.config.minimum_distal_bending_increment,
                0.0, 1.0))
            tendon_axis = self.config.tendon_axis
            distal_confirmed = bool(
                self.config.distal_confirmation_enabled
                and moving[tendon_axis] and pending[tendon_axis]
                and abs(distal_increment)
                >= self.config.minimum_distal_bending_increment)
        self.distal_bending_increment = distal_increment
        self.tendon_distal_response_evidence = distal_evidence
        self.tendon_distal_response_confirmed = distal_confirmed
        # Raw geometry is the production evidence, independent of fitted
        # strain and J. Six distances remove rigid insertion/rotation. Keep
        # the pre-response anchor through PROVISIONAL: confirmation asks for
        # persistent deformation, not another full threshold per frame.
        # Strain-only callers retain the historical research/test contract.
        if descriptor is not None:
            deformation = raw_deformation
            floor = self.config.minimum_distal_marker_deformation_mm
            # RMS deformation alone has no sign: continued relaxation after a
            # pull reversal can exceed the floor without transmitting the new
            # pull. On the supported ordinary-bending branch, pull shortens
            # the measured interface-tip chord and release lengthens it. Use
            # the fixed physical motor sign, never J or predicted strain.
            # A stopped PROVISIONAL hold retains the acquisition direction.
            distal_confirmed = self._raw_tendon_evidence(deformation, raw_tendon_delta)
            # Evidence magnitude retains the onset scale for diagnostic replay.
            # Only classification uses the lower, already-onset persistence floor.
            self.tendon_distal_response_evidence = float(np.clip(deformation/floor, 0., 1.))
            self.tendon_distal_response_confirmed = distal_confirmed

        # Encoder readiness is per shaft; it is not proof of engagement.
        # The diagnostic fit retains nuisance columns for moving shafts.
        pending_ready = pending & moving & (
            np.abs(delta)
            >= self.config.minimum_transmitted_increment_rad)
        # A marker-corrected distal response is itself the tendon
        # observability event and supersedes the interface-fit motor floor.
        if distal_confirmed:
            pending_ready[self.config.tendon_axis] = True
        if not np.any(pending_ready):
            self.inferred_transmitted_increment[:] = 0.0
            self.response_evidence[:] = 0.0
            self.joint_response_residual = float(
                np.linalg.norm(response/scale))
            if np.count_nonzero(interface_moving) > 1 and "PROVISIONAL" not in self.phase:
                self.previous_response_motor = motor.copy()
                self.previous_pose = pose.copy()
                self.marker_response_anchor = chassis_position.copy()
                self.marker_anchor_history = [chassis_position.copy()]
                self.previous_strain = None if strain is None else strain.copy()
                self.response_window_timestamp_ns = timestamp_ns
                self.measured_response_direction[:] = 0.0
            observe_engaged_gain()
            return self.snapshot()

        eligible = moving & (~pending | pending_ready)

        directions = np.sign(delta).astype(np.int8)
        measured = response/scale
        columns = matrix/scale[:, np.newaxis]
        # The joint fit is a metric diagnostic only. Neither its coefficients,
        # evidence nor predicted direction may decide engagement or widths.
        transmitted, evidence, residual = self._bounded_joint_fit(
            columns, measured, delta, eligible,
            self.config.minimum_transmitted_increment_rad)
        if distal_confirmed:
            evidence[self.config.tendon_axis] = max(
                evidence[self.config.tendon_axis], distal_evidence)
        self.inferred_transmitted_increment = transmitted
        self.response_evidence = evidence
        self.joint_response_residual = residual

        covariance = np.asarray(self.config.marker_increment_covariance_m2).reshape(3, 3)
        marker_noise_factor = np.linalg.cholesky(covariance)
        marker_delta = chassis_position-self.marker_response_anchor
        # A single current-frame spike can otherwise stop take-up indefinitely:
        # PROVISIONAL hands ownership to the zero-command confirmation hold.
        # For measured marker translation require the same resolved onset in
        # the causal three-frame median as well. This affects evidence only,
        # not the UKF observation, motor bookkeeping, or confirmation policy.
        marker_onset = marker_delta
        if observed_interface_position_m is not None:
            # A reversal resets this window. Never let its first post-anchor
            # sample bypass the robust onset test and stop take-up on noise.
            # With two post-anchor samples, the actual anchor is the third
            # median member; both responses must support the displacement.
            # Mature windows use the last three post-anchor samples as before.
            marker_onset = (
                np.median(self.marker_anchor_history[-3:], axis=0)
                -self.marker_response_anchor
                if len(self.marker_anchor_history) >= 3
                else np.zeros(3))
        marker_sample = (pose[:3, 3]-sample_pose_anchor[:3, 3]
                         -knob_interface_translation_m(sample_delta[2]))
        # A Cartesian offset alone cannot distinguish insertion and rotation.
        # Do not use a model to resolve that ambiguity. Clear an ambiguous
        # pre-handoff window once the other motion stops before accumulating
        # evidence for the remaining isolated shaft.
        def measured_support(axis):
            if axis == self.config.tendon_axis or not isolated:
                return False, False
            # Shaft 1 is rotation; all other interface evidence is Cartesian
            # translation. Never mix fitted rotation into insertion evidence.
            rotational = axis == 1
            # Store the measured direction in whitened Cartesian coordinates.
            # Onset and persistence must use the same noise metric: mixing a
            # Mahalanobis onset with Euclidean projection can reject even an
            # identical repeated observation under anisotropic covariance.
            offset = (response[:3] if rotational else
                      np.linalg.solve(marker_noise_factor, marker_delta))
            fresh = (sample_response[:3]*scale[:3] if rotational else
                     np.linalg.solve(marker_noise_factor, marker_sample))
            direction = self.measured_response_direction[axis]
            if np.linalg.norm(direction) <= 1e-12:
                if rotational:
                    resolved = np.linalg.norm(offset) >= self.config.minimum_interface_rotation_rad
                else:
                    # Four increment sigmas in 3D, not a single-axis 3-sigma
                    # test after selecting a favorable direction from noise.
                    resolved = offset@offset > 16.0
                    robust_offset = np.linalg.solve(
                        marker_noise_factor, marker_onset)
                    resolved = resolved and robust_offset@robust_offset > 16.0
                if resolved:
                    self.measured_response_direction[axis] = offset/np.linalg.norm(offset)
                return bool(resolved), False
            floor = (self.config.minimum_interface_rotation_rad if rotational
                     else 3.0)
            support = float(offset@direction)
            confirmed = support > floor
            if (self.phase[axis] == "PROVISIONAL"
                    and abs(sample_delta[axis]) >= self.config.minimum_motor_increment_rad):
                confirmed = confirmed and float(fresh@direction) > floor
            return bool(confirmed), bool(support < -floor)

        for axis in np.flatnonzero(eligible):
            interface_confirmed, contradictory = measured_support(axis)
            confirmed = (distal_confirmed if axis == self.config.tendon_axis
                         else interface_confirmed)
            if (self.phase[axis] == "PROVISIONAL"
                    and not (axis == self.config.tendon_axis and descriptor is not None)
                    and abs(sample_delta[axis])
                    >= self.config.minimum_motor_increment_rad):
                confirmed = confirmed and (
                    interface_confirmed
                    or (axis == self.config.tendon_axis
                        and self.config.distal_confirmation_enabled
                        and abs(sample_distal)
                        >= self.config.minimum_distal_bending_increment))
            contradictory = contradictory and not confirmed
            classification = (
                "CONFIRMED" if confirmed else
                "CONTRADICTORY" if contradictory else "INCONCLUSIVE")
            self.response_classification[axis] = classification
            if confirmed:
                self.provisional_rejection_count[axis] = 0
                if self.confirmation_direction[axis] == directions[axis]:
                    self.confirmation_count[axis] += 1
                else:
                    self.confirmation_direction[axis] = directions[axis]
                    self.confirmation_count[axis] = 1
                confirmation_complete = (
                    self.confirmation_count[axis]
                    >= self.config.engagement_confirmation_observations)
                if (self.phase[axis] in ("TAKEUP", "PROVISIONAL")
                        and not np.isfinite(self.engagement_anchor_motor[axis])):
                    self.engagement_anchor_motor[axis] = motor[axis]
                # Actuation handoff and statistical confirmation are separate
                # decisions. The first noise-resolved measured response proves
                # that full-rate geometric take-up must stop immediately, but
                # repeated samples are still required before width learning
                # and persistent ENGAGED state are committed.
                if (self.phase[axis] == "TAKEUP"
                        and not confirmation_complete):
                    self.engaged_direction[axis] = directions[axis]
                    self.phase[axis] = "PROVISIONAL"
                    self.engagement_anchor_motor[axis] = motor[axis]
                    if timestamp_ns is not None:
                        self.last_evidence_timestamp_ns[axis] = int(
                            timestamp_ns)
                if (confirmation_complete
                        and not self.external_motion
                        and self.phase[axis] in ("TAKEUP", "PROVISIONAL")):
                    # Cartesian evidence identifies an observed onset, not a
                    # transmitted motor increment. Do not subtract a Jacobian
                    # fit to learn a gap width from this observation.
                    self._learn_confirmed_boundary(axis)
                if confirmation_complete:
                    self.remaining[axis] = 0.0
                    self.remaining_lower[axis] = 0.0
                    self.remaining_upper[axis] = 0.0
                    self.engaged_direction[axis] = directions[axis]
                    if timestamp_ns is not None:
                        self.last_evidence_timestamp_ns[axis] = int(
                            timestamp_ns)
                    self.phase[axis] = "ENGAGED"
                    self.confidence[axis] = min(
                        1.0, self.confidence[axis]+0.2)
            elif self.phase[axis] == "PROVISIONAL":
                # Loss below the persistence floor clears support. A later
                # observation must meet the full onset floor before hysteresis
                # can be used again. Preserve the anchor and zero ownership.
                self.confirmation_count[axis] = (
                    0 if axis == self.config.tendon_axis and descriptor is not None else 1)
                if contradictory and not self.external_motion:
                    self.provisional_rejection_count[axis] += 1
                    if (self.provisional_rejection_count[axis]
                            >= self.config.provisional_rejection_observations):
                        self.phase[axis] = "FAILED"
                        self.confidence[axis] = 0.0
                    else:
                        self.confidence[axis] = max(
                            0.0, self.confidence[axis]-0.02)
                # INCONCLUSIVE deliberately preserves PROVISIONAL and the
                # first-response engagement
                # boundary. It must not re-enable a geometric take-up burst.
            elif self.phase[axis] == "TAKEUP":
                self.confirmation_count[axis] = 0
                self.confirmation_direction[axis] = 0
                self.confidence[axis] = max(
                    0.0, self.confidence[axis]-0.02)
                # This fail-closed travel bound applies only before the first
                # credible response. A provisional response changes the
                # epistemic problem from "gap not exhausted" to "awaiting
                # confirmation" and is handled above without more take-up.
                limit = self.config.unconfirmed_travel_limit_rad(
                    self._width_for_direction(
                        axis, directions[axis], prior=True)[0][axis], axis=axis)
                if (not self.external_motion and limit > 0.0
                        and self.accumulated_since_reversal[axis] > limit):
                    self.phase[axis] = "FAILED"
        # A ready raw motor is not a resolved visual response. Preserve the
        # common pose/motor/strain window while an isolated shaft awaits
        # confirmation. Rebase ambiguous windows rather than attributing them
        # through a Jacobian prediction.
        if (not any(phase in ("TAKEUP", "PROVISIONAL") for phase in self.phase)
                or (np.count_nonzero(interface_moving) > 1
                    and "PROVISIONAL" not in self.phase)):
            self.previous_response_motor = motor.copy()
            self.previous_pose = pose.copy()
            self.marker_response_anchor = chassis_position.copy()
            self.marker_anchor_history = [chassis_position.copy()]
            self.previous_strain = None if strain is None else strain.copy()
            self.response_window_timestamp_ns = timestamp_ns
            self.measured_response_direction[:] = 0.0
        observe_engaged_gain()
        return self.snapshot()

    def observe_at_timestamp(self, history, timestamp_ns, interface_pose, model, *,
                             control_owned, observed_interface_position_m, **response_options):
        """Correct the existing observation owner at source time and replay.

        History entries are [source_ns, raw shaft radians, checkpoint, owned].
        ROS composition and offline qualification share this one implementation.
        """
        if not history:
            return self.snapshot(), history
        aligned = BacklashStateEstimator._source_motor(history, timestamp_ns, control_owned)
        if aligned is None:
            # A delayed observation from externally owned homing must not
            # create a transaction failure or confirm engagement after arming.
            return self.snapshot(), history
        entries, index, response_motor = aligned
        self.restore_state(entries[index][2])
        self.observe_response(
            response_motor, interface_pose, model.jacobian,
            model.state_scale, timestamp_ns=int(timestamp_ns),
            observed_interface_position_m=observed_interface_position_m,
            **response_options)
        entries[index][2] = self.clone_state()
        for replay_index in range(index+1, len(entries)):
            self.advance_motor(
                entries[replay_index][1],
                external_motion=not entries[replay_index][3],
                track_external_response=entries[replay_index][2].get("track_external_response", False))
            entries[replay_index][2] = (
                self.clone_state())
        return self.snapshot(), entries

    def observe(self, motor_angle_rad, interface_pose, jacobian,
                state_scale):
        """Compatibility wrapper for one combined motor/visual sample."""
        self.advance_motor(motor_angle_rad)
        return self.observe_response(
            motor_angle_rad, interface_pose, jacobian, state_scale)


class BacklashFeedforwardCompensator:
    """Convert desired post-engagement logical velocity into take-up motion."""

    def __init__(self, config: BacklashConfig):
        self.config = config
        self.takeup_velocity = _vector(
            "takeup_velocity", config.takeup_velocity, nonnegative=True)

    def motor_radians_per_second(self, contract):
        """Return physical take-up rates compatible with manager speed floors.

        Subfloor chassis/knob requests can both round to the same logical
        velocity, cancelling chassis motion after coupling. Apply the shared
        contract bounds before RPM quantization; final coupled projection
        remains authoritative for limits, direction and unintended leakage.
        """
        motor = np.zeros(6, dtype=np.float64)
        motor[:CONTROL_AXES] = np.where(
            self.takeup_velocity > 0.,
            np.clip(self.takeup_velocity,
                    contract.velocity_min[:CONTROL_AXES],
                    contract.velocity_max[:CONTROL_AXES]),
            0.)
        _, rates, _ = contract.quantize_motor_velocity(motor)
        return np.abs(rates[:CONTROL_AXES])

    def command(self, desired_logical_velocity, joint_position, contract,
                state: BacklashSnapshot):
        desired = _vector("desired_logical_velocity", desired_logical_velocity,
                          size=6).copy()
        projected = contract.project_velocity(desired, joint_position)
        # Compensate physical shafts, not logical coordinates. Logical
        # insertion and bend are coupled before reaching shaft 0.
        rates = projected.motor_radians_per_second.copy()
        motor_sign = np.sign(rates[:CONTROL_AXES]).astype(np.int8)
        takeup_rates = self.motor_radians_per_second(contract)
        changed = False
        for axis in range(CONTROL_AXES):
            if motor_sign[axis] == 0:
                continue
            needs_takeup = (
                state.remaining_rad[axis] > 0.0
                or ((state.width_positive_rad[axis]
                     if motor_sign[axis] > 0
                     else state.width_negative_rad[axis]) > 0.0
                    and state.motion_direction[axis] != motor_sign[axis]))
            if needs_takeup:
                rates[axis] = motor_sign[axis]*max(
                    abs(rates[axis]), takeup_rates[axis])
                changed = True
        if not changed:
            return projected.logical_velocity.copy()
        motor = contract.motor_radians_per_second_to_motor_axis_velocity(rates)
        compensated = contract.motor_axis_to_logical_velocity(motor)
        return contract.project_logical_velocity(compensated, joint_position)

    def hold_takeup_direction(self, desired_logical_velocity,
                              previous_logical_velocity, joint_position,
                              contract, state: BacklashSnapshot):
        """Prevent optimizer cancellation while a measured gap is active.

        MPPI remains free to choose the post-engagement magnitude.  For every
        shaft in TAKEUP, however, a zero or opposite first command is replaced
        by the most recent same-direction request.  If that is unavailable,
        the bounded calibrated take-up velocity is used.  Conversion happens
        in motor coordinates so catheter insertion/bend coupling is preserved;
        the returned logical command is re-projected against current limits.
        """
        desired = _vector(
            "desired_logical_velocity", desired_logical_velocity,
            size=6).copy()
        previous = _vector(
            "previous_logical_velocity", previous_logical_velocity,
            size=6)
        requested = contract.project_velocity(
            desired, joint_position).motor_radians_per_second.copy()
        prior = contract.project_velocity(
            previous, joint_position).motor_radians_per_second
        takeup_rates = self.motor_radians_per_second(contract)
        changed = False
        for axis in range(CONTROL_AXES):
            if state.phase[axis] != "TAKEUP":
                continue
            direction = int(state.motion_direction[axis])
            if direction == 0:
                continue
            if np.sign(requested[axis]) == direction:
                continue
            if np.sign(prior[axis]) == direction and abs(prior[axis]) > 0.0:
                requested[axis] = prior[axis]
            else:
                requested[axis] = direction*takeup_rates[axis]
            changed = True
        if not changed:
            return contract.project_logical_velocity(desired, joint_position)
        motor = contract.motor_radians_per_second_to_motor_axis_velocity(
            requested)
        logical = contract.motor_axis_to_logical_velocity(motor)
        return contract.project_logical_velocity(logical, joint_position)

    def coordinate_takeup(self, desired_logical_velocity, joint_position,
                          contract, state: BacklashSnapshot):
        """Hold productive shafts while any shaft finishes a take-up macro.

        MPPI plans a coupled post-engagement velocity. Executing productive
        components as soon as their individual gaps close realizes a different
        vector while slower shafts are still blocked. During a macro, pass only
        shafts still in ``TAKEUP``; already engaged shafts wait at zero. The
        complete coupled command is released after every pending shaft has
        its first strongly attributed response. Repeated observations commit
        persistent engagement independently of this actuation handoff.
        """
        desired = _vector(
            "desired_logical_velocity", desired_logical_velocity,
            size=6)
        requested = contract.project_velocity(
            desired, joint_position).motor_radians_per_second.copy()
        motor_sign = np.sign(
            requested[:CONTROL_AXES]).astype(np.int8)
        taking_up = np.asarray(state.taking_up, dtype=bool)
        # Do not wait for the next encoder callback to label a newly requested
        # direction TAKEUP.  Predict the same directional-gap transition used
        # by ``command()`` from the command about to be sent.  Otherwise a new
        # shaft or reversal can leak one complete MPPI interval while already
        # engaged shafts transmit a partial coupled action.
        selected_width = np.where(
            motor_sign > 0, state.width_positive_rad,
            np.where(motor_sign < 0, state.width_negative_rad, 0.0))
        pending = taking_up | (
            (motor_sign != 0)
            & (selected_width > 0.0)
            & (state.motion_direction != motor_sign))
        if not np.any(pending):
            return contract.project_logical_velocity(desired, joint_position)
        requested[:CONTROL_AXES][~pending] = 0.0
        motor = contract.motor_radians_per_second_to_motor_axis_velocity(
            requested)
        logical = contract.motor_axis_to_logical_velocity(motor)
        return contract.project_logical_velocity(logical, joint_position)


@dataclass(frozen=True)
class TakeupDecision:
    """One output of the post-MPPI transmission transaction arbiter."""

    command_logical_velocity: np.ndarray
    state: str
    active_mask: np.ndarray
    pending_mask: np.ndarray
    direction: np.ndarray
    execute_plan: bool = False
    replan_required: bool = False
    reason: str = "ready_to_plan"
    saturated_mask: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=bool))
    leakage_mask: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=bool))
    requested_motor_radians_per_second: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))
    realized_motor_radians_per_second: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES))


class TakeupTransactionArbiter:
    """Serialize backlash take-up and post-engagement MPPI execution.

    MPPI commands describe desired *post-take-up* motion.  A command that
    requires one or more shaft direction changes starts a transaction.  While
    that transaction is active, only still-unconfirmed shafts receive bounded
    take-up rates; shafts already ready for the requested direction are held.
    Once every participating shaft is observation-confirmed, the old MPPI
    command is discarded and a fresh plan is required.
    """

    READY_TO_PLAN = "READY_TO_PLAN"
    TAKEUP_ACTIVE = "TAKEUP_ACTIVE"
    CONFIRMATION_HOLD = "CONFIRMATION_HOLD"
    RESPONSE_HOLD = "RESPONSE_HOLD"
    REPLAN_REQUIRED = "REPLAN_REQUIRED"
    SATURATED_REPLAN = "SATURATED_REPLAN"
    FAILED = "FAILED"

    def __init__(self, compensator: BacklashFeedforwardCompensator,
                 response_free_mask=(True, True, True),
                 confirmation_hold_timeout_s=1.0, clock=None):
        self.compensator = compensator
        self.response_free_mask = np.asarray(
            response_free_mask, dtype=bool)
        if self.response_free_mask.shape != (CONTROL_AXES,):
            raise ValueError("response_free_mask must contain three booleans")
        self.confirmation_hold_timeout_s = float(
            confirmation_hold_timeout_s)
        if (not np.isfinite(self.confirmation_hold_timeout_s)
                or self.confirmation_hold_timeout_s <= 0.0):
            raise ValueError(
                "confirmation_hold_timeout_s must be finite and positive")
        self._clock = monotonic if clock is None else clock
        self.generation = 0
        self.reset()

    def reset(self):
        self.state = self.READY_TO_PLAN
        self.active_mask = np.zeros(CONTROL_AXES, dtype=bool)
        self.pending_mask = np.zeros(CONTROL_AXES, dtype=bool)
        self.direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.saturated_mask = np.zeros(CONTROL_AXES, dtype=bool)
        self.leakage_mask = np.zeros(CONTROL_AXES, dtype=bool)
        self.requested_motor_radians_per_second = np.zeros(CONTROL_AXES)
        self.realized_motor_radians_per_second = np.zeros(CONTROL_AXES)
        self.confirmation_hold_started_s = None
        self.response_hold_started_s = None
        self.response_onset_seen = False
        self.response_stop_source_ns = None
        self.response_evidence_source_ns = 0
        self.response_post_stop_observations = 0
        self.response_burst_started_s = None
        self.response_last_published_moving = False
        self.response_onset_source_ns = 0
        self.response_transaction_source_floor_ns = 0

    def note_tendon_onset(self, snapshot):
        """Latch a stop request only, without changing belief or readiness."""
        if (self.state not in (self.TAKEUP_ACTIVE, self.RESPONSE_HOLD)
                or not self.pending_mask[2] or not snapshot.tendon_response_onset
                or snapshot.phase[2] not in ("TAKEUP", "PROVISIONAL")
                or snapshot.motion_direction[2] != self.direction[2]
                or (snapshot.tendon_response_timestamp_ns
                    and snapshot.tendon_response_timestamp_ns <= self.response_transaction_source_floor_ns)
                or self.response_onset_seen):
            return False
        self.response_onset_seen = True
        self.response_onset_source_ns = snapshot.tendon_response_timestamp_ns
        self.response_hold_started_s = self._clock()
        self.response_stop_source_ns = None
        self.response_post_stop_observations = 0
        return True

    def note_command_publication(self, timestamp_ns, tendon_moving):
        """Source-clock zero barrier for subsequent camera evidence.

        Publication is not a firmware acknowledgement. Three distinct images
        after this barrier replace timer-only permission to add travel.
        """
        if (self.response_onset_seen and not tendon_moving
                and (self.response_last_published_moving
                     or self.response_stop_source_ns is None)):
            self.response_stop_source_ns = int(timestamp_ns)
            self.response_post_stop_observations = 0
            self.response_hold_started_s = self._clock()
        self.response_last_published_moving = bool(tendon_moving)

    @staticmethod
    def _zero():
        return np.zeros(6, dtype=np.float64)

    def _decision(self, command, *, execute=False, replan=False, reason=None):
        return TakeupDecision(
            command_logical_velocity=np.asarray(
                command, dtype=np.float64).copy(),
            state=self.state,
            active_mask=self.active_mask.copy(),
            pending_mask=self.pending_mask.copy(),
            direction=self.direction.copy(),
            execute_plan=bool(execute),
            replan_required=bool(replan),
            reason=self.state.lower() if reason is None else str(reason),
            saturated_mask=self.saturated_mask.copy(),
            leakage_mask=self.leakage_mask.copy(),
            requested_motor_radians_per_second=(
                self.requested_motor_radians_per_second.copy()),
            realized_motor_radians_per_second=(
                self.realized_motor_radians_per_second.copy()))

    def _readiness(self, snapshot: BacklashSnapshot):
        phase = np.asarray(snapshot.phase)
        failed = self.active_mask & (phase == "FAILED")
        ready = (self.active_mask
                 & (phase == "ENGAGED")
                 & (np.asarray(snapshot.engaged_direction, dtype=np.int8)
                    == self.direction))
        # A configured zero-width direction needs no physical transaction.
        widths = np.where(
            self.direction > 0, snapshot.width_positive_rad,
            np.where(self.direction < 0, snapshot.width_negative_rad, 0.0))
        ready |= self.active_mask & (widths <= 0.0)
        return ready, failed

    def _project_takeup(self, joint_position, contract, pending_mask=None,
                        direction=None):
        """Project one physical take-up request through the final contract.

        The transaction is defined in physical shaft coordinates, but the ROS
        manager accepts coupled logical coordinates.  Inspecting the command
        again after logical position/rate projection is essential: clipping a
        coupled logical component can silently stop a pending shaft or move a
        shaft that the transaction intended to hold.
        """
        pending = (self.pending_mask if pending_mask is None else
                   np.asarray(pending_mask, dtype=bool))
        requested_direction = (self.direction if direction is None else
                               np.asarray(direction, dtype=np.int8))
        rates = np.zeros(6, dtype=np.float64)
        takeup = self.compensator.motor_radians_per_second(contract)
        rates[:CONTROL_AXES][pending] = (
            requested_direction[pending]*takeup[pending])
        motor_axis = contract.motor_radians_per_second_to_motor_axis_velocity(
            rates)
        logical = contract.motor_axis_to_logical_velocity(motor_axis)
        projection = contract.project_velocity(logical, joint_position)
        realized = projection.motor_radians_per_second[:CONTROL_AXES]
        realized_direction = np.sign(realized).astype(np.int8)
        blocked = pending & (realized_direction != requested_direction)
        leakage = (~pending) & (realized_direction != 0)
        return projection.logical_velocity, rates[:CONTROL_AXES], realized, (
            blocked), leakage

    def physical_direction_is_feasible(self, axis, direction, joint_position,
                                       contract):
        """Return whether one shaft direction survives final projection."""
        index = int(axis)
        sign = int(np.sign(direction))
        if index < 0 or index >= CONTROL_AXES or sign == 0:
            return False
        pending = np.zeros(CONTROL_AXES, dtype=bool)
        pending[index] = True
        directions = np.zeros(CONTROL_AXES, dtype=np.int8)
        directions[index] = sign
        _, _, _, blocked, leakage = self._project_takeup(
            joint_position, contract, pending, directions)
        return not bool(np.any(blocked) or np.any(leakage))

    def physical_direction_vector_is_feasible(
            self, direction, joint_position, contract):
        """Return whether one complete shaft-direction mode is feasible."""
        directions = np.asarray(direction, dtype=np.int8)
        if (directions.shape != (CONTROL_AXES,)
                or np.any(np.abs(directions) > 1)):
            raise ValueError("direction must contain three signs")
        pending = directions != 0
        if not np.any(pending):
            return True
        _, _, _, blocked, leakage = self._project_takeup(
            joint_position, contract, pending, directions)
        return not bool(np.any(blocked) or np.any(leakage))

    def _takeup_decision(self, joint_position, contract):
        self.confirmation_hold_started_s = None
        command, requested, realized, blocked, leakage = (
            self._project_takeup(joint_position, contract))
        # Trace both the configured physical take-up request and the actual
        # integer-RPM result on every transaction tick, not only saturation.
        self.requested_motor_radians_per_second = requested.copy()
        self.realized_motor_radians_per_second = realized.copy()
        self.saturated_mask = blocked.copy()
        self.leakage_mask = leakage.copy()
        if np.any(blocked) or np.any(leakage):
            # Leakage means the requested pending-shaft set is not realizable
            # as one atomic physical action. Block every pending direction in
            # that case so a fresh planner rollout cannot immediately recreate
            # the same coupled transaction at this encoder position.
            self.saturated_mask = blocked.copy()
            if np.any(leakage):
                self.saturated_mask |= self.pending_mask
            self.state = self.SATURATED_REPLAN
            return self._decision(
                self._zero(), replan=True,
                reason="takeup_saturated_replan")
        return self._decision(command, reason="takeup_active")

    def begin(self, desired_logical_velocity, joint_position, contract,
              snapshot: BacklashSnapshot, *, physical_direction=None):
        """Accept a fresh MPPI result or start a bounded take-up transaction."""
        if self.state != self.READY_TO_PLAN:
            raise RuntimeError(
                f"cannot begin a take-up transaction in state {self.state}")
        desired = _vector(
            "desired_logical_velocity", desired_logical_velocity, size=6)
        projected = contract.project_velocity(desired, joint_position)
        motor = projected.motor_radians_per_second[:CONTROL_AXES]
        self.direction = np.sign(motor).astype(np.int8)
        if physical_direction is not None:
            direction = _vector('endpoint physical direction', physical_direction,
                                size=CONTROL_AXES)
            if not np.all(np.isin(direction, [-1, 0, 1])):
                raise ValueError('endpoint physical direction must contain signs')
            self.direction = direction.astype(np.int8)
        # Every commanded shaft must be observation-confirmed before the
        # post-engagement MPPI action can execute.
        self.active_mask = self.direction != 0
        self.pending_mask.fill(False)
        if not np.any(self.active_mask):
            return self._decision(
                projected.logical_velocity, execute=True,
                reason="post_takeup_command_ready")

        ready, failed = self._readiness(snapshot)
        if np.any(failed):
            self.state = self.FAILED
            return self._decision(
                self._zero(), reason="takeup_estimator_failed")
        self.pending_mask = self.active_mask & ~ready
        if not np.any(self.pending_mask):
            return self._decision(
                projected.logical_velocity, execute=True,
                reason="post_takeup_command_ready")

        self.generation += 1
        self.response_hold_started_s = None
        self.response_onset_seen = False
        self.response_stop_source_ns = None
        self.response_evidence_source_ns = 0
        self.response_post_stop_observations = 0
        self.response_burst_started_s = None
        self.response_onset_source_ns = 0
        self.response_transaction_source_floor_ns = snapshot.tendon_response_timestamp_ns
        self.state = self.TAKEUP_ACTIVE
        return self._takeup_decision(joint_position, contract)

    def advance(self, joint_position, contract, snapshot: BacklashSnapshot):
        """Refresh an active transaction from the newest measured state."""
        if self.state in (self.REPLAN_REQUIRED, self.SATURATED_REPLAN):
            return self._decision(
                self._zero(), replan=True,
                reason=("takeup_saturated_replan"
                        if self.state == self.SATURATED_REPLAN else
                        "takeup_complete_replan"))
        if self.state == self.FAILED:
            return self._decision(
                self._zero(), reason="takeup_estimator_failed")
        if self.state not in (self.TAKEUP_ACTIVE, self.CONFIRMATION_HOLD,
                              self.RESPONSE_HOLD):
            return self._decision(self._zero())

        ready, failed = self._readiness(snapshot)
        if np.any(failed):
            self.state = self.FAILED
            self.pending_mask = self.active_mask & ~ready
            return self._decision(
                self._zero(), reason="takeup_estimator_failed")
        self.pending_mask = self.active_mask & ~ready
        if not np.any(self.pending_mask):
            self.confirmation_hold_started_s = None
            self.state = self.REPLAN_REQUIRED
            return self._decision(
                self._zero(), replan=True, reason="takeup_complete_replan")
        phase = np.asarray(snapshot.phase)
        provisional = (self.pending_mask & (phase == "PROVISIONAL")
                       & (np.asarray(snapshot.engaged_direction,
                                     dtype=np.int8) == self.direction))
        if np.any(provisional):
            if self.confirmation_hold_started_s is None:
                self.confirmation_hold_started_s = self._clock()
            elif (self._clock()-self.confirmation_hold_started_s
                  >= self.confirmation_hold_timeout_s):
                self.state = self.FAILED
                return self._decision(
                    self._zero(), reason="takeup_confirmation_timeout")
            self.state = self.CONFIRMATION_HOLD
            return self._decision(
                self._zero(), reason="takeup_confirmation_hold")
        # Early response does not make a shaft ready. Retain ownership and
        # allow observations to settle before adding more physical travel.
        # Thereafter use minimum-speed-compliant bursts: a continuous scaled
        # rate would be raised back to the hardware floor by projection.
        self.note_tendon_onset(snapshot)
        if self.response_onset_seen and self.pending_mask[2]:
            now = self._clock()
            if self.response_burst_started_s is not None:
                if now-self.response_burst_started_s < 1./15.:
                    self.state = self.TAKEUP_ACTIVE
                    return self._takeup_decision(joint_position, contract)
                self.response_burst_started_s = None
                self.response_stop_source_ns = None
                self.response_post_stop_observations = 0
                self.response_hold_started_s = now
            else:
                stamp = snapshot.tendon_response_timestamp_ns
                if (self.response_stop_source_ns is not None
                        and stamp > self.response_stop_source_ns
                        and stamp > self.response_evidence_source_ns
                        and snapshot.motion_direction[2] == self.direction[2]):
                    self.response_evidence_source_ns = stamp
                    if snapshot.tendon_response_encoder_quiet:
                        self.response_post_stop_observations += 1
                    else:
                        self.response_post_stop_observations = 0
                if (self.response_post_stop_observations >= 3
                        and not snapshot.tendon_distal_response_confirmed):
                    self.response_burst_started_s = now
                    self.response_post_stop_observations = 0
                    self.state = self.TAKEUP_ACTIVE
                    return self._takeup_decision(joint_position, contract)
            if now-self.response_hold_started_s >= self.confirmation_hold_timeout_s:
                self.state = self.FAILED
                return self._decision(self._zero(), reason="takeup_response_evidence_timeout")
            self.state = self.RESPONSE_HOLD
            self.requested_motor_radians_per_second.fill(0.)
            self.realized_motor_radians_per_second.fill(0.)
            return self._decision(self._zero(), reason="takeup_response_hold")
        self.state = self.TAKEUP_ACTIVE
        return self._takeup_decision(joint_position, contract)

    def release_for_replan(self):
        """Acknowledge the zero barrier and permit a new MPPI rollout."""
        if self.state not in (self.REPLAN_REQUIRED, self.SATURATED_REPLAN):
            return False
        self.state = self.READY_TO_PLAN
        self.active_mask.fill(False)
        self.pending_mask.fill(False)
        self.direction.fill(0)
        self.confirmation_hold_started_s = None
        return True
