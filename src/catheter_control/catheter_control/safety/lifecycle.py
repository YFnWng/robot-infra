"""Pure lifecycle gating for the minimal closed-loop ROS integration."""
from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math


class ControllerState(str, Enum):
    DISARMED = "DISARMED"
    WAITING_FOR_MANAGER = "WAITING_FOR_MANAGER"
    INITIALIZING_ESTIMATOR = "INITIALIZING_ESTIMATOR"
    READY = "READY"
    ACTIVE = "ACTIVE"
    DEGRADED = "DEGRADED"
    FAULTED = "FAULTED"


@dataclass(frozen=True)
class FreshnessLimits:
    manager_s: float = 0.5
    feedback_s: float = 0.15
    marker_s: float = 0.15
    marker_diagnostic_s: float = 0.5
    feedback_pair_skew_s: float = 0.15

    def __post_init__(self) -> None:
        values = (
            self.manager_s, self.feedback_s, self.marker_s,
            self.marker_diagnostic_s, self.feedback_pair_skew_s)
        if any(value <= 0.0 for value in values):
            raise ValueError("freshness limits must be positive")


@dataclass(frozen=True)
class GateInputs:
    manager_ready: bool = False
    manager_age_s: float | None = None
    position_age_s: float | None = None
    encoder_age_s: float | None = None
    encoder_receive_age_s: float | None = None
    position_encoder_skew_s: float | None = None
    marker_age_s: float | None = None
    marker_diagnostic_age_s: float | None = None
    marker_diagnostic_error: bool = False
    accepted_observations: int = 0
    required_observations: int = 5
    consecutive_rejections: int = 0
    maximum_rejections: int = 3
    target_available: bool = False
    collection_present: bool = False
    position_valid: bool = False
    encoder_valid: bool = False
    model_valid: bool = False
    estimator_health: str = "UNINITIALIZED"
    last_marker_update_rejected: bool = False
    last_marker_update_reason: str = "none"


def recoverable_encoder_processing_lag(
        inputs: GateInputs, limits: FreshnessLimits,
        catchup_elapsed_s: float | None,
        catchup_timeout_s: float) -> bool:
    """Return whether stale processed ENC may enter a bounded zero pause.

    This exception is deliberately narrower than :func:`readiness`: only the
    learned-runtime commit may be stale, the device must still be delivering
    advancing valid encoder samples inside the normal feedback deadline, and
    the continuous pause must remain inside its independent timeout. Callers
    must command zero while this predicate is true.
    """
    if catchup_timeout_s <= 0.0:
        raise ValueError("catch-up timeout must be positive")
    raw_ready = False
    if inputs.encoder_receive_age_s is not None:
        raw_ready = readiness(
            replace(inputs, encoder_age_s=inputs.encoder_receive_age_s),
            limits) == (ControllerState.READY, "ready")
    return bool(
        raw_ready
        and inputs.encoder_age_s is not None
        and inputs.encoder_age_s > limits.feedback_s
        and catchup_elapsed_s is not None
        and 0.0 <= catchup_elapsed_s <= catchup_timeout_s)


def recoverable_runtime_interruption(inputs: GateInputs, limits: FreshnessLimits,
                                    reason: str, elapsed_s: float,
                                    timeout_s: float) -> bool:
    """Authorize zero output only; never substitute these inputs for motion.

    Share the existing catch-up budget across visual gaps and compute lag.
    Initialization, raw transport, invalid data and rejection streaks are not
    recoverable here. Sanitizing just the transient fields exposes any second,
    hard failure even when readiness reports the transient failure first.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("recovery timeout must be finite and positive")
    if reason not in {
            "encoder_stale", "accepted_marker_stale",
            "marker_diagnostic_degraded", "marker_diagnostic_error",
            "planner_snapshot_stale_at_commit", "deadline_missed", "ready"}:
        return False
    if not math.isfinite(elapsed_s) or not 0 <= elapsed_s <= timeout_s:
        return False
    raw_age = inputs.encoder_receive_age_s
    if raw_age is None or not math.isfinite(raw_age) or not 0 <= raw_age <= limits.feedback_s:
        return False
    if any(value is None or not math.isfinite(value) or value < 0. for value in (
            inputs.manager_age_s, inputs.position_age_s,
            inputs.position_encoder_skew_s)):
        return False
    sanitized = replace(inputs, encoder_age_s=raw_age, marker_age_s=0.,
                        marker_diagnostic_age_s=0., marker_diagnostic_error=False)
    return readiness(sanitized, limits) == (ControllerState.READY, "ready")


def controller_motion_held(reason: str) -> bool:
    """Shared task-side hold contract; does not grant controller readiness."""
    return reason.startswith("runtime_pause_zero:") or reason in {
        "estimator_catchup_zero", "planner_deadline_miss_zero",
        "planner_reference_changed_zero", "planner_state_changed_zero",
        "model_posterior_not_ready", "model_posterior_changed_during_plan_zero",
        "takeup_active", "takeup_confirmation_hold", "takeup_complete_replan",
        "takeup_saturated_replan", "replanning_after_takeup",
        "replanning_after_takeup_response", "replanning_after_takeup_saturation"}


def paired_source_skew_s(position_timestamp_ns: int | None,
                         encoder_timestamp_ns: int | None
                         ) -> float | None:
    """Return POS/ENC source-stamp skew without mixing in processing delay.

    Freshness is deliberately handled separately from monotonic callback and
    estimator-commit times.  Comparing those arrival/commit times here would
    misclassify estimator scheduling latency as transport pair skew.
    """
    if (position_timestamp_ns is None or encoder_timestamp_ns is None
            or position_timestamp_ns <= 0 or encoder_timestamp_ns <= 0):
        return None
    return abs(position_timestamp_ns-encoder_timestamp_ns)/1e9


def readiness(inputs: GateInputs, limits: FreshnessLimits
              ) -> tuple[ControllerState, str]:
    """Return the pre-activation state and its first fail-closed reason."""
    if inputs.collection_present:
        return ControllerState.DEGRADED, "collection_node_present"
    if (not inputs.manager_ready or inputs.manager_age_s is None
            or inputs.manager_age_s > limits.manager_s):
        return ControllerState.WAITING_FOR_MANAGER, "manager_not_ready"
    feedback = (
        ("position_stale", inputs.position_age_s, limits.feedback_s),
        ("encoder_stale", inputs.encoder_age_s, limits.feedback_s),
    )
    for reason, age, maximum in feedback:
        if age is None or age > maximum:
            return ControllerState.WAITING_FOR_MANAGER, reason
    if not inputs.position_valid:
        return ControllerState.DEGRADED, "position_feedback_out_of_range"
    if not inputs.encoder_valid:
        return ControllerState.DEGRADED, "encoder_feedback_out_of_range"
    if (inputs.position_encoder_skew_s is None
            or inputs.position_encoder_skew_s
            > limits.feedback_pair_skew_s):
        return ControllerState.WAITING_FOR_MANAGER, "feedback_pair_skew"
    if not inputs.model_valid:
        return ControllerState.DEGRADED, "model_invalid"
    if inputs.accepted_observations < inputs.required_observations:
        return (ControllerState.INITIALIZING_ESTIMATOR,
                "estimator_initializing")
    if inputs.marker_age_s is None or inputs.marker_age_s > limits.marker_s:
        return ControllerState.DEGRADED, "accepted_marker_stale"
    if inputs.consecutive_rejections >= inputs.maximum_rejections:
        return (ControllerState.DEGRADED,
                "repeated_marker_rejection:"
                + str(inputs.last_marker_update_reason or "unknown"))
    transient_marker_rejection = (
        inputs.estimator_health == "DEGRADED"
        and inputs.last_marker_update_rejected
        and 0 < inputs.consecutive_rejections < inputs.maximum_rejections)
    if (inputs.estimator_health != "TRACKING"
            and not transient_marker_rejection):
        return ControllerState.DEGRADED, "estimator_degraded"
    if (inputs.marker_diagnostic_age_s is None
            or inputs.marker_diagnostic_age_s
            > limits.marker_diagnostic_s
            or inputs.marker_diagnostic_error):
        return ControllerState.DEGRADED, "marker_diagnostic_degraded"
    if not inputs.target_available:
        return ControllerState.READY, "target_missing"
    return ControllerState.READY, "ready"
