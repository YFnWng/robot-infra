from dataclasses import replace

import pytest

from catheter_control.safety.lifecycle import (
    ControllerState, FreshnessLimits, GateInputs, paired_source_skew_s,
    readiness, recoverable_encoder_processing_lag)


@pytest.fixture
def ready_inputs():
    return GateInputs(
        manager_ready=True,
        manager_age_s=0.01,
        position_age_s=0.01,
        encoder_age_s=0.01,
        encoder_receive_age_s=0.01,
        position_encoder_skew_s=0.005,
        marker_age_s=0.01,
        marker_diagnostic_age_s=0.01,
        marker_diagnostic_error=False,
        accepted_observations=5,
        required_observations=5,
        consecutive_rejections=0,
        maximum_rejections=3,
        target_available=True,
        collection_present=False,
        position_valid=True,
        encoder_valid=True,
        model_valid=True,
        estimator_health="TRACKING")


def test_all_gates_pass_only_when_every_active_input_is_ready(ready_inputs):
    assert readiness(ready_inputs, FreshnessLimits()) == (
        ControllerState.READY, "ready")


@pytest.mark.parametrize(
    "change,expected",
    [
        ({"manager_ready": False},
         (ControllerState.WAITING_FOR_MANAGER, "manager_not_ready")),
        ({"position_age_s": 1.0},
         (ControllerState.WAITING_FOR_MANAGER, "position_stale")),
        ({"encoder_age_s": 1.0},
         (ControllerState.WAITING_FOR_MANAGER, "encoder_stale")),
        ({"position_encoder_skew_s": 1.0},
         (ControllerState.WAITING_FOR_MANAGER, "feedback_pair_skew")),
        ({"position_valid": False},
         (ControllerState.DEGRADED, "position_feedback_out_of_range")),
        ({"encoder_valid": False},
         (ControllerState.DEGRADED, "encoder_feedback_out_of_range")),
        ({"model_valid": False},
         (ControllerState.DEGRADED, "model_invalid")),
        ({"accepted_observations": 4},
         (ControllerState.INITIALIZING_ESTIMATOR,
          "estimator_initializing")),
        ({"estimator_health": "DEGRADED"},
         (ControllerState.DEGRADED, "estimator_degraded")),
        ({"marker_age_s": 1.0},
         (ControllerState.DEGRADED, "accepted_marker_stale")),
        ({"consecutive_rejections": 3},
         (ControllerState.DEGRADED, "repeated_marker_rejection:none")),
        ({"marker_diagnostic_error": True},
         (ControllerState.DEGRADED, "marker_diagnostic_degraded")),
        ({"target_available": False},
         (ControllerState.READY, "target_missing")),
        ({"collection_present": True},
         (ControllerState.DEGRADED, "collection_node_present")),
    ])
def test_gate_failure_reports_specific_state_and_reason(
        ready_inputs, change, expected):
    assert readiness(
        replace(ready_inputs, **change), FreshnessLimits()) == expected


def test_freshness_configuration_rejects_nonpositive_values():
    with pytest.raises(ValueError, match="positive"):
        FreshnessLimits(marker_s=0.0)


def test_feedback_pair_skew_uses_source_timestamps_only():
    assert paired_source_skew_s(10_000_000_000, 10_004_000_000) == pytest.approx(
        0.004)
    assert paired_source_skew_s(None, 10_004_000_000) is None
    assert paired_source_skew_s(0, 10_004_000_000) is None


def test_fresh_raw_encoder_allows_only_bounded_processing_pause(ready_inputs):
    lagged = replace(
        ready_inputs, encoder_age_s=0.18, encoder_receive_age_s=0.01)
    limits = FreshnessLimits(feedback_s=0.15)

    assert recoverable_encoder_processing_lag(
        lagged, limits, catchup_elapsed_s=0.02,
        catchup_timeout_s=0.35)
    assert not recoverable_encoder_processing_lag(
        lagged, limits, catchup_elapsed_s=0.36,
        catchup_timeout_s=0.35)
    assert not recoverable_encoder_processing_lag(
        replace(lagged, encoder_receive_age_s=0.16), limits,
        catchup_elapsed_s=0.02, catchup_timeout_s=0.35)
    assert not recoverable_encoder_processing_lag(
        replace(lagged, encoder_valid=False), limits,
        catchup_elapsed_s=0.02, catchup_timeout_s=0.35)
    assert not recoverable_encoder_processing_lag(
        replace(lagged, marker_age_s=0.16), limits,
        catchup_elapsed_s=0.02, catchup_timeout_s=0.35)


def test_encoder_processing_pause_rejects_nonpositive_timeout(ready_inputs):
    with pytest.raises(ValueError, match="catch-up timeout"):
        recoverable_encoder_processing_lag(
            ready_inputs, FreshnessLimits(), catchup_elapsed_s=0.0,
            catchup_timeout_s=0.0)


def test_transient_marker_rejection_uses_configured_rejection_budget(
        ready_inputs):
    transient = replace(
        ready_inputs,
        estimator_health="DEGRADED",
        last_marker_update_rejected=True,
        consecutive_rejections=1)
    assert readiness(transient, FreshnessLimits()) == (
        ControllerState.READY, "ready")


def test_marker_rejection_budget_still_fails_closed_at_threshold(
        ready_inputs):
    exhausted = replace(
        ready_inputs,
        estimator_health="DEGRADED",
        last_marker_update_rejected=True,
        consecutive_rejections=3)
    assert readiness(exhausted, FreshnessLimits()) == (
        ControllerState.DEGRADED, "repeated_marker_rejection:none")


def test_non_marker_estimator_degradation_remains_immediate(
        ready_inputs):
    internal = replace(
        ready_inputs,
        estimator_health="DEGRADED",
        last_marker_update_rejected=False,
        consecutive_rejections=1)
    assert readiness(internal, FreshnessLimits()) == (
        ControllerState.DEGRADED, "estimator_degraded")


def test_marker_rejection_fault_preserves_last_rejection_reason(ready_inputs):
    exhausted = replace(
        ready_inputs,
        estimator_health="DEGRADED",
        last_marker_update_rejected=True,
        last_marker_update_reason="marker_outlier",
        consecutive_rejections=3)
    assert readiness(exhausted, FreshnessLimits()) == (
        ControllerState.DEGRADED,
        "repeated_marker_rejection:marker_outlier")
