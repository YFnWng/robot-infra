from types import SimpleNamespace

import numpy as np

from catheter_control.orchestration.diagnostics import (
    marker_diagnostic_values,
    model_diagnostic_values,
    plan_diagnostic_values,
    planner_policy_diagnostic_values,
    response_diagnostic_values,
    runtime_diagnostic_values,
    transmission_diagnostic_values,
)
from catheter_control.planning.mppi import MppiConfig, MppiPlan
from catheter_control.planning.tracking import TipForecastResult
from catheter_control.transmission.backlash import (
    BacklashConfig, BacklashFeedforwardCompensator, BacklashStateEstimator,
    TakeupTransactionArbiter,
)
from catheter_control.transmission.reversal_scheduler import (
    ReversalDirectionScheduler, ReversalSchedulerConfig,
)


def _plan():
    return MppiPlan(
        command_logical_velocity=np.zeros(3),
        logical_velocity_sequence=np.zeros((4, 3)),
        motor_radians_per_second_sequence=np.zeros((4, 3)),
        best_tip_sequence_m=None,
        best_cost=1.25,
        effective_samples=3.5,
        elapsed_s=0.0123,
        valid=True,
        reason="ok",
        command_tip_sequence_m=np.array([
            [0.1, 0.2, 0.3],
            [0.4, 0.5, 0.6],
        ]),
    )


def test_plan_diagnostics_preserve_public_surface_and_formatting():
    assert plan_diagnostic_values(None) == {}

    values = plan_diagnostic_values(_plan(), terminal_error_mm=2.75)

    assert len(values) == 59
    assert list(values)[:3] == [
        "plan_elapsed_ms", "best_cost", "effective_samples"]
    assert list(values)[-2:] == [
        "plan_command_predicted_terminal_tip_m",
        "plan_command_predicted_terminal_error_mm",
    ]
    assert values["plan_elapsed_ms"] == "12.300"
    assert values["best_cost"] == "1.25"
    assert values["effective_samples"] == "3.500"
    assert values["plan_command_prediction_kind"] == (
        "weighted_feasible_candidate_mean")
    assert values["plan_blocked_motor_direction"] == "none"
    assert values[
        "plan_compensated_motor_radians_per_second_sequence"] == "none"
    assert values["plan_command_predicted_terminal_tip_m"] == (
        "[0.4,0.5,0.6]")
    assert values["plan_command_predicted_terminal_error_mm"] == "2.75"


def test_response_diagnostics_preserve_units_and_optional_direction():
    response = TipForecastResult(
        start_timestamp_ns=1_000_000_000,
        start_observation_timestamp_ns=990_000_000,
        due_timestamp_ns=1_200_000_000,
        observation_timestamp_ns=1_250_000_000,
        predicted_delta_mm=np.array([1.0, 2.0, 3.0]),
        measured_delta_mm=np.array([0.5, 1.0, 1.5]),
        endpoint_error_mm=np.array([-0.5, -1.0, -1.5]),
        endpoint_error_norm_mm=1.8708,
        direction_cosine=None,
    )

    values = response_diagnostic_values(response, pending_count=4)

    assert len(values) == 12
    assert values["response_forecast_horizon_ms"] == "200"
    assert values["response_observation_lateness_ms"] == "50"
    assert values["response_start_observation_skew_ms"] == "10"
    assert values["response_predicted_tip_delta_mm"] == "[1.0,2.0,3.0]"
    assert values["response_direction_cosine"] == "none"
    assert values["response_pending_forecasts"] == "4"
    assert response_diagnostic_values(None, pending_count=4) == {}


def test_model_diagnostics_publish_only_the_stable_public_subset():
    values = model_diagnostic_values({
        "distal_sha256": "abc",
        "lambda": 0.5,
        "initialization_complete": True,
        "jacobian": [[1.0, 2.0]],
        "downstream": [3.0, 4.0],
        "private_detail": "omit",
    }, model_valid=False)

    assert values == {
        "model_distal_sha256": "abc",
        "model_lambda": "0.5",
        "model_initialization_complete": "True",
        "model_jacobian": "[[1.0,2.0]]",
        "model_downstream": "[3.0,4.0]",
        "model_valid": "False",
    }


def _transmission_state():
    config = BacklashConfig(width_rad=(1.0, 2.0, 3.0))
    snapshot = BacklashStateEstimator(config).snapshot()
    arbiter = TakeupTransactionArbiter(
        BacklashFeedforwardCompensator(config))
    return snapshot, arbiter


def test_marker_diagnostics_preserve_absent_result_surface():
    values = marker_diagnostic_values(None)

    assert len(values) == 11
    assert set(values.values()) == {"none"}
    assert list(values)[:2] == [
        "marker_update_reason", "marker_rms_before_mm"]


def test_transmission_diagnostics_preserve_belief_and_transaction_state():
    snapshot, arbiter = _transmission_state()

    values = transmission_diagnostic_values(
        np.zeros(6), np.ones(6), True, None, snapshot, True, arbiter)

    assert len(values) == 49
    assert values["command"] == "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    assert values["backlash_model_encoder_input"] == (
        "estimated_transmitted")
    assert values["upstream_raw_encoder_counts_first_three"] == "none"
    assert values["backlash_width_rad"] == "[1.0, 2.0, 3.0]"
    assert values["takeup_transaction_state"] == "READY_TO_PLAN"
    assert values["takeup_transaction_active_mask"] == "[0, 0, 0]"


def test_planner_policy_diagnostics_preserve_mode_and_group_counts():
    _, arbiter = _transmission_state()
    scheduler = ReversalDirectionScheduler(ReversalSchedulerConfig())
    capture = {
        "passive_response_scale": 1.0,
        "last_response_ratio": 0.5,
        "last_response_reason": "consistent",
        "release_count": 2,
        "rearm_blocked": False,
    }

    values = planner_policy_diagnostic_values(
        MppiConfig(), capture, scheduler, arbiter)

    assert len(values) == 39
    assert values["reversal_mode_selector"] == "grouped_mppi"
    assert values["mppi_samples"] == "32"
    assert values["mppi_active_proposal_groups"] == "1"
    assert values["mppi_minimum_samples_per_active_group"] == "32"
    assert values["capture_last_response_reason"] == "consistent"
    assert values["capture_release_count"] == "2"


def test_runtime_diagnostics_preserve_none_and_boolean_encoding():
    contract = SimpleNamespace(
        velocity_min=np.full(6, -1.0),
        velocity_max=np.ones(6),
    )

    values = runtime_diagnostic_values(
        blocked_motor_direction=np.zeros(3),
        takeup_saturation_position=None,
        takeup_saturation_position_timestamp_ns=None,
        takeup_saturation_release_reason="cleared",
        position_valid=True,
        encoder_valid=False,
        planner_snapshot_source_time=None,
        now=1.0,
        torch_intraop_threads=2,
        torch_interop_threads=1,
        contract=contract,
        raw_response_during_interface_takeup=np.array([False, False, True]),
        takeup_response_free_mask=np.array([True, True, False]),
    )

    assert len(values) == 15
    assert values["takeup_saturation_position"] == "none"
    assert values["takeup_saturation_position_timestamp_ns"] == "none"
    assert values["planner_snapshot_age_ms"] == "none"
    assert values["position_feedback_valid"] == "True"
    assert values["encoder_feedback_valid"] == "False"
    assert values["controller_velocity_max"] == "[1.0,1.0,1.0,1.0,1.0,1.0]"
