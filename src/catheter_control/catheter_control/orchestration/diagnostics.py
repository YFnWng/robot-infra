"""Controller diagnostic formatting and callback instrumentation."""

from functools import wraps
import json

import numpy as np

def instrument_timer(name):
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


def plan_diagnostic_values(plan, terminal_error_mm=None):
    """Serialize an immutable planner result without ROS dependencies."""
    if plan is None:
        return {}
    values = {
        "plan_elapsed_ms": f"{1e3*plan.elapsed_s:.3f}",
        "best_cost": f"{plan.best_cost:.6g}",
        "effective_samples": (
            f"{plan.effective_samples:.3f}"),
        "plan_sample_projection_ms": (
            f"{plan.sample_projection_ms:.3f}"),
        "plan_rollout_ms": f"{plan.rollout_ms:.3f}",
        "plan_engaged_gain_scenario_count": str(
            plan.engaged_gain_scenario_count),
        "plan_selected_engaged_gain_scenarios": json.dumps(
            plan.selected_engaged_gain_scenarios.tolist()),
        "plan_selected_gain_tracking_costs": json.dumps(
            plan.selected_gain_tracking_costs.tolist()),
        "plan_selected_maximum_first_step_lambda_shift": format(
            plan
            .selected_maximum_first_step_lambda_shift, ".6g"),
        "plan_cost_weighting_ms": (
            f"{plan.cost_weighting_ms:.3f}"),
        "plan_update_projection_ms": (
            f"{plan.update_projection_ms:.3f}"),
        "plan_command_prediction_kind": (
            "scored_feasible_candidate"
            if plan.scored_candidate_guard_applied else
            "weighted_feasible_candidate_mean"),
        "plan_transmission_prediction_applied": str(
            plan.transmission_prediction_applied),
        "plan_rotation_direction_latched": str(
            plan.rotation_direction_latched),
        "plan_takeup_direction_latched": str(
            plan.takeup_direction_latched),
        "plan_best_candidate_selected": str(
            plan.best_candidate_selected),
        "plan_scored_candidate_guard_applied": str(
            plan.scored_candidate_guard_applied),
        "plan_blocked_motor_direction": (
            "none" if plan.blocked_motor_direction
            is None else json.dumps(
                plan.blocked_motor_direction.tolist())),
        "plan_blocked_candidate_count": str(
            plan.blocked_candidate_count),
        "plan_direction_lease": json.dumps(
            plan.direction_lease.tolist()),
        "plan_approved_reversal_direction": json.dumps(
            plan.approved_reversal_direction.tolist()),
        "plan_proposed_reversal_direction": json.dumps(
            plan.proposed_reversal_direction.tolist()),
        "plan_direction_lease_applied": str(
            plan.direction_lease_applied),
        "plan_unrestricted_candidate_index": str(
            plan.unrestricted_candidate_index),
        "plan_lease_constrained_candidate_index": str(
            plan.lease_constrained_candidate_index),
        "plan_unrestricted_total_cost": format(
            plan.unrestricted_total_cost, ".6g"),
        "plan_lease_constrained_total_cost": format(
            plan.lease_constrained_total_cost, ".6g"),
        "plan_reversal_axis_cost_improvement": json.dumps(
            plan.reversal_axis_cost_improvement.tolist()),
        "plan_reversal_axis_terminal_error_improvement_mm": (
            json.dumps(
                plan
                .reversal_axis_terminal_error_improvement_mm
                .tolist())),
        "plan_hold_branch_applied": str(
            plan.hold_branch_applied),
        "plan_hold_branch_terminal_error_mm": format(
            plan.hold_branch_terminal_error_mm, ".6g"),
        "plan_zero_terminal_error_mm": format(
            plan.zero_terminal_error_mm, ".6g"),
        "plan_raw_zero_terminal_error_mm": format(
            plan.raw_zero_terminal_error_mm, ".6g"),
        "plan_capture_passive_response_scale": format(
            plan.capture_passive_response_scale, ".6g"),
        "plan_selected_gain_learning_velocity_scale": format(
            plan
            .selected_gain_learning_velocity_scale, ".6g"),
        "plan_proposal_group_count": str(
            plan.proposal_group_count),
        "plan_prediction_horizon_steps": str(
            plan.prediction_horizon_steps),
        "plan_prediction_horizon_s": format(
            plan.prediction_horizon_s, ".6g"),
        "plan_tendon_probe_candidate_count": str(
            plan.tendon_probe_candidate_count),
        "plan_selected_tendon_probe": str(
            plan.selected_tendon_probe),
        "plan_selected_reversal_mask": str(
            plan.selected_reversal_mask),
        "plan_unrestricted_reversal_mask": str(
            plan.unrestricted_reversal_mask),
        "plan_mode_best_total_cost": json.dumps(
            plan.mode_best_total_cost.tolist(),
            separators=(",", ":")),
        "plan_takeup_joint_position_offset": json.dumps(
            plan.takeup_joint_position_offset.tolist(),
            separators=(",", ":")),
        "plan_selected_takeup_risk_s": format(
            plan.selected_takeup_risk_s, ".6g"),
        "plan_selected_takeup_risk_cost": format(
            plan.selected_takeup_risk_cost, ".6g"),
        "plan_selected_switch_count": str(
            plan.selected_switch_count),
        "plan_selected_candidate_index": str(
            plan.selected_candidate_index),
        "plan_selected_total_cost": (
            f"{plan.selected_total_cost:.6g}"),
        "plan_zero_total_cost": (
            f"{plan.zero_total_cost:.6g}"),
        "plan_weighted_tracking_cost": (
            f"{plan.weighted_tracking_cost:.6g}"),
        "plan_zero_tracking_cost": (
            f"{plan.zero_tracking_cost:.6g}"),
        "plan_best_tracking_cost": (
            f"{plan.best_tracking_cost:.6g}"),
        "plan_logical_velocity_sequence": json.dumps(
            plan.logical_velocity_sequence.tolist(),
            separators=(",", ":")),
        "plan_motor_radians_per_second_sequence": json.dumps(
            plan.motor_radians_per_second_sequence
            .tolist(),
            separators=(",", ":")),
        "plan_compensated_motor_radians_per_second_sequence": (
            "none" if plan
            .compensated_motor_radians_per_second_sequence is None
            else json.dumps(
                plan
                .compensated_motor_radians_per_second_sequence
                .tolist(), separators=(",", ":"))),
        "plan_transmitted_motor_radians_per_second_sequence": (
            "none" if plan
            .transmitted_motor_radians_per_second_sequence is None
            else json.dumps(
                plan
                .transmitted_motor_radians_per_second_sequence
                .tolist(), separators=(",", ":"))),
    }
    if plan.command_tip_sequence_m is not None:
        terminal = plan.command_tip_sequence_m[-1]
        values["plan_command_predicted_terminal_tip_m"] = json.dumps(
            terminal.tolist(), separators=(",", ":"))
        if terminal_error_mm is not None:
            values["plan_command_predicted_terminal_error_mm"] = (
                f"{terminal_error_mm:.6g}")
    return values


def response_diagnostic_values(response, pending_count):
    """Serialize a completed horizon response without ROS dependencies."""
    if response is None:
        return {}
    return {
        "response_forecast_start_timestamp_ns": str(
            response.start_timestamp_ns),
        "response_forecast_due_timestamp_ns": str(
            response.due_timestamp_ns),
        "response_forecast_horizon_ms": format(
            1e-6*(response.due_timestamp_ns-response.start_timestamp_ns),
            ".6g"),
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
            pending_count),
    }


def model_diagnostic_values(model_diagnostics, model_valid):
    """Serialize the stable public subset of learned-runtime diagnostics."""
    values = {}
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
        if key in model_diagnostics:
            values[f"model_{key}"] = str(
                model_diagnostics[key])
    for key in ("initialization_inlier_streak",
                "initialization_complete"):
        if key in model_diagnostics:
            values[f"model_{key}"] = str(
                model_diagnostics[key])
    if "jacobian" in model_diagnostics:
        values["model_jacobian"] = json.dumps(
            model_diagnostics["jacobian"], separators=(",", ":"))
    for key in ("raw_encoder_counts_first_three", "raw_motor_angle_rad",
                "motor_angle_rad", "downstream"):
        if key in model_diagnostics:
            values[f"model_{key}"] = json.dumps(
                model_diagnostics[key], separators=(",", ":"))
    values["model_valid"] = str(model_valid)
    return values


def marker_diagnostic_values(result):
    """Serialize the most recent marker correction result."""
    return {
        "marker_update_reason": (
            "none" if result is None
            else result.reason),
        "marker_rms_before_mm": (
            "none" if result is None
            or result.rms_before_mm is None
            else f"{result.rms_before_mm:.6g}"),
        "marker_rms_after_mm": (
            "none" if result is None
            or result.rms_after_mm is None
            else f"{result.rms_after_mm:.6g}"),
        "marker_maximum_residual_before_mm": (
            "none" if result is None
            or result.maximum_residual_before_mm
            is None else format(
                result.maximum_residual_before_mm,
                ".6g")),
        "marker_maximum_residual_after_mm": (
            "none" if result is None
            or result.maximum_residual_after_mm
            is None else format(
                result.maximum_residual_after_mm,
                ".6g")),
        "marker_nis": (
            "none" if result is None
            or result.normalized_innovation is None
            else format(
                result.normalized_innovation,
                ".6g")),
        "marker_postfit_normalized_residual": (
            "none" if result is None
            or result.normalized_innovation is None
            else format(
                result.normalized_innovation,
                ".6g")),
        "marker_innovation_nis": (
            "none" if result is None
            or result.innovation_nis is None
            else format(
                result.innovation_nis, ".6g")),
        "marker_innovation_nis_per_dof": (
            "none" if result is None
            or result.innovation_nis_per_dof is None
            else format(
                result.innovation_nis_per_dof,
                ".6g")),
        "marker_innovation_dof": (
            "none" if result is None
            else str(result.innovation_dof)),
        "marker_observable_rank": (
            "none" if result is None
            else str(result.observable_rank)),
    }


def transmission_diagnostic_values(
        command, effective_command, compensation_enabled, raw_encoder_counts,
        snapshot, transaction_enabled, arbiter):
    """Serialize transmission belief and take-up transaction state."""
    return {
        "command": json.dumps(command.tolist()),
        "effective_command": json.dumps(
            effective_command.tolist()),
        "backlash_compensation_enabled": str(
            compensation_enabled),
        "backlash_model_encoder_input": (
            "estimated_transmitted"
            if compensation_enabled else "raw_shaft"),
        "upstream_raw_encoder_counts_first_three": (
            "none" if raw_encoder_counts is None
            else json.dumps(
                raw_encoder_counts.tolist(),
                separators=(",", ":"))),
        "backlash_width_rad": json.dumps(
            snapshot.width_rad.tolist()),
        "backlash_width_positive_rad": json.dumps(
            snapshot.width_positive_rad.tolist()),
        "backlash_width_negative_rad": json.dumps(
            snapshot.width_negative_rad.tolist()),
        "backlash_remaining_rad": json.dumps(
            snapshot.remaining_rad.tolist()),
        "backlash_remaining_lower_rad": json.dumps(
            snapshot.remaining_lower_rad.tolist()),
        "backlash_remaining_upper_rad": json.dumps(
            snapshot.remaining_upper_rad.tolist()),
        "backlash_width_positive_interval_rad": json.dumps([
            snapshot.width_positive_lower_rad.tolist(),
            snapshot.width_positive_upper_rad.tolist()]),
        "backlash_width_negative_interval_rad": json.dumps([
            snapshot.width_negative_lower_rad.tolist(),
            snapshot.width_negative_upper_rad.tolist()]),
        "backlash_reversal_start_motor_rad": json.dumps(
            snapshot.reversal_start_motor_rad.tolist()),
        "backlash_engagement_anchor_motor_rad": json.dumps(
            snapshot.engagement_anchor_motor_rad.tolist()),
        "backlash_accumulated_takeup_rad": json.dumps(
            snapshot.accumulated_takeup_rad.tolist()),
        "backlash_effective_motor_rad": json.dumps(
            snapshot.effective_motor_rad.tolist()),
        "backlash_effective_motor_uncertainty_rad": json.dumps(
            snapshot
            .effective_motor_uncertainty_rad.tolist()),
        "backlash_last_evidence_timestamp_ns": json.dumps(
            snapshot.last_evidence_timestamp_ns.tolist()),
        "backlash_motion_direction": json.dumps(
            snapshot.motion_direction.tolist()),
        "backlash_engaged_direction": json.dumps(
            snapshot.engaged_direction.tolist()),
        "backlash_confidence": json.dumps(
            snapshot.confidence.tolist()),
        "backlash_confirmation_count": json.dumps(
            snapshot.confirmation_count.tolist()),
        "backlash_inferred_transmitted_increment_rad": json.dumps(
            snapshot
            .inferred_transmitted_increment_rad.tolist()),
        "backlash_response_evidence": json.dumps(
            snapshot.response_evidence.tolist()),
        "backlash_response_classification": json.dumps(
            list(snapshot.response_classification)),
        "backlash_provisional_rejection_count": json.dumps(
            snapshot
            .provisional_rejection_count.tolist()),
        "backlash_joint_response_residual": format(
            snapshot.joint_response_residual, ".6g"),
        "backlash_distal_bending_increment": format(
            snapshot.distal_bending_increment, ".6g"),
        "backlash_tendon_distal_response_evidence": format(
            snapshot
            .tendon_distal_response_evidence, ".6g"),
        "backlash_tendon_distal_response_confirmed": str(
            snapshot
            .tendon_distal_response_confirmed),
        "backlash_phase": json.dumps(
            list(snapshot.phase)),
        "engaged_gain_enabled": str(
            snapshot.engaged_gain.enabled),
        "engaged_gain_mean": json.dumps(
            snapshot.engaged_gain.mean.tolist()),
        "engaged_gain_lower": json.dumps(
            snapshot.engaged_gain.lower.tolist()),
        "engaged_gain_upper": json.dumps(
            snapshot.engaged_gain.upper.tolist()),
        "engaged_gain_update_count": json.dumps(
            snapshot.engaged_gain.update_count.tolist()),
        "engaged_gain_status": json.dumps(
            snapshot.engaged_gain.status),
        "engaged_gain_last_reason": json.dumps(
            snapshot.engaged_gain.last_reason),
        "takeup_transaction_enabled": str(
            transaction_enabled),
        "takeup_transaction_state": arbiter.state,
        "takeup_transaction_generation": str(
            arbiter.generation),
        "takeup_transaction_active_mask": json.dumps(
            arbiter.active_mask.astype(int).tolist()),
        "takeup_transaction_pending_mask": json.dumps(
            arbiter.pending_mask.astype(int).tolist()),
        "takeup_transaction_direction": json.dumps(
            arbiter.direction.tolist()),
        "takeup_transaction_saturated_mask": json.dumps(
            arbiter.saturated_mask.astype(int).tolist()),
        "takeup_transaction_leakage_mask": json.dumps(
            arbiter.leakage_mask.astype(int).tolist()),
        "takeup_requested_motor_rad_s": json.dumps(
            arbiter
            .requested_motor_radians_per_second.tolist()),
        "takeup_realized_motor_rad_s": json.dumps(
            arbiter
            .realized_motor_radians_per_second.tolist()),
    }


def planner_policy_diagnostic_values(
        config, capture_diagnostics, scheduler, arbiter):
    """Serialize MPPI, capture, and reversal-policy state."""
    return {
        "reversal_scheduler_enabled": str(
            scheduler.config.enabled),
        "reversal_scheduler_gating_active": str(
            scheduler.config.enabled
            and not config.grouped_mode_sampling),
        "mppi_grouped_mode_sampling": str(
            config.grouped_mode_sampling),
        "mppi_best_candidate_guard": str(
            config.best_candidate_guard),
        "reversal_mode_selector": (
            "grouped_mppi" if config.grouped_mode_sampling
            else ("legacy_reversal_scheduler"
                  if scheduler.config.enabled
                  else "plain_mppi")),
        "mppi_samples": str(config.samples),
        "mppi_engaged_gain_scenarios": str(
            config.engaged_gain_scenarios),
        "mppi_engaged_gain_risk_beta": format(
            config.engaged_gain_risk_beta, ".6g"),
        "mppi_engaged_gain_cvar_alpha": format(
            config.engaged_gain_cvar_alpha, ".6g"),
        "mppi_engaged_gain_maximum_first_step_shift": format(
            config
            .engaged_gain_maximum_first_step_shift, ".6g"),
        "mppi_engaged_gain_learning_velocity_scale": format(
            config
            .engaged_gain_learning_velocity_scale, ".6g"),
        "mppi_capture_radius_mm": format(
            config.capture_radius_mm, ".6g"),
        "mppi_capture_minimum_terminal_improvement_mm": format(
            config
            .capture_minimum_terminal_improvement_mm, ".6g"),
        "mppi_capture_hold_s": format(
            config.capture_hold_s, ".6g"),
        "mppi_capture_response_minimum_prediction_mm": format(
            config
            .capture_response_minimum_prediction_mm, ".6g"),
        "mppi_capture_response_minimum_ratio": format(
            config.capture_response_minimum_ratio, ".6g"),
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
            config.point_rollout_step_s, ".6g"),
        "mppi_point_rollout_coarse_steps": str(
            config.point_rollout_coarse_steps).lower(),
        "mppi_path_rollout_coarse_steps": str(
            config.path_rollout_coarse_steps).lower(),
        "mppi_point_prediction_tail_steps": str(
            config.point_prediction_tail_steps),
        "mppi_point_prediction_tail_step_s": format(
            config.point_prediction_tail_step_s, ".6g"),
        "mppi_takeup_risk_cost_weight": format(
            config.takeup_risk_weight, ".6g"),
        "mppi_takeup_confirmation_time_s": format(
            config.takeup_confirmation_time_s, ".6g"),
        "takeup_confirmation_hold_timeout_s": format(
            arbiter.confirmation_hold_timeout_s, ".6g"),
        "mppi_active_proposal_groups": str(
            1 << int(np.count_nonzero(
                scheduler.lease_direction))
            if config.grouped_mode_sampling else 1),
        "mppi_minimum_samples_per_active_group": str(
            config.samples // (
                1 << int(np.count_nonzero(
                    scheduler.lease_direction)))
            if config.grouped_mode_sampling else
            config.samples),
        "reversal_lease_direction": json.dumps(
            scheduler.lease_direction.tolist()),
        "reversal_pending_direction": json.dumps(
            scheduler.pending_direction.tolist()),
        "reversal_pending_count": json.dumps(
            scheduler.pending_count.tolist()),
        "reversal_approved_direction": json.dumps(
            scheduler.approved_direction.tolist()),
        "reversal_scheduler_reason": (
            scheduler.reason),
        "reversal_absolute_cost_improvement": format(
            scheduler.last_absolute_improvement,
            ".6g"),
        "reversal_fractional_cost_improvement": format(
            scheduler.last_fractional_improvement,
            ".6g"),
        "reversal_terminal_error_improvement_mm": json.dumps(
            scheduler.last_terminal_improvement_mm
            .tolist()),
    }


def runtime_diagnostic_values(
        *, blocked_motor_direction, takeup_saturation_position,
        takeup_saturation_position_timestamp_ns,
        takeup_saturation_release_reason, position_valid, encoder_valid,
        planner_snapshot_source_time, now, torch_intraop_threads,
        torch_interop_threads, contract, raw_response_during_interface_takeup,
        takeup_response_free_mask):
    """Serialize controller runtime state not owned by a domain component."""
    return {
        "planner_blocked_motor_direction": json.dumps(
            blocked_motor_direction.tolist()),
        "takeup_saturation_position": (
            "none" if takeup_saturation_position is None else
            json.dumps(
                takeup_saturation_position.tolist(),
                separators=(",", ":"))),
        "takeup_saturation_position_timestamp_ns": (
            "none" if takeup_saturation_position_timestamp_ns
            is None else str(
                takeup_saturation_position_timestamp_ns)),
        "takeup_saturation_release_reason": (
            takeup_saturation_release_reason),
        "position_feedback_valid": str(position_valid),
        "encoder_feedback_valid": str(encoder_valid),
        "estimator_runtime_owner": "single_timer",
        "planner_state_exchange": "replace_only_snapshot",
        "planner_snapshot_age_ms": (
            "none" if planner_snapshot_source_time is None
            else f"{1e3*max(0.0, now-planner_snapshot_source_time):.3f}"),
        "torch_intraop_threads": str(torch_intraop_threads),
        "torch_interop_threads": str(torch_interop_threads),
        "controller_velocity_min": json.dumps(
            contract.velocity_min.tolist(), separators=(",", ":")),
        "controller_velocity_max": json.dumps(
            contract.velocity_max.tolist(), separators=(",", ":")),
        "raw_response_during_interface_takeup": json.dumps(
            raw_response_during_interface_takeup.tolist(),
            separators=(",", ":")),
        "takeup_response_free_mask": json.dumps(
            takeup_response_free_mask.tolist(),
            separators=(",", ":")),
    }
