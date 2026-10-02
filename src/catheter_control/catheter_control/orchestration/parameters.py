"""Declare the controller ROS parameter surface in one place."""
from __future__ import annotations

import os
from pathlib import Path


def declare_parameters(node, *, source_name):
    root = os.environ.get(
        "CR_META_LNN_ROOT", "/home/chen-lab/Yifan/cr_meta_lnn")
    common = os.environ.get(
        "CR_COMMON_ROOT", "/home/chen-lab/Yifan/cr-common")
    node.declare_parameter("source_name", source_name)
    node.declare_parameter("command_output_enabled", False)
    node.declare_parameter("simulation_state_reset_enabled", False)
    node.declare_parameter("frame_id", "robot_base")
    node.declare_parameter("cr_meta_lnn_root", root)
    node.declare_parameter("cr_common_root", common)
    node.declare_parameter(
        "v171_distal_checkpoint",
        str(Path(root) / "artifacts" / "deployed"
            / "20260929_175554_grouped_no_rotation"
            / "real_distal_first_order_v171_multistep_map_em.pt"))
    node.declare_parameter(
        "jacobian_initialization_json",
        str(Path(root) / "artifacts" / "deployed"
            / "20260929_175554_grouped_no_rotation"
            / "real_joint_local_distal_v174.json"))
    node.declare_parameter("interface_transmission_checkpoint", "")
    node.declare_parameter("distal_tendon_allocation_checkpoint", "")
    # Stay in prequential shadow mode until causal replay is reviewed.
    node.declare_parameter("adaptation_enabled", False)
    node.declare_parameter("adaptation_minimum_observations", 4)
    node.declare_parameter("adaptation_minimum_normalized_action", 0.01)
    node.declare_parameter("adaptation_minimum_rotation_deg", 0.10)
    node.declare_parameter("adaptation_minimum_translation_mm", 0.30)
    node.declare_parameter("adaptation_minimum_response_snr", 3.0)
    node.declare_parameter("adaptation_maximum_window_s", 1.0)
    node.declare_parameter("adaptation_directional_purity", 0.80)
    node.declare_parameter(
        "adaptation_reversal_holdoff_normalized_action", 0.02)
    node.declare_parameter(
        "adaptation_reversal_holdoff_normalized_action_shaft_0", -1.0)
    node.declare_parameter(
        "adaptation_reversal_holdoff_normalized_action_shaft_1", -1.0)
    node.declare_parameter(
        "adaptation_reversal_holdoff_normalized_action_shaft_2", -1.0)
    node.declare_parameter("adaptation_confirmation_windows", 2)
    node.declare_parameter("adaptation_consistency_cosine", 0.50)
    node.declare_parameter("adaptation_minimum_column_gain", 0.25)
    node.declare_parameter("adaptation_maximum_column_gain", 4.0)
    node.declare_parameter(
        "adaptation_maximum_direction_deviation_deg", 60.0)
    node.declare_parameter("limits_file", "")
    node.declare_parameter("catheter", "imricor_test")
    # Negative entries inherit the hardware profile.  Zero disables an
    # axis only inside this controller; manager/firmware limits remain the
    # final independent safety authority.
    node.declare_parameter("controller_velocity_max", [-1.0]*6)
    node.declare_parameter("device", "cpu")
    node.declare_parameter("marker_estimator", "gauss_newton")
    node.declare_parameter("estimator_filter_initial_covariance", 0.25)
    node.declare_parameter("estimator_filter_process_std_sqrt_s", 1.0)
    node.declare_parameter("estimator_initial_roll_hypotheses", 24)
    node.declare_parameter(
        "estimator_history_reconciliation_enabled", True)
    node.declare_parameter(
        "estimator_history_reconciliation_maximum_shift", 120.0)
    node.declare_parameter("horizon_steps", 4)
    node.declare_parameter("rollout_step_s", 0.04)
    node.declare_parameter("mppi_point_rollout_step_s", 0.0)
    node.declare_parameter("mppi_point_rollout_coarse_steps", False)
    node.declare_parameter("mppi_path_rollout_coarse_steps", False)
    node.declare_parameter("mppi_point_prediction_tail_steps", 0)
    node.declare_parameter("mppi_point_prediction_tail_step_s", 0.12)
    node.declare_parameter("samples", 32)
    node.declare_parameter("mppi_noise_std", [4.0, 20.0, 2.0])
    node.declare_parameter("mppi_noise_correlation", 0.65)
    node.declare_parameter("mppi_exploration_fraction", 0.15)
    node.declare_parameter(
        "mppi_reversal_backlash_rad", [0.0, 0.0, 0.0])
    node.declare_parameter("backlash_compensation_enabled", False)
    node.declare_parameter("backlash_width_rad", [0.0, 0.0, 0.0])
    node.declare_parameter(
        "backlash_width_positive_rad", [0.0, 0.0, 0.0])
    node.declare_parameter(
        "backlash_width_negative_rad", [0.0, 0.0, 0.0])
    node.declare_parameter(
        "backlash_takeup_velocity", [8.0, 40.0, 4.5])
    node.declare_parameter(
        "backlash_minimum_motor_increment_rad", 0.01)
    node.declare_parameter(
        "backlash_minimum_transmitted_increment_rad", 0.10)
    node.declare_parameter("backlash_directional_purity", 0.90)
    node.declare_parameter(
        "backlash_response_direction_cosine", 0.50)
    node.declare_parameter(
        "backlash_minimum_response_evidence", 0.50)
    node.declare_parameter(
        "backlash_minimum_distal_bending_increment", 0.05)
    node.declare_parameter("backlash_width_learning_rate", 0.05)
    node.declare_parameter(
        "backlash_engagement_confirmation_observations", 1)
    node.declare_parameter(
        "backlash_provisional_rejection_observations", 2)
    node.declare_parameter("engaged_gain_enabled", False)
    node.declare_parameter("engaged_gain_minimum", 0.10)
    node.declare_parameter("engaged_gain_maximum", 2.0)
    node.declare_parameter("engaged_gain_prior_mean", [1.0, 1.0])
    node.declare_parameter("engaged_gain_prior_log_std", 0.70)
    node.declare_parameter("engaged_gain_reversal_log_std", 0.80)
    node.declare_parameter(
        "engaged_gain_process_log_std_sqrt_s", 0.05)
    node.declare_parameter("engaged_gain_observation_std", 0.08)
    node.declare_parameter(
        "engaged_gain_minimum_nominal_increment", 0.02)
    node.declare_parameter("engaged_gain_huber_sigma", 3.0)
    node.declare_parameter(
        "engaged_gain_maximum_normalized_innovation", 8.0)
    node.declare_parameter(
        "engaged_gain_contradiction_log_std", 0.70)
    node.declare_parameter("engaged_gain_confidence_log_width", 0.50)
    node.declare_parameter("engaged_gain_minimum_updates", 2)
    node.declare_parameter("engaged_gain_credible_sigma", 1.645)
    node.declare_parameter("mppi_engaged_gain_scenarios", False)
    node.declare_parameter("mppi_engaged_gain_risk_beta", 0.50)
    node.declare_parameter("mppi_engaged_gain_cvar_alpha", 0.67)
    node.declare_parameter(
        "mppi_engaged_gain_maximum_first_step_shift", 0.0)
    node.declare_parameter(
        "mppi_engaged_gain_learning_velocity_scale", 1.0)
    node.declare_parameter("mppi_capture_radius_mm", 0.0)
    node.declare_parameter(
        "mppi_capture_minimum_terminal_improvement_mm", 0.0)
    node.declare_parameter("mppi_capture_hold_s", 0.0)
    node.declare_parameter(
        "mppi_capture_response_minimum_prediction_mm", 0.25)
    node.declare_parameter(
        "mppi_capture_response_minimum_ratio", 0.50)
    node.declare_parameter("mppi_transmission_aware_rollout", False)
    node.declare_parameter("takeup_transaction_enabled", False)
    node.declare_parameter(
        "takeup_confirmation_hold_timeout_s", 1.0)
    node.declare_parameter("mppi_rotation_direction_latch", False)
    node.declare_parameter("mppi_best_candidate_guard", True)
    node.declare_parameter("mppi_grouped_mode_sampling", True)
    node.declare_parameter("mppi_takeup_limit_reserve_scale", 1.0)
    node.declare_parameter("mppi_takeup_risk_weight", 4.0)
    node.declare_parameter("mppi_takeup_confirmation_time_s", 0.10)
    node.declare_parameter("reversal_scheduler_enabled", True)
    node.declare_parameter("reversal_scheduler_required_plans", 3)
    node.declare_parameter(
        "reversal_scheduler_minimum_absolute_cost_improvement", 5.0)
    node.declare_parameter(
        "reversal_scheduler_minimum_fractional_cost_improvement", 0.0)
    node.declare_parameter(
        "reversal_scheduler_minimum_terminal_error_improvement_mm", 0.25)
    node.declare_parameter(
        "reversal_scheduler_minimum_accepted_observations", 3)
    node.declare_parameter("reversal_scheduler_cooldown_s", 1.0)
    node.declare_parameter("mppi_seed", 0)
    node.declare_parameter("planning_deadline_s", 0.06)
    node.declare_parameter("plan_rate_hz", 15.0)
    node.declare_parameter("command_rate_hz", 100.0)
    node.declare_parameter("diagnostic_rate_hz", 10.0)
    node.declare_parameter("encoder_update_rate_hz", 50.0)
    node.declare_parameter("marker_update_rate_hz", 20.0)
    node.declare_parameter("torch_intraop_threads", 2)
    node.declare_parameter("torch_interop_threads", 1)
    node.declare_parameter("manager_timeout_s", 0.5)
    node.declare_parameter("feedback_timeout_s", 0.15)
    # If raw ENC remains fresh while estimator correction runs, command
    # zero for this bounded window before retaining encoder_stale.
    node.declare_parameter("estimator_catchup_timeout_s", 0.35)
    node.declare_parameter("marker_timeout_s", 0.15)
    node.declare_parameter("maximum_marker_lag_s", 0.15)
    node.declare_parameter("marker_diagnostic_timeout_s", 0.5)
    node.declare_parameter("feedback_pair_max_skew_s", 0.15)
    node.declare_parameter("command_timeout_s", 0.15)
    node.declare_parameter("path_reference_timeout_s", 0.20)
    node.declare_parameter("tip_error_log_rate_hz", 1.0)
    node.declare_parameter("mode_settle_s", 0.10)
    node.declare_parameter("initialization_observations", 8)
    node.declare_parameter("initialization_consecutive_inliers", 2)
    node.declare_parameter("maximum_marker_rejections", 3)
    node.declare_parameter("maximum_planner_deadline_misses", 3)
    node.declare_parameter(
        "marker_topic", "/shape_tracking/markers")
    node.declare_parameter(
        "marker_diagnostic_topic", "/shape_tracking/marker_status")
