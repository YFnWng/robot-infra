"""Launch the guarded MPPI node and optional external-drive rosbag."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter


RECORD_TOPICS = [
    "/teleop/control",
    "/teleop/event",
    "/manager/control",
    "/manager/event",
    "/manager/safety_status",
    "/manager/state",
    "/device/state",
    "/device/event",
    "/device/command_tx",
    "/device/transport_status",
    "/shape_tracking/markers",
    "/shape_tracking/marker_status",
    "/collection/events",
    "/catheter_mppi/target_tip",
    "/catheter_mppi/reference_horizon",
    "/catheter_mppi/reference_path",
    "/catheter_mppi/path_reference_point",
    "/catheter_mppi/path_tracking_trace",
    "/catheter_mppi/planned_control",
    "/catheter_mppi/predicted_tip",
    "/catheter_mppi/response_trace",
    "/catheter_mppi/estimator_trace",
    "/catheter_mppi/control_cycle_timing",
    "/catheter_mppi/status",
    "/catheter_mppi/track_tip_trajectory/_action/feedback",
    "/catheter_mppi/track_tip_trajectory/_action/status",
    "/catheter_mppi/track_tip_path/_action/feedback",
    "/catheter_mppi/track_tip_path/_action/status",
    "/parameter_events",
    "/rosout",
]


def _artifact_manifest(path):
    resolved = Path(path).expanduser().resolve()
    result = {"path": str(resolved), "exists": resolved.is_file()}
    if resolved.is_file():
        digest = hashlib.sha256()
        with resolved.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        result.update({"bytes": resolved.stat().st_size,
                       "sha256": digest.hexdigest()})
    return result


def _default_limits():
    try:
        return os.path.join(
            get_package_share_directory("automation"),
            "config", "catheter_limits.yaml")
    except Exception:
        candidate = (Path(__file__).resolve().parents[2]
                     / "automation" / "config" / "catheter_limits.yaml")
        return str(candidate) if candidate.is_file() else ""


def _reject_profile_output_interlock(path, label):
    """Reserve command_output_enabled for the explicit launch argument."""
    resolved = Path(path).expanduser().resolve()
    with resolved.open("r", encoding="utf-8") as stream:
        content = yaml.safe_load(stream) or {}

    def declares(value):
        if isinstance(value, dict):
            return ("command_output_enabled" in value
                    or any(declares(item) for item in value.values()))
        if isinstance(value, list):
            return any(declares(item) for item in value)
        return False

    if declares(content):
        raise RuntimeError(
            f"{label} must not declare command_output_enabled; use the "
            "explicit launch argument")
    return str(resolved)


def _profile_parameters(path, node_name="catheter_mppi"):
    """Resolve the parameters a ROS YAML contributes to one named node.

    Exact node selectors outrank wildcard selectors in ROS 2.  Computing the
    same merge here makes the session manifest describe the effective runtime
    configuration instead of the launch defaults that existed before profile
    overlays were applied.
    """
    resolved = Path(path).expanduser().resolve()
    with resolved.open("r", encoding="utf-8") as stream:
        content = yaml.safe_load(stream) or {}
    if not isinstance(content, dict):
        raise RuntimeError(f"ROS parameter profile is not a mapping: {resolved}")

    merged = {}
    selectors = ("/**", node_name, f"/{node_name}")
    for selector in selectors:
        entry = content.get(selector, {})
        if not isinstance(entry, dict):
            continue
        values = entry.get("ros__parameters", {})
        if isinstance(values, dict):
            merged.update(values)
    return merged


def _setup(context, *_args, **_kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    meta_root = value("cr_meta_lnn_root")
    distal_checkpoint = value("v171_distal_checkpoint") or os.path.join(
        meta_root, "checkpoints",
        "real_distal_first_order_v171_multistep_map_em.pt")
    jacobian_json = value("jacobian_initialization_json") or os.path.join(
        meta_root, "evaluation", "real_joint_local_distal_v174.json")
    interface_transmission_checkpoint = value(
        "interface_transmission_checkpoint")
    distal_tendon_allocation_checkpoint = value(
        "distal_tendon_allocation_checkpoint")
    command_output_enabled = value(
        "command_output_enabled").lower() in ("1", "true", "yes")
    parameters = {
            "cr_meta_lnn_root": value("cr_meta_lnn_root"),
            "cr_common_root": value("cr_common_root"),
            "v171_distal_checkpoint": distal_checkpoint,
            "jacobian_initialization_json": jacobian_json,
            "interface_transmission_checkpoint": (
                interface_transmission_checkpoint),
            "distal_tendon_allocation_checkpoint": (
                distal_tendon_allocation_checkpoint),
            "adaptation_enabled": value(
                "adaptation_enabled").lower() in ("1", "true", "yes"),
            "adaptation_minimum_observations": int(value(
                "adaptation_minimum_observations")),
            "adaptation_minimum_normalized_action": float(value(
                "adaptation_minimum_normalized_action")),
            "adaptation_minimum_rotation_deg": float(value(
                "adaptation_minimum_rotation_deg")),
            "adaptation_minimum_translation_mm": float(value(
                "adaptation_minimum_translation_mm")),
            "adaptation_minimum_response_snr": float(value(
                "adaptation_minimum_response_snr")),
            "adaptation_maximum_window_s": float(value(
                "adaptation_maximum_window_s")),
            "adaptation_directional_purity": float(value(
                "adaptation_directional_purity")),
            "adaptation_reversal_holdoff_normalized_action": float(value(
                "adaptation_reversal_holdoff_normalized_action")),
            "adaptation_reversal_holdoff_normalized_action_shaft_0": float(
                value("adaptation_reversal_holdoff_normalized_action_shaft_0")),
            "adaptation_reversal_holdoff_normalized_action_shaft_1": float(
                value("adaptation_reversal_holdoff_normalized_action_shaft_1")),
            "adaptation_reversal_holdoff_normalized_action_shaft_2": float(
                value("adaptation_reversal_holdoff_normalized_action_shaft_2")),
            "adaptation_confirmation_windows": int(value(
                "adaptation_confirmation_windows")),
            "adaptation_consistency_cosine": float(value(
                "adaptation_consistency_cosine")),
            "adaptation_minimum_column_gain": float(value(
                "adaptation_minimum_column_gain")),
            "adaptation_maximum_column_gain": float(value(
                "adaptation_maximum_column_gain")),
            "adaptation_maximum_direction_deviation_deg": float(value(
                "adaptation_maximum_direction_deviation_deg")),
            "limits_file": value("limits_file"),
            "catheter": value("catheter"),
            "device": value("device"),
            "marker_estimator": value("marker_estimator"),
            "estimator_filter_initial_covariance": float(
                value("estimator_filter_initial_covariance")),
            "estimator_filter_process_std_sqrt_s": float(
                value("estimator_filter_process_std_sqrt_s")),
            "estimator_initial_roll_hypotheses": int(
                value("estimator_initial_roll_hypotheses")),
            "frame_id": value("frame_id"),
            "command_output_enabled": command_output_enabled,
            "horizon_steps": int(value("horizon_steps")),
            "rollout_step_s": float(value("rollout_step_s")),
            "mppi_point_rollout_step_s": float(value(
                "mppi_point_rollout_step_s")),
            "mppi_point_rollout_coarse_steps": value(
                "mppi_point_rollout_coarse_steps").lower()
            in ("1", "true", "yes"),
            "mppi_path_rollout_coarse_steps": value(
                "mppi_path_rollout_coarse_steps").lower()
            in ("1", "true", "yes"),
            "mppi_point_prediction_tail_steps": int(value(
                "mppi_point_prediction_tail_steps")),
            "mppi_point_prediction_tail_step_s": float(value(
                "mppi_point_prediction_tail_step_s")),
            "samples": int(value("samples")),
            "backlash_compensation_enabled": value(
                "backlash_compensation_enabled").lower()
            in ("1", "true", "yes"),
            "backlash_width_rad": json.loads(value("backlash_width_rad")),
            "backlash_width_positive_rad": json.loads(value(
                "backlash_width_positive_rad")),
            "backlash_width_negative_rad": json.loads(value(
                "backlash_width_negative_rad")),
            "backlash_takeup_velocity": json.loads(value(
                "backlash_takeup_velocity")),
            "backlash_minimum_motor_increment_rad": float(value(
                "backlash_minimum_motor_increment_rad")),
            "backlash_minimum_transmitted_increment_rad": float(value(
                "backlash_minimum_transmitted_increment_rad")),
            "backlash_directional_purity": float(value(
                "backlash_directional_purity")),
            "backlash_response_direction_cosine": float(value(
                "backlash_response_direction_cosine")),
            "backlash_minimum_response_evidence": float(value(
                "backlash_minimum_response_evidence")),
            "backlash_minimum_distal_bending_increment": float(value(
                "backlash_minimum_distal_bending_increment")),
            "backlash_width_learning_rate": float(value(
                "backlash_width_learning_rate")),
            "backlash_engagement_confirmation_observations": int(value(
                "backlash_engagement_confirmation_observations")),
            "backlash_provisional_rejection_observations": int(value(
                "backlash_provisional_rejection_observations")),
            "mppi_transmission_aware_rollout": value(
                "mppi_transmission_aware_rollout").lower()
            in ("1", "true", "yes"),
            "takeup_transaction_enabled": value(
                "takeup_transaction_enabled").lower()
            in ("1", "true", "yes"),
            "takeup_confirmation_hold_timeout_s": float(value(
                "takeup_confirmation_hold_timeout_s")),
            "mppi_rotation_direction_latch": value(
                "mppi_rotation_direction_latch").lower()
            in ("1", "true", "yes"),
            "mppi_best_candidate_guard": value(
                "mppi_best_candidate_guard").lower()
            in ("1", "true", "yes"),
            "mppi_takeup_risk_weight": float(value(
                "mppi_takeup_risk_weight")),
            "mppi_takeup_confirmation_time_s": float(value(
                "mppi_takeup_confirmation_time_s")),
            "reversal_scheduler_enabled": value(
                "reversal_scheduler_enabled").lower()
            in ("1", "true", "yes"),
            "reversal_scheduler_required_plans": int(value(
                "reversal_scheduler_required_plans")),
            "reversal_scheduler_minimum_absolute_cost_improvement": float(
                value(
                    "reversal_scheduler_minimum_absolute_cost_improvement")),
            "reversal_scheduler_minimum_fractional_cost_improvement": float(
                value(
                    "reversal_scheduler_minimum_fractional_cost_improvement")),
            "reversal_scheduler_minimum_terminal_error_improvement_mm": float(
                value(
                    "reversal_scheduler_minimum_terminal_error_improvement_mm")),
            "reversal_scheduler_minimum_accepted_observations": int(value(
                "reversal_scheduler_minimum_accepted_observations")),
            "reversal_scheduler_cooldown_s": float(value(
                "reversal_scheduler_cooldown_s")),
            "mppi_seed": int(value("mppi_seed")),
            "planning_deadline_s": float(value("planning_deadline_s")),
            "plan_rate_hz": float(value("plan_rate_hz")),
            "command_rate_hz": float(value("command_rate_hz")),
            "tip_error_log_rate_hz": float(
                value("tip_error_log_rate_hz")),
            "encoder_update_rate_hz": float(
                value("encoder_update_rate_hz")),
            "marker_update_rate_hz": float(value("marker_update_rate_hz")),
            "torch_intraop_threads": int(value("torch_intraop_threads")),
            "torch_interop_threads": int(value("torch_interop_threads")),
            "initialization_observations": int(
                value("initialization_observations")),
            "initialization_consecutive_inliers": int(
                value("initialization_consecutive_inliers")),
            "maximum_marker_lag_s": float(value("maximum_marker_lag_s")),
            "feedback_pair_max_skew_s": float(
                value("feedback_pair_max_skew_s")),
            "maximum_planner_deadline_misses": int(
                value("maximum_planner_deadline_misses")),
        }
    controller_config = value("controller_config").strip()
    node_parameters = [parameters]
    if controller_config:
        node_parameters.append(_reject_profile_output_interlock(
            controller_config, "controller_config"))
    performance_config = value("performance_config").strip()
    if performance_config:
        if command_output_enabled:
            raise RuntimeError(
                "command_output_enabled=true cannot be combined with a "
                "performance_config shadow overlay")
        node_parameters.append(_reject_profile_output_interlock(
            performance_config, "performance_config"))
    # This direct ROS parameter rule is the hardware-output interlock. Launch
    # dictionaries become wildcard ``/**`` parameter files; an exact
    # ``catheter_mppi:`` entry in a profile outranks that wildcard regardless
    # of file order. A direct ``-p`` rule avoids that selector-precedence trap:
    # profiles cannot silently enable output, and their conservative false
    # cannot hide an explicit launch request. Performance overlays remain
    # shadow-only through the guard above.
    node_parameters.append(Parameter(
        "command_output_enabled", command_output_enabled, value_type=bool))
    effective_parameters = dict(parameters)
    if controller_config:
        effective_parameters.update(_profile_parameters(controller_config))
    if performance_config:
        effective_parameters.update(_profile_parameters(performance_config))
    effective_parameters["command_output_enabled"] = command_output_enabled
    effective_horizon_steps = int(effective_parameters["horizon_steps"])
    effective_rollout_step_s = float(
        effective_parameters["rollout_step_s"])
    # Size the continuous-path preview from the effective controller profile,
    # including YAML overrides.  The extra margin covers callback phasing
    # while ReferenceHorizon continues to reject actual extrapolation.
    path_preview_duration_s = max(
        0.60,
        effective_horizon_steps*effective_rollout_step_s+0.20)
    effective_distal_checkpoint = str(effective_parameters.get(
        "v171_distal_checkpoint", distal_checkpoint))
    effective_jacobian_json = str(effective_parameters.get(
        "jacobian_initialization_json", jacobian_json))
    effective_interface_transmission_checkpoint = str(
        effective_parameters.get(
            "interface_transmission_checkpoint",
            interface_transmission_checkpoint))
    effective_distal_tendon_allocation_checkpoint = str(
        effective_parameters.get(
            "distal_tendon_allocation_checkpoint",
            distal_tendon_allocation_checkpoint))
    node = Node(
        package="catheter_control",
        executable="catheter_mppi",
        name="catheter_mppi",
        output="screen",
        parameters=node_parameters)
    trajectory_node = Node(
        package="catheter_control",
        executable="catheter_tip_trajectory",
        name="catheter_tip_trajectory",
        output="screen",
        parameters=[{"frame_id": value("frame_id")}])
    path_node = Node(
        package="catheter_control",
        executable="catheter_tip_path",
        name="catheter_tip_path",
        output="screen",
        parameters=[{
            "frame_id": value("frame_id"),
            "preview_duration_s": path_preview_duration_s,
        }])
    actions = [node, trajectory_node, path_node]
    if value("record").lower() in ("1", "true", "yes"):
        root = os.path.abspath(os.path.expanduser(value("session_root")))
        os.makedirs(root, exist_ok=True)
        output = os.path.join(
            root, datetime.now().strftime("%Y%m%d_%H%M%S_mppi_demo"))
        manifest = {
            "schema_version": 1,
            "bag_output": output,
            "record_topics": RECORD_TOPICS,
            "controller_parameters": effective_parameters,
            "controller_launch_defaults": parameters,
            "controller_config": (
                _artifact_manifest(controller_config)
                if controller_config else None),
            "performance_config": (
                _artifact_manifest(performance_config)
                if performance_config else None),
            "artifacts": {
                "v171_distal_checkpoint": _artifact_manifest(
                    effective_distal_checkpoint),
                "jacobian_initialization_json": _artifact_manifest(
                    effective_jacobian_json),
                "interface_transmission_checkpoint": (
                    _artifact_manifest(
                        effective_interface_transmission_checkpoint)
                    if effective_interface_transmission_checkpoint else None),
                "distal_tendon_allocation_checkpoint": (
                    _artifact_manifest(
                        effective_distal_tendon_allocation_checkpoint)
                    if effective_distal_tendon_allocation_checkpoint else None),
            },
        }
        with open(output + "_manifest.json", "x", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "--include-hidden-topics",
                 "-o", output, *RECORD_TOPICS],
            output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "cr_meta_lnn_root",
            default_value=os.environ.get(
                "CR_META_LNN_ROOT", "/home/chen-lab/Yifan/cr_meta_lnn")),
        DeclareLaunchArgument(
            "cr_common_root",
            default_value=os.environ.get(
                "CR_COMMON_ROOT", "/home/chen-lab/Yifan/cr-common")),
        DeclareLaunchArgument("v171_distal_checkpoint", default_value=""),
        DeclareLaunchArgument(
            "jacobian_initialization_json", default_value=""),
        DeclareLaunchArgument(
            "interface_transmission_checkpoint", default_value="",
            description=(
                "Optional v175 symmetric motor-to-interface transmission "
                "artifact. It replaces configured backlash widths and the "
                "v174 interface Jacobian, while retaining v171 distal "
                "tendon history.")),
        DeclareLaunchArgument(
            "distal_tendon_allocation_checkpoint", default_value="",
            description=(
                "Optional insertion-conditioned allocation fitted around "
                "frozen v171 distal tendon history.")),
        DeclareLaunchArgument(
            "controller_config", default_value="",
            description=(
                "Optional ROS parameter YAML applied after launch defaults; "
                "use only a reviewed profile.")),
        DeclareLaunchArgument(
            "performance_config", default_value="",
            description=(
                "Optional compute-only ROS parameter YAML applied after the "
                "controller profile. It must not enable hardware output.")),
        DeclareLaunchArgument(
            "adaptation_enabled", default_value="false",
            description=(
                "Enable accepted-observation RLS updates. Keep false for the "
                "first shadow-mode causal replay.")),
        DeclareLaunchArgument(
            "adaptation_minimum_observations", default_value="4"),
        DeclareLaunchArgument(
            "adaptation_minimum_normalized_action", default_value="0.01"),
        DeclareLaunchArgument(
            "adaptation_minimum_rotation_deg", default_value="0.10"),
        DeclareLaunchArgument(
            "adaptation_minimum_translation_mm", default_value="0.30"),
        DeclareLaunchArgument(
            "adaptation_minimum_response_snr", default_value="3.0"),
        DeclareLaunchArgument(
            "adaptation_maximum_window_s", default_value="1.0"),
        DeclareLaunchArgument(
            "adaptation_directional_purity", default_value="0.80"),
        DeclareLaunchArgument(
            "adaptation_reversal_holdoff_normalized_action",
            default_value="0.02"),
        DeclareLaunchArgument(
            "adaptation_reversal_holdoff_normalized_action_shaft_0",
            default_value="-1.0"),
        DeclareLaunchArgument(
            "adaptation_reversal_holdoff_normalized_action_shaft_1",
            default_value="-1.0"),
        DeclareLaunchArgument(
            "adaptation_reversal_holdoff_normalized_action_shaft_2",
            default_value="-1.0"),
        DeclareLaunchArgument(
            "adaptation_confirmation_windows", default_value="2"),
        DeclareLaunchArgument(
            "adaptation_consistency_cosine", default_value="0.50"),
        DeclareLaunchArgument(
            "adaptation_minimum_column_gain", default_value="0.25"),
        DeclareLaunchArgument(
            "adaptation_maximum_column_gain", default_value="4.0"),
        DeclareLaunchArgument(
            "adaptation_maximum_direction_deviation_deg",
            default_value="60.0"),
        DeclareLaunchArgument("limits_file", default_value=_default_limits()),
        DeclareLaunchArgument("catheter", default_value="imricor_test"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument(
            "marker_estimator", default_value="gauss_newton",
            description=(
                "Marker correction backend: gauss_newton, ekf, or ukf. "
                "The established gauss_newton path remains the default.")),
        DeclareLaunchArgument(
            "estimator_filter_initial_covariance", default_value="0.25"),
        DeclareLaunchArgument(
            "estimator_filter_process_std_sqrt_s", default_value="1.0"),
        DeclareLaunchArgument(
            "estimator_initial_roll_hypotheses", default_value="24"),
        DeclareLaunchArgument("frame_id", default_value="robot_base"),
        DeclareLaunchArgument(
            "command_output_enabled", default_value="false",
            description=(
                "Explicit hardware-output interlock. Keep false for Phase-5 "
                "dry runs.")),
        DeclareLaunchArgument("horizon_steps", default_value="4"),
        DeclareLaunchArgument("rollout_step_s", default_value="0.04"),
        DeclareLaunchArgument(
            "mppi_point_rollout_step_s", default_value="0.0"),
        DeclareLaunchArgument(
            "mppi_point_rollout_coarse_steps", default_value="false"),
        DeclareLaunchArgument(
            "mppi_path_rollout_coarse_steps", default_value="false"),
        DeclareLaunchArgument(
            "mppi_point_prediction_tail_steps", default_value="0"),
        DeclareLaunchArgument(
            "mppi_point_prediction_tail_step_s", default_value="0.12"),
        DeclareLaunchArgument("samples", default_value="32"),
        DeclareLaunchArgument(
            "backlash_compensation_enabled", default_value="false",
            description=(
                "Enable persistent physical-shaft backlash state estimation "
                "and bounded feedforward take-up.")),
        DeclareLaunchArgument(
            "backlash_width_rad", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "backlash_width_positive_rad",
            default_value="[0.0, 0.0, 0.0]",
            description=(
                "Physical-shaft take-up after entering positive motor "
                "direction; supplying either directional vector supersedes "
                "the legacy symmetric width.")),
        DeclareLaunchArgument(
            "backlash_width_negative_rad",
            default_value="[0.0, 0.0, 0.0]",
            description=(
                "Physical-shaft take-up after entering negative motor "
                "direction.")),
        DeclareLaunchArgument(
            "backlash_takeup_velocity", default_value="[8.0, 40.0, 4.5]"),
        DeclareLaunchArgument(
            "backlash_minimum_motor_increment_rad", default_value="0.01"),
        DeclareLaunchArgument(
            "backlash_minimum_transmitted_increment_rad",
            default_value="0.10"),
        DeclareLaunchArgument(
            "backlash_directional_purity", default_value="0.90"),
        DeclareLaunchArgument(
            "backlash_response_direction_cosine", default_value="0.50"),
        DeclareLaunchArgument(
            "backlash_minimum_response_evidence", default_value="0.50"),
        DeclareLaunchArgument(
            "backlash_minimum_distal_bending_increment",
            default_value="0.05",
            description=(
                "Minimum marker-corrected v171 bending-mode increment used "
                "as primary tendon engagement evidence.")),
        DeclareLaunchArgument(
            "backlash_width_learning_rate", default_value="0.05"),
        DeclareLaunchArgument(
            "backlash_engagement_confirmation_observations",
            default_value="1"),
        DeclareLaunchArgument(
            "backlash_provisional_rejection_observations",
            default_value="2"),
        DeclareLaunchArgument(
            "mppi_transmission_aware_rollout", default_value="false"),
        DeclareLaunchArgument(
            "takeup_transaction_enabled", default_value="false"),
        DeclareLaunchArgument(
            "takeup_confirmation_hold_timeout_s", default_value="1.0"),
        DeclareLaunchArgument(
            "mppi_rotation_direction_latch", default_value="false"),
        DeclareLaunchArgument(
            "mppi_best_candidate_guard", default_value="true"),
        DeclareLaunchArgument(
            "mppi_takeup_risk_weight", default_value="4.0"),
        DeclareLaunchArgument(
            "mppi_takeup_confirmation_time_s", default_value="0.10"),
        DeclareLaunchArgument(
            "reversal_scheduler_enabled", default_value="true"),
        DeclareLaunchArgument(
            "reversal_scheduler_required_plans", default_value="3"),
        DeclareLaunchArgument(
            "reversal_scheduler_minimum_absolute_cost_improvement",
            default_value="5.0"),
        DeclareLaunchArgument(
            "reversal_scheduler_minimum_fractional_cost_improvement",
            default_value="0.0"),
        DeclareLaunchArgument(
            "reversal_scheduler_minimum_terminal_error_improvement_mm",
            default_value="0.25"),
        DeclareLaunchArgument(
            "reversal_scheduler_minimum_accepted_observations",
            default_value="3"),
        DeclareLaunchArgument(
            "reversal_scheduler_cooldown_s", default_value="1.0"),
        DeclareLaunchArgument("mppi_seed", default_value="0"),
        DeclareLaunchArgument("planning_deadline_s", default_value="0.06"),
        DeclareLaunchArgument("plan_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
        DeclareLaunchArgument(
            "tip_error_log_rate_hz", default_value="1.0",
            description=(
                "Rate for ACTIVE target-minus-observed tip-error logs; "
                "zero disables terminal logging.")),
        DeclareLaunchArgument("encoder_update_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("marker_update_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("torch_intraop_threads", default_value="2"),
        DeclareLaunchArgument("torch_interop_threads", default_value="1"),
        DeclareLaunchArgument("initialization_observations", default_value="8"),
        DeclareLaunchArgument(
            "initialization_consecutive_inliers", default_value="2"),
        DeclareLaunchArgument("maximum_marker_lag_s", default_value="0.15"),
        DeclareLaunchArgument(
            "feedback_pair_max_skew_s", default_value="0.15"),
        DeclareLaunchArgument(
            "maximum_planner_deadline_misses", default_value="3"),
        DeclareLaunchArgument(
            "record", default_value="true",
            description=(
                "Record the complete hardware control path and response trace. "
                "Set false only for a deliberate non-recorded run.")),
        DeclareLaunchArgument(
            "session_root",
            default_value=(
                "/media/chen-lab/84BABCB7BABCA6D81/Yifan/"
                "catheter_sessions")),
        OpaqueFunction(function=_setup),
    ])
