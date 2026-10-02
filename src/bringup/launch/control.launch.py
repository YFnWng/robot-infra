"""Launch the guarded MPPI node and optional external-drive rosbag."""
from datetime import datetime
from importlib import metadata
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

from catheter_control.orchestration.configuration import (
    ACTIVE_CONTROLLER_PARAMETERS, load_ros_parameters, locate_stack,
    resolve_stack)
from catheter_control.orchestration.runtime import resolve_model_selection
from bringup.recording import RECORD_TOPICS


def _default_model_manifest():
    venv = Path(os.environ.get(
        "CR_VENV", "/home/chen-lab/Yifan/cr-venv")).expanduser()
    return str(
        venv / "lib/python3.10/site-packages/cr_meta_lnn/artifacts/manifests"
        / "20260929_175554_grouped_no_rotation_v2.json")


def _installed_package_identity():
    names = ("catheter-control", "cr-meta-lnn", "cr-common")
    result = {}
    venv_site = (Path(os.environ.get(
        "CR_VENV", "/home/chen-lab/Yifan/cr-venv"))
        / "lib/python3.10/site-packages")
    distributions = {
        str(item.metadata.get("Name", "")).lower(): item.version
        for item in metadata.distributions(
            path=[str(venv_site)] if venv_site.is_dir() else None)
    }
    for name in names:
        result[name] = distributions.get(name.lower())
        if result[name] is None:
            try:
                result[name] = metadata.version(name)
            except metadata.PackageNotFoundError:
                pass
    return result


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


def _model_manifest_identity(path):
    result = _artifact_manifest(path)
    resolved = Path(result["path"])
    if resolved.is_file():
        raw = json.loads(resolved.read_text(encoding="utf-8"))
        result.update({
            "schema_version": raw.get("schema_version"),
            "bundle_name": raw.get("bundle_name"),
            "deployment_api_version": raw.get("deployment_api_version"),
            "runtime_family": raw.get("runtime_family"),
            "artifacts": [
                {key: item.get(key) for key in ("id", "bytes", "sha256")}
                for item in raw.get("artifacts", [])
                if isinstance(item, dict)
            ],
        })
    return result


def _default_limits():
    try:
        return os.path.join(
            get_package_share_directory("control_interface"),
            "config", "catheter_limits.yaml")
    except Exception:
        candidate = (Path(__file__).resolve().parents[2]
                     / "control_interface" / "config" / "catheter_limits.yaml")
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
    """Resolve wildcard and exact ROS parameter selectors."""
    return load_ros_parameters(path, node_name)


def _setup(context, *_args, **_kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    selection = resolve_model_selection(
        value("model_manifest"), legacy={
            "cr_meta_lnn_root": value("cr_meta_lnn_root"),
            "cr_common_root": value("cr_common_root"),
            "v171_distal_checkpoint": value("v171_distal_checkpoint"),
            "jacobian_initialization_json": value(
                "jacobian_initialization_json"),
            "interface_transmission_checkpoint": value(
                "interface_transmission_checkpoint"),
            "distal_tendon_allocation_checkpoint": value(
                "distal_tendon_allocation_checkpoint"),
        })
    command_output_enabled = value(
        "command_output_enabled").lower() in ("1", "true", "yes")
    start_cpp_shadow = value(
        "start_cpp_shadow").lower() in ("1", "true", "yes")
    start_shadow_worker = value(
        "start_shadow_worker").lower() in ("1", "true", "yes")
    if start_shadow_worker and not start_cpp_shadow:
        raise RuntimeError(
            "start_shadow_worker=true requires start_cpp_shadow=true")
    parameters = {
            "model_manifest": selection.manifest_path,
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
            "estimator_device": value("estimator_device"),
            "estimator_dtype": value("estimator_dtype"),
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
    stack_config = value("stack_config").strip()
    controller_config = value("controller_config").strip()
    performance_config = value("performance_config").strip()
    if stack_config and (controller_config or performance_config):
        raise RuntimeError(
            "stack_config cannot be combined with legacy controller_config "
            "or performance_config overlays")
    resolved_configuration = None
    node_parameters = [parameters]
    if stack_config:
        config_root = Path(
            get_package_share_directory("catheter_control")) / "config"
        stack_path = locate_stack(stack_config, config_root)
        resolved_configuration = resolve_stack(
            stack_path, config_root=config_root,
            allowed_parameters=set(ACTIVE_CONTROLLER_PARAMETERS),
            expected_node_name="catheter_mppi")
        node_parameters.append(dict(resolved_configuration.parameters))
    if controller_config:
        node_parameters.append(_reject_profile_output_interlock(
            controller_config, "controller_config"))
    if performance_config:
        if command_output_enabled:
            raise RuntimeError(
                "command_output_enabled=true cannot be combined with a "
                "legacy performance_config shadow overlay")
        node_parameters.append(_reject_profile_output_interlock(
            performance_config, "performance_config"))
    # This direct ROS parameter rule is the hardware-output interlock. No YAML
    # layer may control it, and the final direct rule cannot be outranked by an
    # exact node selector.
    node_parameters.append(Parameter(
        "command_output_enabled", command_output_enabled, value_type=bool))
    effective_parameters = dict(parameters)
    if resolved_configuration is not None:
        effective_parameters.update(resolved_configuration.parameters)
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
    effective_model_manifest = str(
        effective_parameters.get("model_manifest", selection.manifest_path))
    node = Node(
        package="catheter_control",
        executable="catheter_mppi",
        name="catheter_mppi",
        output="screen",
        parameters=node_parameters)
    trajectory_node = Node(
        package="control_tasks",
        executable="catheter_tip_trajectory",
        name="catheter_tip_trajectory",
        output="screen",
        parameters=[{"frame_id": value("frame_id")}])
    path_node = Node(
        package="control_tasks",
        executable="catheter_tip_path",
        name="catheter_tip_path",
        output="screen",
        parameters=[{
            "frame_id": value("frame_id"),
            "preview_duration_s": path_preview_duration_s,
        }])
    actions = [node, trajectory_node, path_node]
    if start_cpp_shadow:
        identity = stack_config or controller_config or "launch_parameters"
        actions.append(Node(
            package="control_cpp",
            executable="control_shadow",
            name="control_shadow",
            output="screen",
            parameters=[{
                "request_rate_hz": float(value("plan_rate_hz")),
                "heartbeat_rate_hz": float(value("command_rate_hz")),
                "maximum_result_age_s": max(
                    0.20, 2.0*float(value("planning_deadline_s"))),
                "marker_topic": "/shape_tracking/markers",
                "controller_mode": (
                    "cpp_shadow_with_worker" if start_shadow_worker
                    else "cpp_shadow"),
                "configuration_identity": identity,
            }]))
    if start_shadow_worker:
        actions.append(Node(
            package="catheter_control",
            executable="control_shadow_worker",
            name="control_shadow_worker",
            output="screen",
            parameters=[{
                "maximum_input_age_s": max(
                    0.20, 2.0*float(value("planning_deadline_s"))),
            }]))
    recording = value("record").lower() in ("1", "true", "yes")
    manifest_path = value("recording_manifest_path")
    if recording or manifest_path:
        if recording:
            root = os.path.abspath(os.path.expanduser(value("session_root")))
            os.makedirs(root, exist_ok=True)
            output = os.path.join(
                root, datetime.now().strftime("%Y%m%d_%H%M%S_mppi_demo"))
        else:
            output = str(Path(manifest_path).expanduser().resolve().parent / "robot_bag")
        manifest = {
            "schema_version": 1,
            "bag_output": output,
            "record_topics": RECORD_TOPICS,
            "controller_parameters": effective_parameters,
            "controller_launch_defaults": parameters,
            "resolved_configuration": (
                resolved_configuration.as_manifest()
                if resolved_configuration is not None else None),
            "controller_config": (
                _artifact_manifest(controller_config)
                if controller_config else None),
            "performance_config": (
                _artifact_manifest(performance_config)
                if performance_config else None),
            "model_manifest": _model_manifest_identity(
                effective_model_manifest),
            "model_selection_compatibility_alias": (
                selection.compatibility_alias),
            "installed_packages": _installed_package_identity(),
        }
        with open(manifest_path or output + "_manifest.json", "x", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
        if recording:
            actions.append(ExecuteProcess(
                cmd=["ros2", "bag", "record", "--include-hidden-topics",
                     "-o", output, *RECORD_TOPICS],
                output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_manifest",
            default_value=os.environ.get(
                "CATHETER_MODEL_MANIFEST", _default_model_manifest()),
            description="Qualified schema-v2 deployment manifest."),
        DeclareLaunchArgument(
            "cr_meta_lnn_root", default_value="",
            description="Deprecated manifest-selection compatibility alias."),
        DeclareLaunchArgument(
            "cr_common_root", default_value="",
            description="Deprecated compatibility input; import paths are immutable."),
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
            "stack_config", default_value="",
            description=(
                "Semantic stack name under config/stacks or an explicit "
                "stack YAML path. Cannot be combined with legacy overlays.")),
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
        DeclareLaunchArgument("estimator_dtype", default_value="",
                             description="Estimator precision: float32 (default) or opt-in float64"),
        DeclareLaunchArgument("estimator_device", default_value="",
                             description="Estimator device; empty preserves the planner device."),
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
            "start_cpp_shadow", default_value="false",
            description=(
                "Start the non-commanding C++ Phase 4 shadow shell.")),
        DeclareLaunchArgument(
            "start_shadow_worker", default_value="false",
            description=(
                "Mirror fresh Python reference plans into the C++ shadow "
                "contract; requires start_cpp_shadow=true.")),
        DeclareLaunchArgument(
            "record", default_value="true",
            description=(
                "Record the complete hardware control path and response trace. "
                "Set false only for a deliberate non-recorded run.")),
        DeclareLaunchArgument(
            "recording_manifest_path", default_value="",
            description="Explicit identity-manifest path for coordinated recording."),
        DeclareLaunchArgument(
            "session_root",
            default_value=(
                "/media/chen-lab/84BABCB7BABCA6D81/Yifan/"
                "catheter_sessions")),
        OpaqueFunction(function=_setup),
    ])
