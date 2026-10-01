"""Launch the isolated exact-model catheter MPPI simulation."""
from datetime import datetime
import fcntl
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
    TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


SIM_RECORD_TOPICS = [
    "/sim/teleop/control",
    "/sim/teleop/event",
    "/sim/manager/control",
    "/sim/manager/event",
    "/sim/manager/safety_status",
    "/sim/device/state",
    "/sim/device/event",
    "/sim/shape_tracking/markers",
    "/sim/shape_tracking/marker_status",
    "/sim/catheter_mppi/target_tip",
    "/sim/catheter_mppi/reference_horizon",
    "/sim/catheter_mppi/reference_path",
    "/sim/catheter_mppi/path_reference_point",
    "/sim/catheter_mppi/path_tracking_trace",
    "/sim/catheter_mppi/planned_control",
    "/sim/catheter_mppi/predicted_tip",
    "/sim/catheter_mppi/response_trace",
    "/sim/catheter_mppi/estimator_trace",
    "/sim/catheter_mppi/control_cycle_timing",
    "/sim/collection/events",
    "/sim/collection/causal_trace",
    "/sim/catheter_mppi/status",
    "/sim/catheter_mppi/track_tip_trajectory/_action/feedback",
    "/sim/catheter_mppi/track_tip_trajectory/_action/status",
    "/sim/catheter_mppi/track_tip_path/_action/feedback",
    "/sim/catheter_mppi/track_tip_path/_action/status",
    "/sim/catheter_sim/ground_truth_tip",
    "/sim/catheter_sim/ground_truth_markers",
    "/sim/catheter_sim/ground_truth_interface_pose",
    "/sim/catheter_sim/joint_states",
    "/sim/catheter_sim/projected_control",
    "/sim/catheter_sim/realized_control",
    "/sim/catheter_sim/tip_error_mm",
    "/sim/catheter_sim/device_status",
    "/sim/catheter_sim/transmitted_state",
    "/sim/catheter_sim/visualization",
    "/tf_static",
]


_DOMAIN_LOCK = None


CONTROLLER_REMAPS = [
    ("/teleop/control", "/sim/teleop/control"),
    ("/teleop/event", "/sim/teleop/event"),
    ("/manager/safety_status", "/sim/manager/safety_status"),
    ("/manager/event", "/sim/manager/event"),
    ("/device/state", "/sim/device/state"),
    ("/shape_tracking/markers", "/sim/shape_tracking/markers"),
    ("/shape_tracking/marker_status", "/sim/shape_tracking/marker_status"),
    ("/catheter_mppi/target_tip", "/sim/catheter_mppi/target_tip"),
    ("/catheter_mppi/reference_horizon",
     "/sim/catheter_mppi/reference_horizon"),
    ("/catheter_mppi/planned_control", "/sim/catheter_mppi/planned_control"),
    ("/catheter_mppi/predicted_tip", "/sim/catheter_mppi/predicted_tip"),
    ("/catheter_mppi/response_trace", "/sim/catheter_mppi/response_trace"),
    ("/catheter_mppi/estimator_trace", "/sim/catheter_mppi/estimator_trace"),
    ("/catheter_mppi/control_cycle_timing",
     "/sim/catheter_mppi/control_cycle_timing"),
    ("/catheter_mppi/status", "/sim/catheter_mppi/status"),
    ("/catheter_mppi/set_armed", "/sim/catheter_mppi/set_armed"),
    ("/catheter_mppi/emergency_stop", "/sim/catheter_mppi/emergency_stop"),
    ("/catheter_mppi/generate_sparse_targets",
     "/sim/catheter_mppi/generate_sparse_targets"),
    ("/catheter_mppi/reset_simulation_state",
     "/sim/catheter_mppi/reset_simulation_state"),
    ("/collection/events", "/sim/collection/events"),
]


TRAJECTORY_REMAPS = [
    ("/shape_tracking/markers", "/sim/shape_tracking/markers"),
    ("/catheter_mppi/status", "/sim/catheter_mppi/status"),
    ("/catheter_mppi/target_tip", "/sim/catheter_mppi/target_tip"),
    ("/catheter_mppi/set_armed", "/sim/catheter_mppi/set_armed"),
    ("/catheter_mppi/track_tip_trajectory",
     "/sim/catheter_mppi/track_tip_trajectory"),
]


PATH_REMAPS = [
    ("/shape_tracking/markers", "/sim/shape_tracking/markers"),
    ("/catheter_mppi/status", "/sim/catheter_mppi/status"),
    ("/catheter_mppi/reference_horizon",
     "/sim/catheter_mppi/reference_horizon"),
    ("/catheter_mppi/reference_path",
     "/sim/catheter_mppi/reference_path"),
    ("/catheter_mppi/path_reference_point",
     "/sim/catheter_mppi/path_reference_point"),
    ("/catheter_mppi/path_tracking_trace",
     "/sim/catheter_mppi/path_tracking_trace"),
    ("/catheter_mppi/set_armed", "/sim/catheter_mppi/set_armed"),
    ("/catheter_mppi/track_tip_path",
     "/sim/catheter_mppi/track_tip_path"),
]


MANAGER_REMAPS = [
    ("/teleop/control", "/sim/teleop/control"),
    ("/teleop/event", "/sim/teleop/event"),
    ("/manager/control", "/sim/manager/control"),
    ("/manager/state", "/sim/manager/state"),
    ("/manager/event", "/sim/manager/event"),
    ("/manager/safety_status", "/sim/manager/safety_status"),
    ("/manager/qualify_driver_power",
     "/sim/manager/qualify_driver_power"),
    ("/device/state", "/sim/device/state"),
    ("/device/event", "/sim/device/event"),
    ("/device/transport_status", "/sim/device/transport_status"),
    ("/device/command", "/sim/device/command"),
]


def _default_limits():
    try:
        return os.path.join(
            get_package_share_directory("automation"),
            "config", "catheter_limits.yaml")
    except Exception:
        candidate = (Path(__file__).resolve().parents[2]
                     / "automation" / "config" / "catheter_limits.yaml")
        return str(candidate) if candidate.is_file() else ""


def _setup(context, *_args, **_kwargs):
    global _DOMAIN_LOCK

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def float_list(name, length):
        parsed = yaml.safe_load(value(name))
        if (not isinstance(parsed, list) or len(parsed) != length
                or any(not isinstance(item, (int, float)) for item in parsed)):
            raise ValueError(f"{name} must be a list of {length} numbers")
        return [float(item) for item in parsed]

    domain_id = os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0"
    if (value("require_nondefault_domain").lower() in ("1", "true", "yes")
            and domain_id == "0"):
        raise RuntimeError(
            "simulation requires a non-default ROS_DOMAIN_ID; for example, "
            "export ROS_DOMAIN_ID=42")
    lock_path = f"/tmp/catheter_mppi_sim_domain_{domain_id}.lock"
    lock_file = open(lock_path, "w", encoding="utf-8")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.close()
        raise RuntimeError(
            f"another catheter simulation is already running in ROS domain "
            f"{domain_id}") from exc
    _DOMAIN_LOCK = lock_file

    meta_root = value("cr_meta_lnn_root")
    distal = value("v171_distal_checkpoint") or os.path.join(
        meta_root, "checkpoints",
        "real_distal_first_order_v171_multistep_map_em.pt")
    jacobian = value("jacobian_initialization_json") or os.path.join(
        meta_root, "evaluation", "real_joint_local_distal_causal_v2.json")
    interface_transmission_checkpoint = value(
        "interface_transmission_checkpoint")
    distal_tendon_allocation_checkpoint = value(
        "distal_tendon_allocation_checkpoint")
    if interface_transmission_checkpoint:
        if value("backlash_compensation_enabled").lower() not in (
                "1", "true", "yes"):
            raise RuntimeError(
                "v175 simulation requires backlash_compensation_enabled")
        if value("takeup_transaction_enabled").lower() not in (
                "1", "true", "yes"):
            raise RuntimeError(
                "v175 simulation requires takeup_transaction_enabled")
        actuator_widths = (
            float_list("actuator_reversal_backlash_rad", 6)
            +float_list("actuator_reversal_backlash_positive_rad", 6)
            +float_list("actuator_reversal_backlash_negative_rad", 6))
        if any(width > 0.0 for width in (
                actuator_widths[:3]+actuator_widths[6:9]
                +actuator_widths[12:15])):
            raise RuntimeError(
                "v175 simulation truth owns proximal play; actuator "
                "reversal backlash for axes 0-2 must be zero")
    common_parameters = {
        "cr_meta_lnn_root": meta_root,
        "cr_common_root": value("cr_common_root"),
        "v171_distal_checkpoint": distal,
        "jacobian_initialization_json": jacobian,
        "interface_transmission_checkpoint": (
            interface_transmission_checkpoint),
        "distal_tendon_allocation_checkpoint": (
            distal_tendon_allocation_checkpoint),
        "limits_file": value("limits_file"),
        "catheter": value("catheter"),
        "device": value("device"),
        "frame_id": value("frame_id"),
    }
    plan_rate_hz = float(value("plan_rate_hz"))
    if plan_rate_hz <= 0.0:
        raise ValueError("plan_rate_hz must be positive")
    horizon_steps = int(value("horizon_steps"))
    rollout_step_s = float(value("rollout_step_s"))
    # The path server must publish every timestamp the planner can query.
    # Keep its historical samples for estimator-time interpolation, and add a
    # scheduling margin beyond the final rollout target rather than weakening
    # ReferenceHorizon's fail-closed no-extrapolation check.
    path_preview_duration_s = max(
        0.60, horizon_steps*rollout_step_s+0.20)
    command_timeout_text = value("command_timeout_s").strip()
    command_timeout_s = (
        float(command_timeout_text) if command_timeout_text
        else max(0.15, 1.5/plan_rate_hz))

    # Keep the A/B comparison definition inside the launch so both trials use
    # the same plant, estimator, random seeds, samples, and path. ``plain`` is
    # conventional ungrouped MPPI with no explicit reversal/take-up costs or
    # response-clocked reversal scheduler. The physical take-up compensator,
    # when requested, remains identical in both trials.
    variant = value("mppi_variant").strip().lower()
    if variant not in ("grouped", "plain", "custom"):
        raise ValueError("mppi_variant must be grouped, plain, or custom")
    if variant == "plain":
        grouped_mode_sampling = False
        best_candidate_guard = False
        takeup_risk_weight = 0.0
        takeup_confirmation_time_s = float(value(
            "mppi_takeup_confirmation_time_s"))
        reversal_scheduler_enabled = False
        visualizer_label = "PLAIN MPPI | no take-up transaction cost"
    else:
        grouped_mode_sampling = (
            variant == "grouped"
            or value("mppi_grouped_mode_sampling").lower()
            in ("1", "true", "yes"))
        best_candidate_guard = value(
            "mppi_best_candidate_guard").lower() in ("1", "true", "yes")
        takeup_risk_weight = float(value("mppi_takeup_risk_weight"))
        takeup_confirmation_time_s = float(value(
            "mppi_takeup_confirmation_time_s"))
        reversal_scheduler_enabled = value(
            "reversal_scheduler_enabled").lower() in ("1", "true", "yes")
        visualizer_label = (
            "GROUPED MPPI" if variant == "grouped" else "CUSTOM MPPI")

    actions = [
        LogInfo(msg=(
            "=== SIMULATION ONLY: all control/device endpoints are explicitly "
            "remapped under /sim; no serial node will be launched ===")),
        Node(
            package="catheter_control", executable="catheter_sim_device",
            name="catheter_sim_device", output="screen",
            parameters=[{
                "simulation_only": True,
                "frame_id": value("frame_id"),
                "limits_file": value("limits_file"),
                "catheter": value("catheter"),
                "plant_rate_hz": float(value("plant_rate_hz")),
                "actuator_gain": float_list("actuator_gain", 6),
                "actuator_deadband_rad_s": float_list(
                    "actuator_deadband_rad_s", 6),
                "actuator_time_constant_s": float_list(
                    "actuator_time_constant_s", 6),
                "actuator_command_delay_s": float(value(
                    "actuator_command_delay_s")),
                "actuator_reversal_backlash_rad": float_list(
                    "actuator_reversal_backlash_rad", 6),
                "actuator_reversal_backlash_positive_rad": float_list(
                    "actuator_reversal_backlash_positive_rad", 6),
                "actuator_reversal_backlash_negative_rad": float_list(
                    "actuator_reversal_backlash_negative_rad", 6),
                "actuator_initial_backlash_unengaged": value(
                    "actuator_initial_backlash_unengaged").lower()
                in ("1", "true", "yes"),
            }]),
        Node(
            package="catheter_control",
            executable="catheter_sim_perception",
            name="catheter_sim_perception", output="screen",
            parameters=[{
                "simulation_only": True,
                "frame_id": value("frame_id"),
                "cr_meta_lnn_root": meta_root,
                "cr_common_root": value("cr_common_root"),
                "v171_distal_checkpoint": distal,
                "jacobian_initialization_json": jacobian,
                "interface_transmission_checkpoint": (
                    interface_transmission_checkpoint),
                "distal_tendon_allocation_checkpoint": (
                    distal_tendon_allocation_checkpoint),
                # Simulation truth is an independent workload, not part of
                # the controller under test. Keep it off the controller GPU
                # by default so it cannot serialize CUDA work and distort the
                # planner deadline measurement.
                "device": value("truth_model_device"),
                "marker_rate_hz": float(value("marker_rate_hz")),
                "robustness_seed": int(value("robustness_seed")),
                "marker_noise_std_mm": float(value(
                    "marker_noise_std_mm")),
                "marker_common_bias_mm": float_list(
                    "marker_common_bias_mm", 3),
                "marker_specific_bias_mm": float_list(
                    "marker_specific_bias_mm", 12),
                "marker_latency_ms": float(value("marker_latency_ms")),
                "marker_timestamp_jitter_ms": float(value(
                    "marker_timestamp_jitter_ms")),
                "marker_dropout_probability": float(value(
                    "marker_dropout_probability")),
                "marker_outlier_probability": float(value(
                    "marker_outlier_probability")),
                "marker_outlier_magnitude_mm": float(value(
                    "marker_outlier_magnitude_mm")),
                "truth_initial_interface_pose": float_list(
                    "truth_initial_interface_pose", 16),
                "plant_jacobian_angular_column_gain": float_list(
                    "plant_jacobian_angular_column_gain", 3),
                "plant_jacobian_linear_column_gain": float_list(
                    "plant_jacobian_linear_column_gain", 3),
                "torch_intraop_threads": 1,
                "torch_interop_threads": 1,
            }]),
        Node(
            package="control_interface", executable="manager.py",
            name="sim_manager", output="screen",
            remappings=MANAGER_REMAPS,
            parameters=[{
                "limits_file": value("limits_file"),
                "catheter": value("catheter"),
                "feedback_timeout_s": 0.25,
                "command_max_age_s": 0.10,
            }]),
        Node(
            package="catheter_control", executable="catheter_mppi",
            name="sim_catheter_mppi", output="screen",
            remappings=CONTROLLER_REMAPS,
            parameters=[{
                **common_parameters,
                "command_output_enabled": True,
                "simulation_state_reset_enabled": True,
                "adaptation_enabled": False,
                "controller_velocity_max": float_list(
                    "controller_velocity_max", 6),
                "marker_estimator": value("marker_estimator"),
                "estimator_initial_roll_hypotheses": int(
                    value("estimator_initial_roll_hypotheses")),
                "horizon_steps": horizon_steps,
                "rollout_step_s": rollout_step_s,
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
                "mppi_noise_std": float_list("mppi_noise_std", 3),
                "mppi_noise_correlation": float(value(
                    "mppi_noise_correlation")),
                "mppi_exploration_fraction": float(value(
                    "mppi_exploration_fraction")),
                "mppi_reversal_backlash_rad": float_list(
                    "mppi_reversal_backlash_rad", 3),
                "backlash_compensation_enabled": value(
                    "backlash_compensation_enabled").lower()
                in ("1", "true", "yes"),
                "backlash_width_rad": float_list(
                    "backlash_width_rad", 3),
                "backlash_width_positive_rad": float_list(
                    "backlash_width_positive_rad", 3),
                "backlash_width_negative_rad": float_list(
                    "backlash_width_negative_rad", 3),
                "backlash_takeup_velocity": float_list(
                    "backlash_takeup_velocity", 3),
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
                "mppi_best_candidate_guard": best_candidate_guard,
                "mppi_grouped_mode_sampling": grouped_mode_sampling,
                "mppi_takeup_risk_weight": takeup_risk_weight,
                "mppi_takeup_confirmation_time_s":
                    takeup_confirmation_time_s,
                "reversal_scheduler_enabled": reversal_scheduler_enabled,
                "reversal_scheduler_required_plans": int(value(
                    "reversal_scheduler_required_plans")),
                "reversal_scheduler_minimum_absolute_cost_improvement": (
                    float(value(
                        "reversal_scheduler_minimum_absolute_cost_improvement"))),
                "reversal_scheduler_minimum_fractional_cost_improvement": (
                    float(value(
                        "reversal_scheduler_minimum_fractional_cost_improvement"))),
                "reversal_scheduler_minimum_terminal_error_improvement_mm": (
                    float(value(
                        "reversal_scheduler_minimum_terminal_error_improvement_mm"))),
                "reversal_scheduler_minimum_accepted_observations": int(
                    value("reversal_scheduler_minimum_accepted_observations")),
                "reversal_scheduler_cooldown_s": float(value(
                    "reversal_scheduler_cooldown_s")),
                "mppi_seed": int(value("mppi_seed")),
                "planning_deadline_s": float(value("planning_deadline_s")),
                "plan_rate_hz": plan_rate_hz,
                "command_timeout_s": command_timeout_s,
                "command_rate_hz": float(value("command_rate_hz")),
                "encoder_update_rate_hz": float(
                    value("encoder_update_rate_hz")),
                "marker_update_rate_hz": float(
                    value("marker_update_rate_hz")),
                "maximum_planner_deadline_misses": int(
                    value("maximum_planner_deadline_misses")),
                "feedback_timeout_s": float(
                    value("simulation_feedback_timeout_s")),
                "marker_timeout_s": float(
                    value("simulation_marker_timeout_s")),
                "torch_intraop_threads": int(
                    value("torch_intraop_threads")),
                "torch_interop_threads": int(
                    value("torch_interop_threads")),
            }]),
        Node(
            package="catheter_control",
            executable="catheter_tip_trajectory",
            name="sim_catheter_tip_trajectory",
            output="screen",
            remappings=TRAJECTORY_REMAPS,
            parameters=[{
                "frame_id": value("frame_id"),
                "marker_topic": "/sim/shape_tracking/markers",
                "status_topic": "/sim/catheter_mppi/status",
                "target_topic": "/sim/catheter_mppi/target_tip",
                "arm_service": "/sim/catheter_mppi/set_armed",
                "action_name": (
                    "/sim/catheter_mppi/track_tip_trajectory"),
            }]),
        Node(
            package="catheter_control",
            executable="catheter_tip_path",
            name="sim_catheter_tip_path",
            output="screen",
            remappings=PATH_REMAPS,
            parameters=[{
                "frame_id": value("frame_id"),
                "marker_topic": "/sim/shape_tracking/markers",
                "status_topic": "/sim/catheter_mppi/status",
                "reference_topic": (
                    "/sim/catheter_mppi/reference_horizon"),
                "path_topic": "/sim/catheter_mppi/reference_path",
                "reference_point_topic": (
                    "/sim/catheter_mppi/path_reference_point"),
                "arm_service": "/sim/catheter_mppi/set_armed",
                "action_name": "/sim/catheter_mppi/track_tip_path",
                "preview_duration_s": path_preview_duration_s,
            }]),
        Node(
            package="catheter_control",
            executable="catheter_sim_visualizer",
            name="catheter_sim_visualizer", output="screen",
            parameters=[{
                "frame_id": value("frame_id"),
                "controller_label": visualizer_label,
            }]),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="catheter_sim_frame", output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", value("frame_id")]),
    ]

    if value("auto_qualify").lower() in ("1", "true", "yes"):
        actions.append(TimerAction(
            period=float(value("auto_qualify_delay_s")),
            actions=[ExecuteProcess(
                cmd=["ros2", "service", "call",
                     "/sim/manager/qualify_driver_power",
                     "std_srvs/srv/Trigger", "{}"],
                output="screen")]))
    if value("record").lower() in ("1", "true", "yes"):
        root = os.path.abspath(os.path.expanduser(value("session_root")))
        os.makedirs(root, exist_ok=True)
        output = os.path.join(
            root, datetime.now().strftime("%Y%m%d_%H%M%S_mppi_sim"))
        actions.append(LogInfo(msg=f"simulation bag -> {output}"))
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "--include-hidden-topics",
                 "-o", output,
                 *SIM_RECORD_TOPICS], output="screen"))
    return actions


def generate_launch_description():
    share = get_package_share_directory("catheter_control")
    rviz_config = os.path.join(share, "config", "mppi_sim.rviz")
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
        DeclareLaunchArgument("jacobian_initialization_json", default_value=""),
        DeclareLaunchArgument(
            "interface_transmission_checkpoint", default_value="",
            description=(
                "Optional v175 interface-transmission artifact used by both "
                "the controller model and independent simulation truth. "
                "Configure plant reversal widths consistently.")),
        DeclareLaunchArgument(
            "distal_tendon_allocation_checkpoint", default_value="",
            description=(
                "Optional insertion-conditioned distal tendon allocation "
                "used by both controller and independent simulation truth.")),
        DeclareLaunchArgument("limits_file", default_value=_default_limits()),
        DeclareLaunchArgument("catheter", default_value="imricor_test"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument(
            "truth_model_device", default_value="cpu",
            description=(
                "Torch device for independent simulated truth/perception; "
                "keep this on CPU while benchmarking controller CUDA.")),
        DeclareLaunchArgument("frame_id", default_value="robot_base"),
        DeclareLaunchArgument("marker_estimator", default_value="ukf"),
        DeclareLaunchArgument(
            "controller_velocity_max",
            default_value="[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]",
            description=(
                "Controller-local six-axis velocity limits; negative values "
                "inherit the hardware contract. Use an exact zero to disable "
                "an axis for isolation tests.")),
        DeclareLaunchArgument(
            "mppi_variant", default_value="grouped",
            description=(
                "Controller A/B profile: grouped enables grouped U/C "
                "sampling and configured reversal costs; plain uses ordinary "
                "sampling, weighted MPPI execution, and zero reversal/take-up "
                "costs; custom honors the individual MPPI switches.")),
        DeclareLaunchArgument(
            "estimator_initial_roll_hypotheses", default_value="24"),
        DeclareLaunchArgument("plant_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("marker_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("robustness_seed", default_value="0"),
        DeclareLaunchArgument("marker_noise_std_mm", default_value="0.0"),
        DeclareLaunchArgument(
            "marker_common_bias_mm", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "marker_specific_bias_mm",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                          "0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("marker_latency_ms", default_value="0.0"),
        DeclareLaunchArgument(
            "marker_timestamp_jitter_ms", default_value="0.0"),
        DeclareLaunchArgument(
            "marker_dropout_probability", default_value="0.0"),
        DeclareLaunchArgument(
            "marker_outlier_probability", default_value="0.0"),
        DeclareLaunchArgument(
            "marker_outlier_magnitude_mm", default_value="0.0"),
        DeclareLaunchArgument(
            "truth_initial_interface_pose",
            default_value=(
                "[-0.852797151, -0.473069161, 0.221229315, -0.000312328, "
                "0.495341808, -0.866912186, 0.055673875, 0.000244854, "
                "0.165448815, 0.157062665, 0.973631382, 0.009205841, "
                "0.0, 0.0, 0.0, 1.0]")),
        DeclareLaunchArgument(
            "actuator_gain",
            default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"),
        DeclareLaunchArgument(
            "actuator_deadband_rad_s",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "actuator_time_constant_s",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "actuator_command_delay_s", default_value="0.0"),
        DeclareLaunchArgument(
            "actuator_reversal_backlash_rad",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "actuator_reversal_backlash_positive_rad",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "actuator_reversal_backlash_negative_rad",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "actuator_initial_backlash_unengaged", default_value="false",
            description=(
                "Start each simulated shaft on an unknown, unengaged side "
                "of its configured directional gap.")),
        DeclareLaunchArgument(
            "plant_jacobian_angular_column_gain",
            default_value="[1.0, 1.0, 1.0]"),
        DeclareLaunchArgument(
            "plant_jacobian_linear_column_gain",
            default_value="[1.0, 1.0, 1.0]"),
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
            "mppi_noise_std", default_value="[4.0, 20.0, 2.0]"),
        DeclareLaunchArgument(
            "mppi_noise_correlation", default_value="0.65"),
        DeclareLaunchArgument(
            "mppi_exploration_fraction", default_value="0.15"),
        DeclareLaunchArgument(
            "mppi_reversal_backlash_rad",
            default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "backlash_compensation_enabled", default_value="false"),
        DeclareLaunchArgument(
            "backlash_width_rad", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "backlash_width_positive_rad",
            default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument(
            "backlash_width_negative_rad",
            default_value="[0.0, 0.0, 0.0]"),
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
            default_value="0.05"),
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
            "mppi_grouped_mode_sampling", default_value="true"),
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
        DeclareLaunchArgument(
            "command_timeout_s", default_value="",
            description=(
                "Controller plan freshness timeout. Empty derives max(0.15, "
                "1.5 / plan_rate_hz).")),
        DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("encoder_update_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("marker_update_rate_hz", default_value="20.0"),
        DeclareLaunchArgument(
            "maximum_planner_deadline_misses", default_value="5",
            description=(
                "Simulation-only consecutive zero-command recovery window. "
                "The hardware launch retains its stricter default of 3.")),
        DeclareLaunchArgument(
            "simulation_feedback_timeout_s", default_value="0.5"),
        DeclareLaunchArgument(
            "simulation_marker_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("torch_intraop_threads", default_value="2"),
        DeclareLaunchArgument("torch_interop_threads", default_value="1"),
        DeclareLaunchArgument("auto_qualify", default_value="true"),
        DeclareLaunchArgument("auto_qualify_delay_s", default_value="3.0"),
        DeclareLaunchArgument(
            "require_nondefault_domain", default_value="true"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument(
            "session_root", default_value=(
                "/media/chen-lab/84BABCB7BABCA6D81/Yifan/"
                "catheter_sessions")),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
        Node(
            package="rviz2", executable="rviz2", name="catheter_sim_rviz",
            arguments=["-d", rviz_config], output="screen",
            condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
