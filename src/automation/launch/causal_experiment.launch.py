"""Run and record the guarded proximal causal-identification experiment.

The hardware manager, marker tracker, and a *disarmed*, adaptation-disabled
catheter_mppi estimator must already be running.  Motion remains disabled by
default; set ``start_motor:=true`` only after inspecting the generated plan and
passing the manager's hardware qualification.
"""
from datetime import datetime
import json
import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
    RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess, matches_name
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SESSION_ROOT = (
    "/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions")

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
    "/collection/causal_trace",
    "/catheter_mppi/estimator_trace",
    "/catheter_mppi/control_cycle_timing",
    "/catheter_mppi/status",
    "/parameter_events",
    "/rosout",
]

SIM_REMAPS = [
    ("/teleop/control", "/sim/teleop/control"),
    ("/teleop/event", "/sim/teleop/event"),
    ("/device/state", "/sim/device/state"),
    ("/device/event", "/sim/device/event"),
    ("/device/command", "/sim/device/command"),
    ("/collection/events", "/sim/collection/events"),
    ("/collection/causal_trace", "/sim/collection/causal_trace"),
    ("/collection/abort", "/sim/collection/abort"),
    ("/catheter_mppi/status", "/sim/catheter_mppi/status"),
]

SIM_EXTRA_TOPICS = [
    "/sim/catheter_sim/ground_truth_tip",
    "/sim/catheter_sim/ground_truth_markers",
    "/sim/catheter_sim/joint_states",
    "/sim/catheter_sim/projected_control",
    "/sim/catheter_sim/realized_control",
    "/sim/catheter_sim/tip_error_mm",
    "/sim/catheter_sim/device_status",
]


def _sim_topics():
    keep_real = {"/parameter_events", "/rosout"}
    return ([topic if topic in keep_real else "/sim" + topic
             for topic in RECORD_TOPICS] + SIM_EXTRA_TOPICS)


def _floats(value):
    return [float(item) for item in str(value).split(",") if item]


def _setup(context, *_args, **_kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    use_sim = value("use_sim").lower() in ("1", "true", "yes")
    if value("storage_id") != "sqlite3":
        raise RuntimeError(
            "causal isolation sessions require finalized sqlite3 storage")
    parameters = {
        "mode": "causal",
        "causal_schedule": value("schedule"),
        "target": "catheter",
        "rate_hz": float(value("rate_hz")),
        "start_motor": value("start_motor").lower() in ("1", "true", "yes"),
        "limits_file": value("limits_file"),
        "catheter": value("catheter"),
        "causal_amplitudes": _floats(value("amplitudes")),
        "causal_minimum_amplitudes": _floats(
            value("minimum_amplitudes")),
        "causal_margins": _floats(value("margins")),
        "causal_slow_speeds": _floats(value("slow_speeds")),
        "causal_fast_speeds": _floats(value("fast_speeds")),
        "causal_repeats": int(value("repeats")),
        "causal_static_s": float(value("static_s")),
        "causal_endpoint_dwell_s": float(value("endpoint_dwell_s")),
        "causal_between_episode_s": float(value("between_episode_s")),
        "causal_rotation_relax_s": float(value("rotation_relax_s")),
        "causal_timing_leads_ms": _floats(value("timing_leads_ms")),
        "causal_timing_direction": int(value("timing_direction")),
        "causal_bend_bias_position": float(value("bend_bias_position")),
        "causal_insertion_center_position": float(
            value("insertion_center_position")),
        "causal_insertion_plateaus": _floats(
            value("insertion_plateaus")),
        "causal_insertion_plateau_visits": int(
            value("insertion_plateau_visits")),
        "causal_insertion_plateau_dwell_s": float(
            value("insertion_plateau_dwell_s")),
        "causal_tendon_sweep_limits": _floats(
            value("tendon_sweep_limits")),
        # A zero-initialized simulated plant may move to the same 20 mm
        # interior operating point used by hardware. Never silently perform
        # this setup motion on real hardware.
        "causal_allow_insertion_centering": use_sim,
        "causal_max_duration_s": float(value("maximum_duration_s")),
        "causal_require_estimator": True,
        "causal_require_estimator_tracking": (
            value("require_estimator_tracking").lower()
            in ("1", "true", "yes")),
        "causal_require_controller_disarmed": True,
        "causal_require_adaptation_disabled": True,
        # Every actuating isolation run establishes the reviewed operating
        # point before run_start. Non-actuating dry runs issue no such motion.
        "causal_initialize_before_run": (
            value("start_motor").lower() in ("1", "true", "yes")),
        "causal_initial_position": _floats(value("initial_position")),
        "causal_initialization_timeout_s": float(
            value("initialization_timeout_s")),
        "return_to_start": True,
        "return_control_mode": "position",
        "return_to_zero": False,
        "return_timeout_s": float(value("return_timeout_s")),
        "shutdown_on_done": True,
    }
    runner = Node(
        package="automation", executable="collection",
        name="causal_experiment", output="screen",
        parameters=[parameters],
        remappings=SIM_REMAPS if use_sim else [])

    root = os.path.abspath(os.path.expanduser(value("session_root")))
    os.makedirs(root, exist_ok=True)
    output = os.path.join(
        root, datetime.now().strftime("%Y%m%d_%H%M%S_causal_proximal"))
    manifest = {
        "schema_version": 2,
        "experiment": "causal_proximal_identification_v8",
        "schedule": parameters["causal_schedule"],
        "bag_output": os.path.join(output, "robot_bag"),
        "parameters": parameters,
        "record_topics": _sim_topics() if use_sim else RECORD_TOPICS,
        "use_sim": use_sim,
        "required_external_nodes": [
            "manager", "device_serial_com", "marker_tracking",
            "catheter_mppi_disarmed_adaptation_disabled"],
        "runtime_identity_file": os.path.join(output, "runtime_identity.json"),
        "completeness_report": os.path.join(output, "completeness.json"),
        "safety": {
            "start_motor": parameters["start_motor"],
            "initialization_enabled": parameters[
                "causal_initialize_before_run"],
            "initialization_target": parameters["causal_initial_position"],
            "initialization_uses_encoder_zero": False,
            "return_target": "measured_run_start",
            "zero_command_permitted": False,
        },
    }
    os.makedirs(output, exist_ok=False)
    with open(os.path.join(output, "manifest.json"), "w", encoding="utf-8") as file:
        json.dump(manifest, file, indent=2, sort_keys=True)
        file.write("\n")

    identity = ExecuteProcess(
        name="causal_runtime_identity",
        cmd=["ros2", "run", "automation", "causal_runtime_identity",
             "--output", os.path.join(output, "runtime_identity.json"),
             "--required-node",
             # Simulation remaps controller *topics* below /sim/catheter_mppi,
             # but the node itself is named /sim_catheter_mppi. Parameter
             # services follow node identity, not topic remappings.
             "/sim_catheter_mppi" if use_sim else "/catheter_mppi"],
        output="screen")
    record_enabled = value("record").lower() in ("1", "true", "yes")
    actions = [identity]
    if record_enabled:
        record_topics = _sim_topics() if use_sim else RECORD_TOPICS
        recorder = ExecuteProcess(
            name="causal_rosbag_record",
            cmd=["ros2", "bag", "record", "-o",
                 os.path.join(output, "robot_bag"), "--storage",
                 value("storage_id"), *record_topics],
            output="screen")
        checker = ExecuteProcess(
            name="causal_session_check",
            cmd=["ros2", "run", "automation", "causal_session_check",
                 output], output="screen")
        state = {"runner_finished": False}

        def identity_exit(event, _context):
            if event.returncode != 0:
                return EmitEvent(event=Shutdown(
                    reason="runtime identity collection failed"))
            return [recorder, runner]

        def runner_exit(_event, _context):
            state["runner_finished"] = True
            return EmitEvent(event=SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=matches_name("causal_rosbag_record")))

        def recorder_exit(_event, _context):
            if not state["runner_finished"]:
                return EmitEvent(event=Shutdown(
                    reason="causal rosbag recorder exited early"))
            return checker

        actions.extend([
            RegisterEventHandler(OnProcessExit(
                target_action=identity, on_exit=identity_exit)),
            RegisterEventHandler(OnProcessExit(
                target_action=runner, on_exit=runner_exit)),
            RegisterEventHandler(OnProcessExit(
                target_action=recorder, on_exit=recorder_exit)),
            RegisterEventHandler(OnProcessExit(
                target_action=checker,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="causal session completeness check finished"))])),
        ])
    else:
        def identity_exit_no_record(event, _context):
            if event.returncode != 0:
                return EmitEvent(event=Shutdown(
                    reason="runtime identity collection failed"))
            return runner

        actions.extend([
            RegisterEventHandler(OnProcessExit(
                target_action=identity, on_exit=identity_exit_no_record)),
            RegisterEventHandler(OnProcessExit(
                target_action=runner,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="causal experiment complete"))])),
        ])
    print(f"[causal_experiment.launch] session -> {output}")
    return actions


def generate_launch_description():
    limits = os.path.join(
        get_package_share_directory("automation"),
        "config", "catheter_limits.yaml")
    arguments = [
        DeclareLaunchArgument("session_root", default_value=SESSION_ROOT),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("storage_id", default_value="sqlite3"),
        DeclareLaunchArgument("start_motor", default_value="false"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("limits_file", default_value=limits),
        DeclareLaunchArgument("catheter", default_value="imricor_test"),
        DeclareLaunchArgument("rate_hz", default_value="100.0"),
        DeclareLaunchArgument(
            "schedule", default_value="full",
            description=(
                "full, phase_0_2, stationary, insertion, tendon_motor, "
                "compensated_bend, timing/phase_3, or "
                "chassis_knob_backdrive/backdrive, or "
                "insertion_rotation/phase_2d, or "
                "compensated_bend_insertion_sweep/phase_2e")),
        DeclareLaunchArgument("amplitudes", default_value="6.0,75.0,5.5"),
        DeclareLaunchArgument(
            "minimum_amplitudes", default_value="5.0,65.0,4.75"),
        DeclareLaunchArgument("margins", default_value="15.0,20.0,1.0"),
        DeclareLaunchArgument("slow_speeds", default_value="2.0,7.0,2.0"),
        DeclareLaunchArgument("fast_speeds", default_value="5.0,20.0,4.0"),
        DeclareLaunchArgument("repeats", default_value="3"),
        DeclareLaunchArgument("static_s", default_value="15.0"),
        DeclareLaunchArgument("endpoint_dwell_s", default_value="2.0"),
        DeclareLaunchArgument("between_episode_s", default_value="1.0"),
        DeclareLaunchArgument("rotation_relax_s", default_value="4.0"),
        DeclareLaunchArgument(
            "timing_leads_ms", default_value="20.0,40.0,80.0"),
        DeclareLaunchArgument(
            "timing_direction", default_value="1",
            description="Phase-3 measured direction: +1 or -1"),
        DeclareLaunchArgument("bend_bias_position", default_value="7.5"),
        DeclareLaunchArgument(
            "insertion_center_position", default_value="20.0"),
        DeclareLaunchArgument(
            "insertion_plateaus",
            default_value="0.0,13.3333333333,26.6666666667,40.0"),
        DeclareLaunchArgument(
            "insertion_plateau_visits", default_value="2"),
        DeclareLaunchArgument(
            "insertion_plateau_dwell_s", default_value="3.0"),
        DeclareLaunchArgument(
            "tendon_sweep_limits", default_value="0.0,15.0"),
        DeclareLaunchArgument("maximum_duration_s", default_value="950.0"),
        DeclareLaunchArgument(
            "require_estimator_tracking", default_value="true",
            description=(
                "Require accepted online markers and a TRACKING/DEGRADED UKF. "
                "Set false only for camera/SVO-first identification runs; "
                "controller-disarmed and adaptation-disabled status interlocks "
                "remain required.")),
        DeclareLaunchArgument("return_timeout_s", default_value="30.0"),
        DeclareLaunchArgument(
            "initial_position", default_value="20.0,0.0,0.0"),
        DeclareLaunchArgument(
            "initialization_timeout_s", default_value="30.0"),
    ]
    return LaunchDescription(
        arguments + [OpaqueFunction(function=_setup)])
