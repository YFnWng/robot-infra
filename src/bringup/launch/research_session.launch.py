"""One non-actuating-by-default controller, camera, and bag recording session."""
import math
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bringup.recording import RECORD_TOPICS
from experiments.recording_session import (
    SESSION_ROOT, allocate_session, utc_now, write_manifest)


def _boolean(value):
    if value not in ("true", "false"):
        raise ValueError("boolean arguments must be true or false")
    return value == "true"


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    video = _boolean(value("video"))
    reduced_filter_shadow = _boolean(value("reduced_filter_shadow"))
    stall_diagnostics = _boolean(value("camera_stall_diagnostics"))
    fused_policy = value("fused_timestamp_policy")
    if fused_policy not in ("newest", "weighted_mean"):
        raise ValueError("fused_timestamp_policy must be newest or weighted_mean")
    output_enabled = _boolean(value("command_output_enabled"))
    planner_process = value("planner_process_enabled")
    if planner_process:
        _boolean(planner_process)
    camera_config = Path(value("camera_config")).expanduser().resolve()
    registration = Path(value("registration_file")).expanduser().resolve()
    registration_config = Path(value("registration_config")).expanduser().resolve()
    registration_frames = int(value("registration_frames"))
    if not registration_config.is_file() or registration_frames < 10:
        raise ValueError("existing registration_config and >=10 registration_frames are required")
    # Preserve the venv entry path: resolving its symlink would select system
    # Python and lose the installed camera SDK/site-packages.
    camera_python = Path(value("camera_python")).expanduser().absolute()
    if not camera_config.is_file():
        raise ValueError("existing camera_config is required")
    if not camera_python.is_file():
        raise ValueError("camera_python must be the interpreter with the ZED SDK")
    if not value("stack_config"):
        raise ValueError("an explicit reviewed stack_config is required")
    timeout = float(value("startup_timeout_s"))
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("startup_timeout_s must be finite and positive")
    session = allocate_session(value("session_root"), value("session_label"))
    video_dir = session / f"{session.name}_video"
    topics = [*RECORD_TOPICS, "/experiments/session_status"]
    camera_command = [
        str(camera_python), "-m", "perception.marker_tracking", "--ros-args",
        "-p", f"camera_config:={camera_config}",
        "-p", "registration_refresh_enabled:=true",
        "-p", f"registration_config:={registration_config}",
        "-p", f"registration_frames:={registration_frames}",
        "-p", f"shape_tracking_root:={value('shape_tracking_root')}",
        "-p", "rig_ids:=['primary','oblique']",
        "-p", f"recording_enabled:={str(video).lower()}",
        "-p", f"recording_session_dir:={video_dir}",
        "-p", f"fused_timestamp_policy:={fused_policy}",
        "-p", f"stall_diagnostics_enabled:={str(stall_diagnostics).lower()}"]
    estimator_arguments = [
        f"{name}:={value(name)}"
        for name in ("estimator_device", "estimator_dtype", "mppi_temperature")
        if value(name)]
    if planner_process:
        estimator_arguments.append(f"planner_process_enabled:={planner_process}")
    if value('model_manifest'):
        estimator_arguments.append(f"model_manifest:={value('model_manifest')}")
    if value('path_prediction_grid'):
        estimator_arguments.append(f"path_prediction_grid:={value('path_prediction_grid')}")
    if value('point_rollout_step_override_s'):
        estimator_arguments.append(f"point_rollout_step_override_s:={value('point_rollout_step_override_s')}")
    if value('horizon_steps_override'):
        estimator_arguments.append(f"horizon_steps_override:={value('horizon_steps_override')}")
    commands = {
        "camera": camera_command,
        "bag": ["ros2", "bag", "record", "--include-hidden-topics",
                "-s", "sqlite3", "-o", str(session / "robot_bag"), *topics],
        "controller": [
            "ros2", "launch", "bringup", "control.launch.py",
            f"stack_config:={value('stack_config')}", "record:=false",
            *estimator_arguments,
            f"recording_manifest_path:={session / 'controller_manifest.json'}",
            f"adapj_initialization_dir:={session / 'adapj_initialization'}",
            f"command_output_enabled:={str(output_enabled).lower()}"]}
    if reduced_filter_shadow:
        # Research observer: own process, private status only, never commands.
        # The cr-venv wheel carries only the deployed runtime, so the research
        # filter is imported from the source workspace for this process alone.
        research_path = os.pathsep.join(
            item for item in (value("research_code_root"), os.environ.get("PYTHONPATH", "")) if item)
        commands["reduced_filter_shadow"] = [
            "env", f"PYTHONPATH={research_path}",
            "ros2", "run", "runtime_supervision", "reduced_filter_shadow",
            "--outdir", str(session / "reduced_filter_shadow"), "--duration-s", "3600"]
    manifest = {
        "schema_version": 1, "session_id": session.name,
        "session_label": value("session_label"), "allocated_at": utc_now(),
        "state": "allocated", "video_enabled": video,
        "video_directory": str(video_dir), "bag_directory": str(session / "robot_bag"),
        "stack_config": value("stack_config"), "camera_config": str(camera_config),
        "model_manifest": value('model_manifest'),
        "estimator_device": value("estimator_device"),
        "estimator_dtype": value("estimator_dtype"),
        "mppi_temperature": value("mppi_temperature"),
        "path_prediction_grid": value("path_prediction_grid"),
        "point_rollout_step_override_s": value("point_rollout_step_override_s"),
        "horizon_steps_override": value("horizon_steps_override"),
        "planner_process_enabled": planner_process,
        "registration_file": str(registration),
        "registration_config": str(registration_config),
        "registration_frames": registration_frames,
        "registration_policy": "session_charuco_independent_per_rig",
        "camera_python": str(camera_python),
        "command_output_enabled": output_enabled,
        "startup_timeout_s": timeout, "commands": commands,
        "reduced_filter_shadow": reduced_filter_shadow,
        "research_code_root": value("research_code_root") if reduced_filter_shadow else "",
        "record_topics": topics,
        "required_topics": ["/device/state", "/shape_tracking/markers",
                            "/shape_tracking/marker_status", "/catheter_mppi/status"],
    }
    path = session / "session_manifest.json"
    write_manifest(path, manifest)
    print(f"[research_session.launch] session -> {session}", flush=True)
    return [Node(
        package="experiments", executable="session_recording",
        name="research_session_recording", output="screen",
        arguments=["--manifest", str(path)],
        # Let the supervisor stop controller, finalize bag, then finalize SVO.
        sigterm_timeout="120", sigkill_timeout="10")]


def generate_launch_description():
    shape = os.environ.get("SHAPE_TRACKING_ROOT",
                           "/home/chen-lab/Yifan/catheter-shape-tracking")
    return LaunchDescription([
        DeclareLaunchArgument("session_label", description=(
            "Required semantic experiment/task/controller/budget/repeat label.")),
        DeclareLaunchArgument("stack_config", description="Reviewed semantic stack."),
        DeclareLaunchArgument("model_manifest", default_value="",
                             description="Explicit parallel model bundle; empty retains baseline."),
        DeclareLaunchArgument("estimator_device", default_value="",
                             description="Opt-in estimator device; empty inherits planner."),
        DeclareLaunchArgument("estimator_dtype", default_value="",
                             description="Opt-in estimator precision; empty preserves default."),
        DeclareLaunchArgument("mppi_temperature", default_value="",
                             description="Positive dimensionless MPPI temperature; empty preserves default."),
        DeclareLaunchArgument("path_prediction_grid", default_value="",
                             description="Opt-in point-reaching time grid for path rollouts; empty preserves stack."),
        DeclareLaunchArgument("point_rollout_step_override_s", default_value="",
                             description="Explicit point prediction step override in seconds; empty preserves stack."),
        DeclareLaunchArgument("horizon_steps_override", default_value="",
                             description="Explicit sampled horizon length in steps; empty preserves stack."),
        DeclareLaunchArgument("planner_process_enabled", default_value="",
                             description="Opt-in isolated planner process; empty preserves stack setting."),
        DeclareLaunchArgument("registration_file", default_value=os.environ.get(
            "CATHETER_REGISTRATION_FILE", ""),
            description="Legacy argument; fresh per-session registration does not use it."),
        DeclareLaunchArgument("registration_config", default_value=str(
            Path(shape) / "registration_config.yaml")),
        DeclareLaunchArgument("registration_frames", default_value="20"),
        DeclareLaunchArgument("shape_tracking_root", default_value=shape),
        DeclareLaunchArgument("camera_python", default_value=str(Path(
            os.environ.get("CR_VENV", "/home/chen-lab/Yifan/cr-venv")) / "bin/python"),
            description="Interpreter with installed shape_tracking and pyzed SDK."),
        DeclareLaunchArgument("camera_config", default_value=str(
            Path(shape) / "camera_config_hd720.yaml")),
        DeclareLaunchArgument("session_root", default_value=SESSION_ROOT),
        DeclareLaunchArgument("video", default_value="true"),
        DeclareLaunchArgument(
            "camera_stall_diagnostics", default_value="false",
            description="Add capture-loop, SDK-drop, GC and scheduling stall "
                        "fields to marker_status (needs a supporting wheel)."),
        DeclareLaunchArgument(
            "fused_timestamp_policy", default_value="newest",
            description="Fused marker stamp: newest (historical) or "
                        "weighted_mean (needs a supporting wheel)."),
        DeclareLaunchArgument(
            "reduced_filter_shadow", default_value="false",
            description="Also run the reduced-filter research observer (non-commanding; "
                        "report under the session directory)."),
        DeclareLaunchArgument(
            "research_code_root", default_value="/home/chen-lab/Yifan",
            description="Workspace containing the cr_meta_lnn source tree, used only by the observer."),
        DeclareLaunchArgument("command_output_enabled", default_value="false"),
        DeclareLaunchArgument("startup_timeout_s", default_value="60.0"),
        OpaqueFunction(function=_setup),
    ])
