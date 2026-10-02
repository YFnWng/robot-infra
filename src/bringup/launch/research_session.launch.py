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
    output_enabled = _boolean(value("command_output_enabled"))
    camera_config = Path(value("camera_config")).expanduser().resolve()
    registration = Path(value("registration_file")).expanduser().resolve()
    # Preserve the venv entry path: resolving its symlink would select system
    # Python and lose the installed camera SDK/site-packages.
    camera_python = Path(value("camera_python")).expanduser().absolute()
    if not camera_config.is_file() or not registration.is_file():
        raise ValueError("existing camera_config and registration_file are required")
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
        "-p", f"registration_file:={registration}",
        "-p", f"shape_tracking_root:={value('shape_tracking_root')}",
        "-p", "rig_ids:=['primary','oblique']",
        "-p", f"recording_enabled:={str(video).lower()}",
        "-p", f"recording_session_dir:={video_dir}"]
    commands = {
        "camera": camera_command,
        "bag": ["ros2", "bag", "record", "--include-hidden-topics",
                "-s", "sqlite3", "-o", str(session / "robot_bag"), *topics],
        "controller": [
            "ros2", "launch", "bringup", "control.launch.py",
            f"stack_config:={value('stack_config')}", "record:=false",
            f"recording_manifest_path:={session / 'controller_manifest.json'}",
            f"command_output_enabled:={str(output_enabled).lower()}"]}
    manifest = {
        "schema_version": 1, "session_id": session.name,
        "session_label": value("session_label"), "allocated_at": utc_now(),
        "state": "allocated", "video_enabled": video,
        "video_directory": str(video_dir), "bag_directory": str(session / "robot_bag"),
        "stack_config": value("stack_config"), "camera_config": str(camera_config),
        "registration_file": str(registration),
        "camera_python": str(camera_python),
        "command_output_enabled": output_enabled,
        "startup_timeout_s": timeout, "commands": commands,
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
        DeclareLaunchArgument("registration_file", default_value=os.environ.get(
            "CATHETER_REGISTRATION_FILE", "")),
        DeclareLaunchArgument("shape_tracking_root", default_value=shape),
        DeclareLaunchArgument("camera_python", default_value=str(Path(
            os.environ.get("CR_VENV", "/home/chen-lab/Yifan/cr-venv")) / "bin/python"),
            description="Interpreter with installed shape_tracking and pyzed SDK."),
        DeclareLaunchArgument("camera_config", default_value=str(
            Path(shape) / "camera_config_hd720.yaml")),
        DeclareLaunchArgument("session_root", default_value=SESSION_ROOT),
        DeclareLaunchArgument("video", default_value="true"),
        DeclareLaunchArgument("command_output_enabled", default_value="false"),
        DeclareLaunchArgument("startup_timeout_s", default_value="60.0"),
        OpaqueFunction(function=_setup),
    ])
