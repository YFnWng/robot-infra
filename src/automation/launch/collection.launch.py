"""Automated data-collection launch.

Composes the collection node (velocity commands) + em_bridge (EM -> PoseArray) +
rosbag2 recording into one command, and shuts everything down cleanly when the
collection trajectory finishes.

Start the ZED SVO2 recorder (Windows, shape_tracking) MANUALLY first, then:

    ros2 launch automation collection.launch.py \
        target:=catheter mode:=sinusoidal duration_s:=60 seed:=1 \
        joint_lower:=0.0,-180.0,0.0 joint_upper:=0.1,180.0,0.01

Recorded topics land in <session_root>/<ts>_<target>_<mode>/robot_bag.
Set start_motor:=true only when hardware is connected and safe.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RECORD_TOPICS = [
    "/teleop/control",       # commanded velocity (reference)
    "/device/state",         # raw device stream incl. ENC (model input)
    "/manager/state",        # nominal joint state
    "/em_tracker/poses",     # EM coil poses
    "/manager/event",
    "/device/event",
    "/collection/events",    # run markers
]


def _floats(text):
    return [float(x) for x in str(text).split(",") if x != ""]


def launch_setup(context, *_args, **_kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    target = cfg("target")
    mode = cfg("mode")
    session_root = os.path.expanduser(cfg("session_root"))
    record = cfg("record").lower() in ("true", "1", "yes")
    storage = cfg("storage_id")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(session_root, f"{ts}_{target}_{mode}")
    os.makedirs(session_dir, exist_ok=True)
    bag_dir = os.path.join(session_dir, "robot_bag")

    collection = Node(
        package="automation",
        executable="collection",
        name="collection",
        output="screen",
        parameters=[{
            "mode": mode,
            "target": target,
            "seed": int(cfg("seed")),
            "duration_s": float(cfg("duration_s")),
            "rate_hz": float(cfg("rate_hz")),
            "start_motor": cfg("start_motor").lower() in ("true", "1", "yes"),
            "sofa_sim_path": cfg("sofa_sim_path"),
            "limits_file": cfg("limits_file"),
            "catheter": cfg("catheter"),
            "expect_enc": cfg("expect_enc").lower() in ("true", "1", "yes"),
            "joint_lower": _floats(cfg("joint_lower")),
            "joint_upper": _floats(cfg("joint_upper")),
            "return_to_start": cfg("return_to_start").lower() in ("true", "1", "yes"),
            "shutdown_on_done": True,
        }, {
            'return_control_mode': cfg('return_control_mode'),
            'return_to_zero': cfg('return_to_zero').lower() in ('true', '1', 'yes'),
            'return_position_speed_factor': float(
                cfg('return_position_speed_factor')),
            'return_position_tolerance': _floats(
                cfg('return_position_tolerance')),
            'return_position_settle_s': float(
                cfg('return_position_settle_s')),
            'return_timeout_s': float(cfg('return_timeout_s')),
        }],
    )

    em_bridge = Node(
        package="automation",
        executable="em_bridge",
        name="em_bridge",
        output="screen",
        parameters=[{
            "input_topic": cfg("em_input_topic"),
            "device_name": cfg("em_device_name"),
            "num_coils": int(cfg("em_num_coils")),
        }],
    )

    actions = [em_bridge, collection]

    if record:
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_dir,
                 "--storage", storage, *RECORD_TOPICS],
            output="screen",
        ))
        print(f"[collection.launch] recording -> {bag_dir}")

    # When the trajectory finishes, shut the whole launch down (flushes rosbag).
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=collection,
        on_exit=[EmitEvent(event=Shutdown(reason="collection complete"))],
    )))
    return actions


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    default_limits = os.path.join(
        get_package_share_directory("automation"), "config", "catheter_limits.yaml")
    args = [
        DeclareLaunchArgument('return_control_mode', default_value='position'),
        DeclareLaunchArgument('return_to_zero', default_value='false'),
        DeclareLaunchArgument(
            'return_position_speed_factor', default_value='0.5'),
        DeclareLaunchArgument(
            'return_position_tolerance',
            default_value='0.1,0.5,0.05,0.1,0.5,0.5'),
        DeclareLaunchArgument('return_position_settle_s', default_value='0.2'),
        DeclareLaunchArgument('return_timeout_s', default_value='30.0'),
        DeclareLaunchArgument("target", default_value="catheter"),      # catheter|sheath
        DeclareLaunchArgument("mode", default_value="sinusoidal"),      # sinusoidal|constant
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("limits_file", default_value=default_limits),
        DeclareLaunchArgument("catheter", default_value="default"),     # profile in limits_file
        DeclareLaunchArgument("expect_enc", default_value="true"),
        DeclareLaunchArgument("duration_s", default_value="60.0"),
        DeclareLaunchArgument("rate_hz", default_value="100.0"),
        DeclareLaunchArgument("start_motor", default_value="false"),
        DeclareLaunchArgument("return_to_start", default_value="true"),
        DeclareLaunchArgument("sofa_sim_path",
                              default_value="/home/wangyf/sofa-cosserat-sim"),
        DeclareLaunchArgument("joint_lower", default_value="0.0,-180.0,0.0"),
        DeclareLaunchArgument("joint_upper", default_value="0.1,180.0,0.01"),
        DeclareLaunchArgument("session_root", default_value="~/catheter_sessions"),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("storage_id", default_value="mcap"),      # mcap|sqlite3
        DeclareLaunchArgument("em_input_topic", default_value="/IGTL_POINT_IN"),
        DeclareLaunchArgument("em_device_name", default_value=""),
        DeclareLaunchArgument("em_num_coils", default_value="0"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
