"""Launch native dual-ZED tracking with a bounded diagnostic overlay stream."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shape_root = LaunchConfiguration("shape_tracking_root")
    registration = LaunchConfiguration("registration_file")
    preview_eye = LaunchConfiguration("preview_eye")
    return LaunchDescription([
        DeclareLaunchArgument(
            "shape_tracking_root",
            default_value=os.environ.get(
                "SHAPE_TRACKING_ROOT",
                "/home/chen-lab/Yifan/catheter-shape-tracking")),
        DeclareLaunchArgument(
            "model_manifest",
            default_value=os.environ.get(
                "CATHETER_MODEL_MANIFEST",
                os.path.join(
                    os.environ.get("CR_VENV", "/home/chen-lab/Yifan/cr-venv"),
                    "lib/python3.10/site-packages/cr_meta_lnn/artifacts/"
                    "manifests/20260929_175554_grouped_no_rotation_v2.json"))),
        DeclareLaunchArgument(
            "camera_config",
            default_value=[shape_root, "/camera_config_hd720.yaml"]),
        DeclareLaunchArgument(
            "registration_file",
            default_value=os.environ.get("CATHETER_REGISTRATION_FILE", "")),
        DeclareLaunchArgument("preview_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("preview_maximum_width", default_value="640"),
        DeclareLaunchArgument("preview_jpeg_quality", default_value="70"),
        DeclareLaunchArgument("preview_eye", default_value="left"),
        DeclareLaunchArgument("marker_crosshair_size", default_value="10"),
        DeclareLaunchArgument(
            "marker_crosshair_line_width", default_value="1"),
        DeclareLaunchArgument("target_path_line_width", default_value="1"),
        DeclareLaunchArgument("show_window", default_value="true"),
        Node(
            package="perception",
            executable="marker_tracking",
            name="marker_tracking",
            output="screen",
            parameters=[{
                "shape_tracking_root": shape_root,
                "camera_config": LaunchConfiguration("camera_config"),
                "registration_file": registration,
                "rig_ids": ["primary", "oblique"],
                "preview_enabled": True,
                "preview_rate_hz": ParameterValue(
                    LaunchConfiguration("preview_rate_hz"), value_type=float),
                "preview_maximum_width": ParameterValue(
                    LaunchConfiguration("preview_maximum_width"),
                    value_type=int),
                "preview_jpeg_quality": ParameterValue(
                    LaunchConfiguration("preview_jpeg_quality"),
                    value_type=int),
                "preview_eye": preview_eye,
            }]),
        Node(
            package="control_tasks",
            executable="catheter_camera_overlay",
            name="catheter_camera_overlay",
            output="screen",
            parameters=[{
                "model_manifest": LaunchConfiguration("model_manifest"),
                "registration_file": registration,
                "rig_ids": ["primary", "oblique"],
                "preview_eye": preview_eye,
                "display_rate_hz": ParameterValue(
                    LaunchConfiguration("preview_rate_hz"), value_type=float),
                "marker_crosshair_size": ParameterValue(
                    LaunchConfiguration("marker_crosshair_size"),
                    value_type=int),
                "marker_crosshair_line_width": ParameterValue(
                    LaunchConfiguration("marker_crosshair_line_width"),
                    value_type=int),
                "target_path_line_width": ParameterValue(
                    LaunchConfiguration("target_path_line_width"),
                    value_type=int),
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"), value_type=bool),
            }]),
    ])
