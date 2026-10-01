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
            "cr_meta_lnn_root",
            default_value=os.environ.get(
                "CR_META_LNN_ROOT", "/home/chen-lab/Yifan/cr_meta_lnn")),
        DeclareLaunchArgument(
            "cr_common_root",
            default_value=os.environ.get(
                "CR_COMMON_ROOT", "/home/chen-lab/Yifan/cr-common")),
        DeclareLaunchArgument(
            "camera_config",
            default_value=[shape_root, "/camera_config_hd720.yaml"]),
        DeclareLaunchArgument(
            "registration_file",
            default_value=os.environ.get("CATHETER_REGISTRATION_FILE", "")),
        DeclareLaunchArgument("v171_distal_checkpoint", default_value=""),
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
            package="automation",
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
            package="catheter_control",
            executable="catheter_camera_overlay",
            name="catheter_camera_overlay",
            output="screen",
            parameters=[{
                "shape_tracking_root": shape_root,
                "cr_meta_lnn_root": LaunchConfiguration(
                    "cr_meta_lnn_root"),
                "cr_common_root": LaunchConfiguration("cr_common_root"),
                "registration_file": registration,
                "v171_distal_checkpoint": LaunchConfiguration(
                    "v171_distal_checkpoint"),
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
