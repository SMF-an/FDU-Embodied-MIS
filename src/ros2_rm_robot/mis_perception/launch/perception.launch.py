from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera/color/camera_info"
        ),
        DeclareLaunchArgument("target_frame", default_value="world"),
        DeclareLaunchArgument("camera_extrinsics_file", default_value=""),
        DeclareLaunchArgument("camera_extrinsics_arm", default_value="left"),
        DeclareLaunchArgument("material_profile", default_value="painted"),
        DeclareLaunchArgument("lesion_profile", default_value="material"),
        DeclareLaunchArgument("obstacle_margin_m", default_value="0.015"),
        DeclareLaunchArgument("obstacle_min_extent_m", default_value="0.04"),
    ]
    node = Node(
        package="mis_perception",
        executable="d405_perception",
        name="d405_perception",
        output="screen",
        parameters=[
            {
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "camera_extrinsics_file": LaunchConfiguration("camera_extrinsics_file"),
                "camera_extrinsics_arm": LaunchConfiguration("camera_extrinsics_arm"),
                "material_profile": LaunchConfiguration("material_profile"),
                "lesion_profile": LaunchConfiguration("lesion_profile"),
                "obstacle_margin_m": LaunchConfiguration("obstacle_margin_m"),
                "obstacle_min_extent_m": LaunchConfiguration("obstacle_min_extent_m"),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
