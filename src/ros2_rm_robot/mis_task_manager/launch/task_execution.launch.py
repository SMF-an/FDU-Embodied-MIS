from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("scene_topic", default_value="/perception/scene"),
        DeclareLaunchArgument("color_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera/color/camera_info"
        ),
        DeclareLaunchArgument("camera_topic", default_value="/perception/overlay"),
        DeclareLaunchArgument("target_frame", default_value="world"),
        DeclareLaunchArgument("camera_extrinsics_file", default_value=""),
        DeclareLaunchArgument("camera_extrinsics_arm", default_value="left"),
        DeclareLaunchArgument("material_profile", default_value="painted"),
        DeclareLaunchArgument("lesion_profile", default_value="material"),
        DeclareLaunchArgument("obstacle_margin_m", default_value="0.015"),
        DeclareLaunchArgument("obstacle_min_extent_m", default_value="0.04"),
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8780"),
        DeclareLaunchArgument("status_topic", default_value="/task/status"),
        DeclareLaunchArgument("servo_config_file", default_value=""),
    ]
    package_share = FindPackageShare("mis_task_manager")
    perception_share = FindPackageShare("mis_perception")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([perception_share, "launch", "perception.launch.py"])
        ),
        launch_arguments={
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
        }.items(),
    )
    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "rcm_task_executor.launch.py"])
        ),
        launch_arguments={
            "motion_enabled": LaunchConfiguration("motion_enabled"),
            "scene_topic": LaunchConfiguration("scene_topic"),
        }.items(),
    )
    state_machine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "task_state_machine.launch.py"])
        ),
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "scene_topic": LaunchConfiguration("scene_topic"),
        }.items(),
    )
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "task_dashboard.launch.py"])
        ),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "scene_topic": LaunchConfiguration("scene_topic"),
            "status_topic": LaunchConfiguration("status_topic"),
            "servo_config_file": LaunchConfiguration("servo_config_file"),
        }.items(),
    )
    return LaunchDescription(
        arguments + [perception, executor, state_machine, dashboard]
    )
