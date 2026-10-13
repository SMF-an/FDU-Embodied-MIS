from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("scene_topic", default_value="/perception/scene"),
        DeclareLaunchArgument("scene_max_age_sec", default_value="0.5"),
        DeclareLaunchArgument("state_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("motion_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument("servo_timeout_sec", default_value="110.0"),
        DeclareLaunchArgument("servo_max_iterations", default_value="30"),
        DeclareLaunchArgument("servo_max_radial_step_m", default_value="0.008"),
        DeclareLaunchArgument("servo_max_rotation_deg", default_value="2.0"),
        DeclareLaunchArgument("servo_position_tolerance_m", default_value="0.01"),
        DeclareLaunchArgument("minimum_target_quality", default_value="0.05"),
    ]
    node = Node(
        package="mis_task_manager",
        executable="rcm_task_executor",
        name="mis_rcm_task_executor",
        output="screen",
        parameters=[
            {
                "motion_enabled": ParameterValue(
                    LaunchConfiguration("motion_enabled"), value_type=bool
                ),
                "scene_topic": LaunchConfiguration("scene_topic"),
                "minimum_target_quality": ParameterValue(
                    LaunchConfiguration("minimum_target_quality"), value_type=float
                ),
                "state_timeout_sec": ParameterValue(
                    LaunchConfiguration("state_timeout_sec"), value_type=float
                ),
                "scene_max_age_sec": ParameterValue(
                    LaunchConfiguration("scene_max_age_sec"), value_type=float
                ),
                "motion_timeout_sec": ParameterValue(
                    LaunchConfiguration("motion_timeout_sec"), value_type=float
                ),
                "servo_timeout_sec": ParameterValue(
                    LaunchConfiguration("servo_timeout_sec"), value_type=float
                ),
                "servo_max_iterations": ParameterValue(
                    LaunchConfiguration("servo_max_iterations"), value_type=int
                ),
                "servo_max_radial_step_m": ParameterValue(
                    LaunchConfiguration("servo_max_radial_step_m"), value_type=float
                ),
                "servo_max_rotation_deg": ParameterValue(
                    LaunchConfiguration("servo_max_rotation_deg"), value_type=float
                ),
                "servo_position_tolerance_m": ParameterValue(
                    LaunchConfiguration("servo_position_tolerance_m"), value_type=float
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
