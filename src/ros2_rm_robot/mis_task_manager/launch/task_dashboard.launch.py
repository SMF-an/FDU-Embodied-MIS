from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8780"),
        DeclareLaunchArgument(
            "camera_topic", default_value="/perception/overlay"
        ),
        DeclareLaunchArgument("scene_topic", default_value="/perception/scene"),
        DeclareLaunchArgument("status_topic", default_value="/task/status"),
        DeclareLaunchArgument("servo_config_file", default_value=""),
    ]
    node = Node(
        package="mis_task_manager",
        executable="task_dashboard",
        name="mis_task_dashboard",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "camera_topic": LaunchConfiguration("camera_topic"),
                "scene_topic": LaunchConfiguration("scene_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "servo_config_file": LaunchConfiguration("servo_config_file"),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
