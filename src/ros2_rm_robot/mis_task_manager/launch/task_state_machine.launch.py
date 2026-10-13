from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("scene_topic", default_value="/perception/scene"),
        DeclareLaunchArgument("dry_run", default_value="true"),
    ]
    node = Node(
        package="mis_task_manager",
        executable="task_state_machine",
        name="mis_task_state_machine",
        output="screen",
        parameters=[
            {
                "scene_topic": LaunchConfiguration("scene_topic"),
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"), value_type=bool
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
