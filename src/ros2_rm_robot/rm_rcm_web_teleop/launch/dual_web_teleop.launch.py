import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory("rm_rcm_web_teleop")
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(package_share)))
    )
    default_data_directory = os.path.join(workspace_root, "data")
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("agent_enabled", default_value="false"),
            DeclareLaunchArgument("agent_max_action_hz", default_value="2.0"),
            DeclareLaunchArgument("data_directory", default_value=default_data_directory),
            DeclareLaunchArgument(
                "camera_topic", default_value="/camera/camera/color/image_raw"
            ),
            Node(
                package="rm_rcm_web_teleop",
                executable="dual_rcm_web_teleop",
                name="dual_rcm_web_teleop",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "agent_enabled": LaunchConfiguration("agent_enabled"),
                        "agent_max_action_hz": LaunchConfiguration("agent_max_action_hz"),
                        "camera_topic": LaunchConfiguration("camera_topic"),
                        "data_directory": LaunchConfiguration("data_directory"),
                        "step_angle_deg": 2.0,
                        "step_roll_deg": 2.0,
                        "step_insertion_m": 0.005,
                        "large_step_multiplier": 2.0,
                    }
                ],
            ),
        ]
    )
