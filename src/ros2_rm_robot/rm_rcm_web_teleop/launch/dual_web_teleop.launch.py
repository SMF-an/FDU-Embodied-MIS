from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8765"),
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
                        "camera_topic": LaunchConfiguration("camera_topic"),
                        "step_angle_deg": 2.0,
                        "step_insertion_m": 0.002,
                    }
                ],
            ),
        ]
    )
