from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8765"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="rm_rcm_web_teleop",
                executable="rcm_web_teleop",
                name="rcm_web_teleop",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "step_angle_deg": 2.0,
                        "step_insertion_m": 0.002,
                    }
                ],
            ),
        ]
    )
