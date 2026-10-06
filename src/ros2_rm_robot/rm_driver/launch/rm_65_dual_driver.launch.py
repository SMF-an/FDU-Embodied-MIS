import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("rm_driver"), "config")
    left_config = os.path.join(config_dir, "rm_65_left_config.yaml")
    right_config = os.path.join(config_dir, "rm_65_right_config.yaml")

    def load_parameters(path):
        with open(path, "r", encoding="utf-8") as config_file:
            return yaml.safe_load(config_file)["rm_driver"]["ros__parameters"]

    return LaunchDescription([
        Node(
            package="rm_driver",
            executable="rm_driver",
            namespace="left_arm",
            parameters=[load_parameters(left_config)],
            output="screen",
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rm_driver",
                    executable="rm_driver",
                    namespace="right_arm",
                    parameters=[load_parameters(right_config)],
                    output="screen",
                )
            ],
        ),
    ])
