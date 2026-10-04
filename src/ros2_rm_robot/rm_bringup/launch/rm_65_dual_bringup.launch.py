import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    driver_launch = os.path.join(
        get_package_share_directory("rm_driver"),
        "launch",
        "rm_65_dual_driver.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch)),
    ])
