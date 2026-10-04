import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config")
        .robot_description(
            file_path="config/rm_65_6fb_description.urdf.xacro",
            mappings={"link6_type": "Link6_6fb"},
        )
        .to_moveit_configs()
    )

    bringup_launch = os.path.join(
        get_package_share_directory("rm_bringup"),
        "launch",
        "rm_65_6fb_gazebo.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("derive_rcm_from_current", default_value="true"),
            DeclareLaunchArgument("rcm_x", default_value="0.0"),
            DeclareLaunchArgument("rcm_y", default_value="0.0"),
            DeclareLaunchArgument("rcm_z", default_value="0.0"),
            DeclareLaunchArgument("rcm_distance_from_tip_m", default_value="0.15"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch)),
            Node(
                package="rm_moveit2",
                executable="rm65_rcm_demo",
                name="rm65_rcm_demo",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "use_sim_time": True,
                        "planning_group": "rm_group",
                        "derive_rcm_from_current": ParameterValue(
                            LaunchConfiguration("derive_rcm_from_current"), value_type=bool
                        ),
                        "rcm_x": ParameterValue(
                            LaunchConfiguration("rcm_x"), value_type=float
                        ),
                        "rcm_y": ParameterValue(
                            LaunchConfiguration("rcm_y"), value_type=float
                        ),
                        "rcm_z": ParameterValue(
                            LaunchConfiguration("rcm_z"), value_type=float
                        ),
                        "rcm_distance_from_tip_m": ParameterValue(
                            LaunchConfiguration("rcm_distance_from_tip_m"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
