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
    driver_launch = os.path.join(
        get_package_share_directory("rm_driver"),
        "launch",
        "rm_65_dual_driver.launch.py",
    )
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch)),
    ]

    nodes.extend(
        DeclareLaunchArgument(f"{side}_rcm_{axis}", default_value="nan")
        for side in ("left", "right")
        for axis in ("x", "y", "z")
    )
    nodes.extend(
        [
            DeclareLaunchArgument("velocity_scaling", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.02"),
            DeclareLaunchArgument("max_rcm_rotation_deg", default_value="2.0"),
            DeclareLaunchArgument("max_rcm_insertion_m", default_value="0.002"),
        ]
    )

    for side, prefix in (("left", "left_"), ("right", "right_")):
        moveit_config = (
            MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config")
            .robot_description(
                file_path="config/rm_65_6fb_description.urdf.xacro",
                mappings={"link6_type": "Link6_6fb"},
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .to_moveit_configs()
        )
        namespace = f"{side}_arm"
        remappings = [
            ("/joint_states", "moveit_joint_states"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ]
        rcm_parameters = moveit_config.to_dict()
        rcm_parameters.update(
            {
                "use_sim_time": False,
                "planning_group": "rm_group",
                "derive_rcm_from_current": False,
                "rcm_x": ParameterValue(
                    LaunchConfiguration(f"{side}_rcm_x"), value_type=float
                ),
                "rcm_y": ParameterValue(
                    LaunchConfiguration(f"{side}_rcm_y"), value_type=float
                ),
                "rcm_z": ParameterValue(
                    LaunchConfiguration(f"{side}_rcm_z"), value_type=float
                ),
                "velocity_scaling": ParameterValue(
                    LaunchConfiguration("velocity_scaling"), value_type=float
                ),
                "acceleration_scaling": ParameterValue(
                    LaunchConfiguration("acceleration_scaling"), value_type=float
                ),
                "max_rcm_rotation_deg": ParameterValue(
                    LaunchConfiguration("max_rcm_rotation_deg"), value_type=float
                ),
                "max_rcm_insertion_m": ParameterValue(
                    LaunchConfiguration("max_rcm_insertion_m"), value_type=float
                ),
            }
        )

        nodes.extend(
            [
                Node(
                    package="rm_control",
                    executable="rm_control",
                    namespace=namespace,
                    name="rm_control",
                    parameters=[{"follow": False, "arm_type": 65}],
                    output="screen",
                ),
                Node(
                    package="rm_description",
                    executable="joint_state_prefix_bridge.py",
                    namespace=namespace,
                    name="moveit_joint_state_bridge",
                    parameters=[
                        {
                            "input_topic": f"/{namespace}/joint_states",
                            "output_topic": f"/{namespace}/moveit_joint_states",
                            "joint_prefix": prefix,
                        }
                    ],
                    output="screen",
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=namespace,
                    name="robot_state_publisher",
                    parameters=[{"robot_description": moveit_config.robot_description}],
                    remappings=remappings,
                    output="screen",
                ),
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    namespace=namespace,
                    name="move_group",
                    parameters=[
                        moveit_config.to_dict(),
                        {
                            "use_sim_time": False,
                            "allow_trajectory_execution": True,
                            "moveit_manage_controllers": False,
                            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                            "trajectory_execution.allowed_goal_duration_margin": 0.5,
                            "trajectory_execution.allowed_start_tolerance": 0.15,
                        },
                    ],
                    remappings=remappings,
                    output="screen",
                ),
                Node(
                    package="rm_moveit2",
                    executable="rm65_rcm_demo",
                    namespace=namespace,
                    name="rm65_rcm_demo",
                    parameters=[rcm_parameters],
                    remappings=remappings,
                    output="screen",
                ),
            ]
        )

    return LaunchDescription(nodes)
