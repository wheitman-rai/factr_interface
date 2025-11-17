"""
Launch file for FACTR teleoperation with RViz visualization.

This launch file starts:
1. The FACTR teleoperation node
2. Robot state publisher for the URDF
3. RViz2 for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("factr_interface")
    urdf_file = os.path.join(pkg_dir, "urdf", "factr_teleop_franka.urdf")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "factr_teleop.rviz")

    # Robot description parameter
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            # Robot State Publisher - publishes TF transforms from URDF and /robot_description topic
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="factr_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/joint_states", "/factr/joint_states"),
                    ("/robot_description", "/factr/robot_description"),
                ],
            ),
            # FACTR Teleoperation Node
            # Node(
            #     package="factr_interface",
            #     executable="factr_teleop_node",
            #     name="factr_teleop_node",
            #     output="screen",
            #     parameters=[{"use_sim_time": use_sim_time}],
            # ),
            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=(
                    ["-d", rviz_config_file] if os.path.exists(rviz_config_file) else []
                ),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
