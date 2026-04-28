"""
Robot State Publisher Launch File
1. Load URDF from xacro file
2. Start robot_state_publisher node (broadcasts robot transforms)
3. RViz can now visualize robot structure
"""

import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Main function: creates and returns launch configuration

    # Get xacro file path from package
    pkg_name = 'mobile_description'
    file_subpath = 'urdf/mobile_robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    # Convert xacro to URDF during launch
    robot_description_raw = Command(['xacro ', xacro_file])

    # Robot State Publisher node: broadcasts transform tree
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_raw, value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')  # For Gazebo simulation
        }]
    )

    # Joint State Publisher GUI: needs robot_description to discover joints and show sliders
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_raw, value_type=str)}]
    )

    # Launch argument: use simulated time (useful for Gazebo)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    # Return launch configuration with all nodes
    return LaunchDescription([
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
        node_joint_state_publisher
    ])
