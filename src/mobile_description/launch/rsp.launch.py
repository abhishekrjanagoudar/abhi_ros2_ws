"""
Robot State Publisher Launch File
=================================
Loads the robot URDF (compiled from xacro) and starts robot_state_publisher,
which broadcasts the TF tree so RViz/Gazebo know where each link is.

Steps:
  1. Locate xacro file at  config/robot/mobile_robot.urdf.xacro
  2. Compile xacro -> URDF XML at launch time (Command substitution)
  3. Start robot_state_publisher with URDF as 'robot_description'

Launch arguments:
  use_sim_time : true when running with Gazebo (uses /clock topic)

Usage:
  ros2 launch mobile_description rsp.launch.py
  ros2 launch mobile_description rsp.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'mobile_description'
    xacro_subpath = 'config/robot/mobile_robot.urdf.xacro'

    # ── Locate + compile xacro -> URDF string at launch time ─────────────────
    xacro_file = os.path.join(get_package_share_directory(pkg_name), xacro_subpath)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true when Gazebo publishes /clock)',
    )

    # ── robot_state_publisher: broadcasts TF tree from URDF + /joint_states ──
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_rsp,
    ])
