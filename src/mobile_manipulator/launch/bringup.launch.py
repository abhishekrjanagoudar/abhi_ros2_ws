"""
Bringup Launch File — mobile_manipulator
=========================================
Loads the combined mobile_manipulator URDF into robot_state_publisher
and spawns the arm + gripper controllers into the controller_manager
that was started by mobile_description's Gazebo plugin.

This file is NOT intended to be run standalone — it is included by
simulation.launch.py after Gazebo and the AGV are already running.

What it does:
  1. Compile mobile_manipulator.urdf.xacro → URDF string
  2. Start robot_state_publisher with the combined URDF
  3. Spawn arm_controller and gripper_controller (with param-file)

Prerequisites:
  - Gazebo Harmonic running with the AGV already spawned
  - controller_manager already started by gz_ros2_control plugin
  - /clock topic available (gz_ros2_bridge)

Launch arguments:
  use_sim_time : true (default) when Gazebo is running
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    """Build the bringup LaunchDescription."""

    pkg_mm = get_package_share_directory('mobile_manipulator')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo for time synchronisation.',
    )

    def launch_setup(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        # ── 1. Compile combined URDF ─────────────────────────────────────
        xacro_file = os.path.join(
            pkg_mm, 'urdf', 'mobile_manipulator.urdf.xacro'
        )
        robot_description_str = subprocess.check_output(
            ['xacro', xacro_file], text=True
        )

        # ── 2. Robot State Publisher (combined URDF) ─────────────────────
        # This REPLACES the one started by rsp.launch.py (from mobile_description)
        # when used in a combined system. It knows about all links including the arm.
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_str,
                'use_sim_time': use_sim_time,
                # Publish rate for joint_states processing
                'publish_frequency': 50.0,
            }],
        )

        # ── 3. Arm controller spawner ────────────────────────────────────
        # --param-file loads arm_controller type + parameters.
        # Controller_manager was started by gz_ros2_control plugin;
        # it already knows about the ArmSystem ros2_control block from the URDF.
        arm_controllers_yaml = os.path.join(
            pkg_mm, 'config', 'arm_controllers.yaml'
        )

        # Spawn arm_controller with 5-second delay to allow controller_manager startup
        arm_controller_spawner = TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'arm_controller',
                        '--param-file', arm_controllers_yaml,
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ],
        )

        # Spawn gripper_controller with 5.5-second delay (after arm_controller)
        gripper_controller_spawner = TimerAction(
            period=5.5,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'gripper_controller',
                        '--param-file', arm_controllers_yaml,
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ],
        )

        return [
            rsp_node,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]

    return LaunchDescription([
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
