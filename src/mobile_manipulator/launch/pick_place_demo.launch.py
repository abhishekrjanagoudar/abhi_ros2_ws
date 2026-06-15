"""
Pick-and-Place Demo Launch File
================================
Starts the complete simulation stack plus the pick_place_node.py demo.

This is the single command that runs the full Phase 2 demonstration:

  ros2 launch mobile_manipulator pick_place_demo.launch.py

What it does:
  1. Runs simulation.launch.py (Gazebo + RSP + controllers + MoveIt + RViz)
  2. After a 12-second startup delay, launches pick_place_node.py

The pick_place_node.py then:
  - Stows the arm for navigation
  - Drives the robot to the pick position (Nav2)
  - Opens the gripper
  - Moves arm to pre-grasp position
  - Lowers to grasp
  - Closes gripper (picks the red cube)
  - Lifts the object
  - Pans the arm to place position
  - Lowers to place height
  - Opens gripper (places the cube)
  - Returns arm to home

Monitor progress:
  ros2 topic echo /task_status

Abort the demo:
  ros2 topic pub /task_goal std_msgs/String "data: abort" --once

Launch arguments:
  gz        : 'true' to show Gazebo GUI
  use_rviz  : 'false' to skip RViz
  use_moveit: 'false' to skip MoveIt (uses pre-computed trajectories only)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the pick-and-place demo LaunchDescription."""

    pkg_mm = get_package_share_directory('mobile_manipulator')

    declare_gz         = DeclareLaunchArgument('gz',        default_value='false')
    declare_use_rviz   = DeclareLaunchArgument('use_rviz',  default_value='true')
    declare_use_moveit = DeclareLaunchArgument('use_moveit', default_value='true')

    # ── Full simulation stack ─────────────────────────────────────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mm, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'gz': LaunchConfiguration('gz'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_moveit': LaunchConfiguration('use_moveit'),
        }.items(),
    )

    # ── arm_planner node (starts at t=13s) ───────────────────────────────────
    # Provides the MoveIt-style interface that pick_place_node talks to.
    arm_planner = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='mobile_manipulator',
                executable='arm_planner.py',
                name='arm_planner',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # ── ik_node (starts at t=13s) ────────────────────────────────────────────
    # Provides the IK service for Phase 3 vision integration.
    ik_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='mobile_manipulator',
                executable='ik.py',
                name='arm_ik_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # ── pick_place_node (starts at t=15s, after all services ready) ─────────
    # Delayed 5 seconds beyond move_group (t=10s) to ensure action servers ready.
    pick_place_demo = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='mobile_manipulator',
                executable='pick_place_node.py',
                name='pick_place_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription([
        declare_gz,
        declare_use_rviz,
        declare_use_moveit,
        simulation,
        arm_planner,
        ik_node,
        pick_place_demo,
    ])
