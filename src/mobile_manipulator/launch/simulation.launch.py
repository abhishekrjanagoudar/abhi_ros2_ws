"""
Simulation Launch File — mobile_manipulator  (BACKWARD-COMPATIBILITY WRAPPER)
============================================================================
`robot.launch.py` is now the canonical single entry point for the unified
mobile manipulator (AGV base + 5-DOF arm + 2-finger gripper). This file is
kept only as a thin wrapper so existing references to
`simulation.launch.py` keep working — it simply includes `robot.launch.py`
and forwards every launch argument unchanged.

There is intentionally NO orchestration logic here (no Gazebo, RSP, spawn,
controller, MoveIt or RViz actions). All of that lives in `robot.launch.py`.
Maintaining the launch graph in one place avoids duplication and drift.

Launch arguments (forwarded verbatim to robot.launch.py):
  gz        : 'true' to open the Gazebo GUI (default: true).
  use_rviz  : 'true' to open RViz2 (default: true).
  use_moveit: 'true' to start move_group (default: true).
  world     : world name selecting <name>.world under worlds/ (default:
              pick_place). May also be an absolute path to a .world file.
  env       : AGV environment model path for GZ_SIM_RESOURCE_PATH
              (default: cpr_office).

Usage (equivalent to launching robot.launch.py directly):
  ros2 launch mobile_manipulator simulation.launch.py
  ros2 launch mobile_manipulator simulation.launch.py gz:=false
  ros2 launch mobile_manipulator simulation.launch.py use_moveit:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Thin wrapper: declare the canonical args and include robot.launch.py."""

    # ── Declare the same arguments as robot.launch.py (matching defaults) ──
    declare_gz         = DeclareLaunchArgument('gz',         default_value='true')
    declare_use_rviz   = DeclareLaunchArgument('use_rviz',   default_value='true')
    declare_use_moveit = DeclareLaunchArgument('use_moveit', default_value='true')
    declare_world      = DeclareLaunchArgument('world',      default_value='pick_place')
    declare_env        = DeclareLaunchArgument('env',        default_value='cpr_office')

    # ── Locate the installed canonical launch file ────────────────────────
    robot_launch_path = os.path.join(
        get_package_share_directory('mobile_manipulator'),
        'launch', 'robot.launch.py'
    )

    # ── Include robot.launch.py, forwarding all launch arguments ──────────
    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={
            'gz':         LaunchConfiguration('gz'),
            'use_rviz':   LaunchConfiguration('use_rviz'),
            'use_moveit': LaunchConfiguration('use_moveit'),
            'world':      LaunchConfiguration('world'),
            'env':        LaunchConfiguration('env'),
        }.items(),
    )

    return LaunchDescription([
        declare_gz,
        declare_use_rviz,
        declare_use_moveit,
        declare_world,
        declare_env,
        include_robot,
    ])
