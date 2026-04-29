"""
Top-Level Robot Launch File
===========================
Single entry point that brings up the entire mobile_robot stack:
  - Gazebo simulation with the selected environment
  - Robot spawn + ros2_control controllers
  - Optional RViz visualization
  - Optional Gazebo GUI
  - Optional NiceGUI web UI (limited controls, browser-accessible)

This file orchestrates includes only — heavy lifting lives in:
  launch/gazebo.launch.py   (sim + bridge + controllers)
  launch/rviz.launch.py     (RViz with default config)

Launch arguments:
  env_name : Environment subfolder under config/env/      (default: cpr_office)
  ros_ui   : 'true'  -> launch RViz (full ROS data access, host display required)
             'false' -> launch NiceGUI web UI on localhost (default)
  gz       : 'true' to enable Gazebo GUI, 'false' for headless (default: false)
  rviz     : 'true' to launch RViz independently of ros_ui  (default: false)

Usage:
  ros2 launch mobile_description robot.launch.py
  ros2 launch mobile_description robot.launch.py env_name:=office_small
  ros2 launch mobile_description robot.launch.py env_name:=cpr_office ros_ui:=true gz:=true
  ros2 launch mobile_description robot.launch.py rviz:=true gz:=true

Notes:
  - ros_ui=true forces RViz on (overrides rviz:=false).
  - NiceGUI requires the 'nicegui' Python package (pip install nicegui).
    If not installed, NiceGUI step is skipped silently.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_description')
    launch_dir = os.path.join(pkg_share, 'launch')

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_env_name = DeclareLaunchArgument(
        'env_name',
        default_value='cpr_office',
        description=(
            'Environment to load. Subfolder under config/env/. Options: '
            'cpr_office, cpr_office_construction, office_small, '
            'office_env_large, office_earthquake'
        ),
    )
    declare_ros_ui = DeclareLaunchArgument(
        'ros_ui',
        default_value='True',
        description=(
            'UI selector. false=NiceGUI web UI (default, limited). '
            'true=RViz with full ROS data (host display required).'
        ),
    )
    declare_gz = DeclareLaunchArgument(
        'gz',
        default_value='True',
        description='Enable Gazebo GUI (true) or run headless (false)',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz independently of ros_ui (true/false)',
    )

    # ── Sub-launches ─────────────────────────────────────────────────────────
    # 1. Gazebo backbone (always runs): forwards env_name + gz GUI flag
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'env_name': LaunchConfiguration('env_name'),
            'gz':       LaunchConfiguration('gz'),
        }.items(),
    )

    # 2. RViz: starts when (rviz=true) OR (ros_ui=true)
    rviz_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('rviz'), "'.lower() == 'true' or ",
        "'", LaunchConfiguration('ros_ui'), "'.lower() == 'true'"
    ]))
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=rviz_condition,
    )

    # 3. NiceGUI web UI: starts when ros_ui=false (default)
    #    Wrapped in OpaqueFunction so we can skip gracefully if package missing.
    def maybe_start_nicegui(context, *args, **kwargs):
        ros_ui = LaunchConfiguration('ros_ui').perform(context).lower()
        if ros_ui == 'true':
            return []  # RViz path — skip web UI

        # Skip silently if nicegui not installed (avoids hard dep)
        try:
            import nicegui  # noqa: F401
        except ImportError:
            print('[robot.launch.py] nicegui not installed — skipping web UI. '
                  'Install with: pip install nicegui')
            return []

        # Spawn NiceGUI as a Python process. Expects a script at
        #   src/nicegui_app.py  (user-supplied; absent = noop)
        nicegui_script = os.path.join(pkg_share, 'src', 'nicegui_app.py')
        if not os.path.isfile(nicegui_script):
            print(f'[robot.launch.py] {nicegui_script} not found — skipping web UI.')
            return []

        return [
            Node(
                package='mobile_description',
                executable='python3',
                name='nicegui_web_ui',
                arguments=[nicegui_script],
                output='screen',
            ),
        ]

    return LaunchDescription([
        declare_env_name,
        declare_ros_ui,
        declare_gz,
        declare_rviz,
        gazebo_launch,
        rviz_launch,
        OpaqueFunction(function=maybe_start_nicegui),
    ])
