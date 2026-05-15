"""
Simulation Launch File (Backward-Compatible Wrapper)
====================================================
Thin wrapper around gazebo.launch.py kept for backward compatibility.
New code should call robot.launch.py instead — it provides the full stack
(Gazebo + RViz/NiceGUI orchestration).

This wrapper accepts env (world auto-resolves from env folder).

Launch arguments:
  env      : Environment subfolder under config/env/  (default: cpr_office)
  env_name : Backward-compatible alias for env
  gz       : 'true' to launch Gazebo GUI, 'false' headless (default: false)

Usage:
  ros2 launch mobile_description sim.launch.py
  ros2 launch mobile_description sim.launch.py env:=office_small gz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build and return the LaunchDescription for this backward-compatible wrapper.

    Declares the same three launch arguments as ``gazebo.launch.py`` and
    forwards them verbatim via ``IncludeLaunchDescription``.  No additional
    processing is done here — all simulation logic lives in gazebo.launch.py.

    Returns
    -------
    LaunchDescription
        Three argument declarations followed by a single
        ``IncludeLaunchDescription`` action delegating to gazebo.launch.py.
    """
    # Resolve the installed share directory for this package.
    pkg_share = get_package_share_directory('mobile_description')

    # ── Launch argument declarations ──────────────────────────────────────────
    # Mirror the arguments from gazebo.launch.py so callers get the same
    # interface whether they invoke sim.launch.py or gazebo.launch.py directly.
    declare_env = DeclareLaunchArgument(
        'env',
        default_value='cpr_office',
        description='Environment to load. Subfolder under config/env/.',
    )
    declare_env_name_alias = DeclareLaunchArgument(
        'env_name',
        default_value='',
        description='Deprecated alias for env. Prefer env:=<name>.',
    )
    declare_gz = DeclareLaunchArgument(
        'gz',
        default_value='false',
        description='Enable Gazebo GUI (true) or run headless (false)',
    )

    # ── Delegate to gazebo.launch.py ──────────────────────────────────────────
    # Forward all three arguments so gazebo.launch.py has the full context.
    # Using LaunchConfiguration here keeps the values lazy — they resolve at
    # execution time, not at launch-graph construction time.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'env':      LaunchConfiguration('env'),       # primary env selector
            'env_name': LaunchConfiguration('env_name'),  # legacy alias passthrough
            'gz':       LaunchConfiguration('gz'),         # GUI on/off
        }.items(),
    )

    return LaunchDescription([
        declare_env,
        declare_env_name_alias,
        declare_gz,
        gazebo_launch,   # single action: everything else is in gazebo.launch.py
    ])
