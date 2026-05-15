"""
RViz Launch File
================
Starts RViz2 with the package's default configuration loaded.

Used standalone (URDF preview) or as a sub-include from robot.launch.py
when the user opts into the RViz UI.

Launch arguments:
  rviz_config  : path to .rviz config file (default: rviz/default.rviz)
  use_sim_time : true when running with Gazebo

Usage:
  ros2 launch mobile_description rviz.launch.py
  ros2 launch mobile_description rviz.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build and return the LaunchDescription for RViz2.

    Resolves the default ``.rviz`` config path, declares the two launch
    arguments (``rviz_config`` and ``use_sim_time``), and starts the
    ``rviz2`` node configured to load the requested config file.

    When used as a sub-launch from ``robot.launch.py``, ``use_sim_time``
    is forwarded as ``true`` so RViz timestamps match Gazebo's /clock.

    Returns
    -------
    LaunchDescription
        Contains two argument declarations and one rviz2 node.
    """
    pkg_share = get_package_share_directory('mobile_description')
    # Default RViz config bundled with the package.
    # Contains pre-configured displays for TF, LaserScan, Map, and the robot
    # model.  Override with rviz_config:=<path> to load a custom layout.
    default_cfg = os.path.join(pkg_share, 'rviz', 'default.rviz')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_cfg,
        description=(
            'Absolute path to the .rviz configuration file. '
            'Defaults to the package-bundled default.rviz.'
        ),
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use /clock topic from Gazebo for time synchronisation. '
            'Set true when running with Gazebo to avoid TF timestamp mismatches.'
        ),
    )

    # rviz2 node — opens the RViz2 GUI window.
    # -d flag tells RViz2 to load the specified .rviz config on startup,
    # restoring all display panels, topic subscriptions, and view settings.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # clock source
        }],
    )

    return LaunchDescription([
        declare_rviz_config,   # must be declared before rviz_node resolves it
        declare_use_sim_time,
        rviz_node,
    ])
