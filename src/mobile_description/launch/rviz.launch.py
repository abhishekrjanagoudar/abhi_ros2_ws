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
    pkg_share = get_package_share_directory('mobile_description')
    default_cfg = os.path.join(pkg_share, 'rviz', 'default.rviz')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_cfg,
        description='Absolute path to the .rviz configuration file',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true when Gazebo publishes /clock)',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_use_sim_time,
        rviz_node,
    ])
