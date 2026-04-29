"""
Gazebo Simulation Launch File (ROS 2 Jazzy + Gazebo Harmonic)
=============================================================
Brings up the robot inside Gazebo Harmonic.

Steps:
  1. Include rsp.launch.py with use_sim_time=true, use_gui=false
     (Gazebo controls joints; the slider GUI is for RViz only)
  2. Start Gazebo Harmonic with an empty world
     (--render-engine ogre forces Ogre1, more compatible than default Ogre2)
  3. Spawn the robot from the /robot_description topic published by RSP

Usage:
  ros2 launch mobile_description sim.launch.py

Tip: in Gazebo, press Play (bottom-left) to start physics. Use F to focus camera.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'mobile_description'

    # 1. Robot State Publisher (sim time on, GUI off — Gazebo drives the joints)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )),
        launch_arguments={
            'use_sim_time': 'true',
            'use_gui': 'false',
        }.items(),
    )

    # 2. Gazebo Harmonic with empty world (-r = run physics on start)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={'gz_args': '-r empty.sdf --render-engine ogre'}.items(),
    )

    # 3. Spawn robot at world origin (z slightly above ground to avoid clipping)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen',
    )

    # 4. Bridge Gazebo clock → ROS /clock (required for use_sim_time to work)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 5. Start Joint State Broadcaster (reads wheel positions from Gazebo, publishes /joint_states)
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen',
    )

    # 5. Start Diff Drive Controller (listens to /cmd_vel, outputs wheel velocity commands)
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen',
    )

    return LaunchDescription([
        rsp_launch,
        gazebo_launch,
        spawn_entity,
        clock_bridge,
        joint_broad_spawner,
        diff_cont_spawner,
    ])
