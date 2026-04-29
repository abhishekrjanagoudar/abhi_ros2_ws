"""
Robot State Publisher Launch File
=================================
Loads the URDF (compiled from xacro) and starts robot_state_publisher,
which broadcasts the TF tree so RViz/Gazebo know where each link is.

Steps:
  1. Locate xacro file in the mobile_description package
  2. Compile xacro -> URDF XML at launch time (Command substitution)
  3. Start robot_state_publisher with the URDF as 'robot_description'
  4. Optionally start joint_state_publisher_gui (sliders for RViz only)

Launch arguments:
  use_sim_time : true when running with Gazebo (uses /clock topic)
  use_gui      : true to spawn the joint slider GUI (RViz only, disable for Gazebo)

Usage:
  ros2 launch mobile_description rsp.launch.py            # standalone (RViz)
  ros2 launch mobile_description rsp.launch.py use_gui:=false use_sim_time:=true
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
    xacro_subpath = 'urdf/mobile_robot.urdf.xacro'

    # Compile xacro -> URDF string at launch time
    xacro_file = os.path.join(get_package_share_directory(pkg_name), xacro_subpath)
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time (true when Gazebo is running)'
    )
    declare_use_gui = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Start joint_state_publisher_gui (RViz sliders); disable for Gazebo'
    )

    # robot_state_publisher: broadcasts TF tree from URDF + joint states
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # joint_state_publisher_gui: interactive joint sliders (only useful in RViz, not Gazebo)
    # NOTE: Disabled in sim.launch.py because gazebo_ros2_control publishes /joint_states.
    # Running both causes conflicts (TF tree jitters). Enable only for RViz standalone.
    # node_jsp_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_gui')),
    # )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_gui,
        node_rsp,
        # node_jsp_gui,  # Disabled: conflicts with gazebo_ros2_control joint_states publisher
    ])
