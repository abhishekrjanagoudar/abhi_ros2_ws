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
    """Build and return the LaunchDescription for the Robot State Publisher.

    Steps
    -----
    1. Resolve the path to the robot's xacro file inside the installed package.
    2. Wrap the ``xacro`` CLI call in a ``Command`` substitution so the URDF
       is compiled *at launch time* (picks up any environment-variable macros).
    3. Declare the ``use_sim_time`` argument.
    4. Start ``robot_state_publisher`` with the compiled URDF and publish the
       static and dynamic TF frames derived from it.

    TF frames published
    -------------------
    Static (from URDF fixed joints):
        base_link  -> chassis          (chassis_joint)
        chassis    -> caster_wheel     (caster_wheel_joint)
        chassis    -> laser_frame      (laser_joint — LiDAR puck on top of chassis)
    Dynamic (from /joint_states — driven by ros2_control in simulation):
        base_link  -> left_wheel       (left_wheel_joint,  continuous)
        base_link  -> right_wheel      (right_wheel_joint, continuous)

    Returns
    -------
    LaunchDescription
        Contains one argument declaration and one RSP node.
    """
    pkg_name = 'mobile_description'
    # Relative path inside the installed share directory to the xacro source.
    xacro_subpath = 'config/robot/mobile_robot.urdf.xacro'

    # ── Locate + compile xacro -> URDF string at launch time ─────────────────
    # get_package_share_directory resolves to the *installed* package path
    # (inside install/), not the source tree.
    xacro_file = os.path.join(get_package_share_directory(pkg_name), xacro_subpath)

    # Command(['xacro ', xacro_file]) runs `xacro <path>` as a subprocess and
    # captures stdout as the robot_description string parameter.
    # ParameterValue wraps it so robot_state_publisher receives it as a string.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use /clock from Gazebo for time synchronisation. '
            'Must be true when any Gazebo node is running to keep TF stamps '
            'consistent with simulation time.'
        ),
    )

    # ── robot_state_publisher node ────────────────────────────────────────────
    # Responsibilities:
    #   • Parses robot_description URDF and resolves the kinematic tree.
    #   • Publishes static TF transforms for fixed joints immediately.
    #   • Listens on /joint_states and re-publishes dynamic TF transforms
    #     (e.g. wheel rotations) at the joint_states publish rate.
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,       # compiled URDF XML
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # clock source
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_rsp,
    ])
