"""
Robot Launch File — mobile_manipulator  (CANONICAL SINGLE ENTRY POINT)
======================================================================
One-command launcher that brings up the complete integrated mobile
manipulator (AGV base + 5-DOF arm + 2-finger gripper) as ONE robot:

  1. Gazebo Harmonic with the selected world (default pick_place.world)
  2. AGV base (mobile_description ros2_control, diff_cont, joint_broad)
  3. Combined URDF in robot_state_publisher (AGV + arm + gripper)
  4. Arm controller spawner (arm_controller, gripper_controller)
  5. MoveIt 2 move_group node
  6. RViz2 with manipulator configuration

This file is the canonical top-level launcher (Req 1). It promotes the
proven `simulation.launch.py` content with documented defaults. A single
combined URDF feeds BOTH robot_state_publisher and `ros_gz_sim create`,
guaranteeing exactly one RSP and one spawned Gazebo model (Req 1.2, 3.4,
5.1, 10.3). The spawn uses `-string` so the compiled URDF is passed
directly (avoids the RSP-volatile / create-transient_local QoS mismatch).

Launch sequence (delays handle startup ordering):
  t=0.0s : Gazebo starts, RSP (combined URDF) starts, env vars set
  t=1.0s : spawn ONE integrated robot via ros_gz_sim create -string
  t=1.5s : ros_gz_bridge (/clock, /scan)
  t=3.0s : joint_broad + diff_cont spawners
  t=4.0s : arm_controller spawner
  t=4.5s : gripper_controller spawner
  t=9.0s : RViz2 (if use_rviz)
  t=10.0s: move_group (if use_moveit)

The existing mobile_description package is NOT modified.

Launch arguments (all documented, with defaults):
  gz        : 'true' to open the Gazebo GUI (default: true). 'false' ⇒
              headless server (-s --headless-rendering).
  use_rviz  : 'true' to open RViz2 (default: true).
  use_moveit: 'true' to start move_group (default: true).
  world     : world name selecting <name>.world under worlds/ (default:
              pick_place). May also be an absolute path to a .world file.
  env       : AGV environment model path for GZ_SIM_RESOURCE_PATH
              (default: cpr_office).

Usage:
  # Full system with documented defaults (GUI + RViz + MoveIt, pick_place):
  ros2 launch mobile_manipulator robot.launch.py

  # Headless Gazebo:
  ros2 launch mobile_manipulator robot.launch.py gz:=false

  # Without MoveIt (arm only, no planning):
  ros2 launch mobile_manipulator robot.launch.py use_moveit:=false
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the full integrated-robot LaunchDescription."""

    # ── Declare arguments (documented defaults) ───────────────────────────
    declare_gz         = DeclareLaunchArgument('gz',         default_value='true')
    declare_use_rviz   = DeclareLaunchArgument('use_rviz',   default_value='true')
    declare_use_moveit = DeclareLaunchArgument('use_moveit', default_value='true')
    declare_world      = DeclareLaunchArgument('world',      default_value='pick_place')
    declare_env        = DeclareLaunchArgument('env',        default_value='cpr_office')

    def launch_setup(context, *args, **kwargs):
        use_rviz   = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
        use_moveit = LaunchConfiguration('use_moveit').perform(context).lower() == 'true'
        gz_gui     = LaunchConfiguration('gz').perform(context)
        world_name = LaunchConfiguration('world').perform(context)
        env_name   = LaunchConfiguration('env').perform(context)

        pkg_mm_share  = get_package_share_directory('mobile_manipulator')
        pkg_agv_share = get_package_share_directory('mobile_description')

        arm_controllers_yaml = os.path.join(
            pkg_mm_share, 'config', 'arm_controllers.yaml'
        )

        # ── Compile combined URDF at launch time ─────────────────────────
        # ONE combined URDF string feeds BOTH robot_state_publisher and the
        # ros_gz_sim `create` spawn → exactly one RSP, one spawned model.
        xacro_file = os.path.join(
            pkg_mm_share, 'urdf', 'mobile_manipulator.urdf.xacro'
        )
        combined_urdf_str = subprocess.check_output(
            ['xacro', xacro_file], text=True
        )

        # ── Compute Gazebo world path ────────────────────────────────────
        # `world` is a name selecting <name>.world under worlds/, or may be
        # an absolute path to a .world file (graceful override).
        if os.path.isabs(world_name):
            world_file = world_name
        else:
            world_file = os.path.join(
                pkg_mm_share, 'worlds', f'{world_name}.world'
            )
        gz_flags = '-r'
        if gz_gui != 'true':
            gz_flags += ' -s --headless-rendering'
        gz_args = f'{gz_flags} {world_file}'

        # ── Build GZ_SIM_RESOURCE_PATH from AGV environment models ───────
        agv_env_model_path = os.path.join(
            pkg_agv_share, 'config', 'env', env_name, 'models'
        )

        actions = [

            # ── 1. Gazebo Harmonic (selected world) ──────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )),
                launch_arguments={'gz_args': gz_args}.items(),
            ),

            # ── 2. Robot State Publisher — combined URDF (t=0) ───────────
            # This RSP knows about ALL links: AGV + arm + gripper.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': combined_urdf_str,
                    'use_sim_time': True,
                    'publish_frequency': 50.0,
                }],
            ),

            # ── 3. Spawn combined robot into Gazebo (t=1s delay) ─────────
            # ONE integrated model, spawned from the same combined URDF
            # string via `-string` (Req 1.2).
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-string', combined_urdf_str,
                            '-name',   'mobile_manipulator',
                            '-x', '0.0', '-y', '0.0', '-z', '0.1',
                        ],
                        output='screen',
                    ),
                ],
            ),

            # ── 4. ROS-Gazebo bridge (clock + lidar, reuse AGV bridge config) ──
            TimerAction(
                period=1.5,
                actions=[
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        parameters=[{
                            'config_file': os.path.join(
                                pkg_agv_share, 'config', 'robot', 'ros_gz_bridge.yaml'
                            ),
                            'qos_overrides./tf_static.publisher.durability': 'transient_local',
                        }],
                        output='screen',
                    ),
                ],
            ),

            # ── 5. AGV controllers (diff_cont, joint_broad) (t=3s) ───────
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_broad'],
                        output='screen',
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['diff_cont'],
                        output='screen',
                    ),
                ],
            ),

            # ── 6. Arm controller (t=4s) ─────────────────────────────────
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                        output='screen',
                    ),
                ],
            ),

            # ── 7. Gripper controller (t=4.5s) ───────────────────────────
            TimerAction(
                period=4.5,
                actions=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                        output='screen',
                    ),
                ],
            ),
        ]

        # ── 8. MoveIt 2 move_group (t=10s) ───────────────────────────────
        if use_moveit:
            actions.append(
                TimerAction(
                    period=10.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(pkg_mm_share, 'launch', 'moveit.launch.py')
                            ),
                            launch_arguments={'use_sim_time': 'true'}.items(),
                        ),
                    ],
                )
            )

        # ── 9. RViz2 (t=9s) ──────────────────────────────────────────────
        if use_rviz:
            rviz_config = os.path.join(pkg_mm_share, 'rviz', 'mobile_manipulator.rviz')
            actions.append(
                TimerAction(
                    period=9.0,
                    actions=[
                        Node(
                            package='rviz2',
                            executable='rviz2',
                            name='rviz2',
                            output='screen',
                            arguments=['-d', rviz_config],
                            parameters=[{'use_sim_time': True}],
                        ),
                    ],
                )
            )

        return actions

    return LaunchDescription([
        # ── WSL2 software-rendering workaround ────────────────────────────
        # Under WSL2 there is no working hardware GL context, so the gz sim
        # GUI (and RViz) abort with "Failed to create OpenGL context" /
        # "glx: failed to create drisw screen". Forcing the Mesa llvmpipe
        # software rasteriser lets them start. When the GUI crashes, gz tears
        # the sim server down too, which is what previously killed the
        # controller_manager and broke the controller spawners.
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        declare_gz,
        declare_use_rviz,
        declare_use_moveit,
        declare_world,
        declare_env,
        OpaqueFunction(function=launch_setup),
    ])
