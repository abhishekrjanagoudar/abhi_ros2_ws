"""
Gazebo Simulation Launch File (ROS 2 Jazzy + Gazebo Harmonic)
=============================================================
Brings up the robot inside Gazebo Harmonic using the new env/ layout.

Environment structure (config/env/<env_name>/):
  models/   — Gazebo model library for this environment
  worlds/   — World SDF/world files for this environment

Available environments:
  cpr_office              — Clearpath office (DAE mesh)
  cpr_office_construction — CPR office under construction
  office_small            — Small office with furniture
  office_env_large        — Large open-plan office (wall-based SDF)
  office_earthquake       — Earthquake scenario (uses Gazebo cloud models)

Usage:
  ros2 launch mobile_description sim.launch.py
  ros2 launch mobile_description sim.launch.py env:=cpr_office world:=office_cpr.world
  ros2 launch mobile_description sim.launch.py env:=office_small world:=office_small.world

Tip: in Gazebo, press Play (bottom-left) to start physics. Use F to focus camera.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'mobile_description'
    pkg_share = get_package_share_directory(pkg_name)
    env_root  = os.path.join(pkg_share, 'config', 'env')

    # ── Launch arguments ────────────────────────────────────────────────────
    declare_env = DeclareLaunchArgument(
        'env',
        default_value='cpr_office',
        description=(
            'Environment to load. Must match a subfolder name inside config/env/. '
            'Options: cpr_office, cpr_office_construction, office_small, '
            'office_env_large, office_earthquake'
        ),
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description=(
            'World file name inside the selected env/worlds/ folder '
            '(e.g. office_cpr.world). Leave empty to use empty.sdf.'
        ),
    )

    # ── OpaqueFunction: resolve paths at launch time ─────────────────────────
    # We need the actual env string value (not a Substitution) to build fs paths,
    # so we use OpaqueFunction which receives the resolved context.
    def launch_setup(context, *args, **kwargs):
        env_name   = LaunchConfiguration('env').perform(context)
        world_name = LaunchConfiguration('world').perform(context)

        models_path = os.path.join(env_root, env_name, 'models')
        worlds_path = os.path.join(env_root, env_name, 'worlds')

        # Build the gz_args world string
        if world_name:
            world_full = os.path.join(worlds_path, world_name)
            if os.path.isfile(world_full):
                gz_world = world_full          # absolute path for custom worlds
            else:
                gz_world = world_name          # let Gazebo resolve built-in worlds
        else:
            gz_world = 'empty.sdf'

        gz_args_str = f'-r {gz_world} --render-engine ogre'

        # GZ_SIM_RESOURCE_PATH must include:
        #   1. Every models/ dir  → resolves  model://xxx  URIs in world files
        #   2. Every textures/ dir → resolves  bare filenames (e.g. cp_walls.jpg)
        #      embedded inside .dae Collada meshes via <init_from> elements.
        #      Gazebo Harmonic searches GZ_SIM_RESOURCE_PATH for these bare names.
        resource_dirs = []

        for subenv in os.listdir(env_root):
            env_path = os.path.join(env_root, subenv)
            if not os.path.isdir(env_path):
                continue
            # Walk every subdirectory inside this environment and collect dirs
            # that Gazebo needs to search (models root + all nested dirs)
            for dirpath, dirnames, filenames in os.walk(env_path):
                # Add every directory that contains files (textures, scripts…)
                # AND every models/<model_name>/ dir for model:// resolution
                if filenames or 'model.config' in (filenames or []):
                    resource_dirs.append(dirpath)
                # Always add the models/ root itself for model:// resolution
                if os.path.basename(dirpath) == 'models':
                    resource_dirs.append(dirpath)

        # Deduplicate while preserving order
        seen = set()
        unique_dirs = []
        for d in resource_dirs:
            if d not in seen:
                seen.add(d)
                unique_dirs.append(d)

        gz_resource_path = ':'.join(unique_dirs)

        return [
            # Set resource path BEFORE Gazebo starts
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

            # 1. Robot State Publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'rsp.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'use_gui':      'false',
                }.items(),
            ),

            # 2. Gazebo Harmonic
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )),
                launch_arguments={'gz_args': gz_args_str}.items(),
            ),

            # 3. Spawn robot at world origin
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name',  'mobile_robot',
                    '-x', '0.0', '-y', '0.0', '-z', '0.1',
                ],
                output='screen',
            ),

            # 4a. Bridge: Gazebo clock → ROS /clock
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            ),

            # 4b. Bridge: LiDAR scan
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                output='screen',
            ),

            # 5a. Joint State Broadcaster
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad'],
                output='screen',
            ),

            # 5b. Differential Drive Controller
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont'],
                output='screen',
            ),
        ]

    return LaunchDescription([
        declare_env,
        declare_world,
        OpaqueFunction(function=launch_setup),
    ])
