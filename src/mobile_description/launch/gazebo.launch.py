"""
Gazebo Harmonic Launch File
===========================
Brings up Gazebo Harmonic with the selected environment, spawns the robot,
bridges Gazebo <-> ROS topics, and starts ros2_control controllers.

This file is the simulation backbone — it is included by robot.launch.py.

Behavior:
  - env_name resolves automatically to:
        config/env/<env_name>/worlds/*.world   (first .world file found)
        config/env/<env_name>/models/          (added to GZ_SIM_RESOURCE_PATH)
  - GZ_SIM_RESOURCE_PATH walks every env's models/textures so meshes resolve
    correctly across all environments.

Launch arguments:
  env_name : Environment subfolder under config/env/   (default: cpr_office)
  gz       : 'true' to launch Gazebo GUI, 'false' for headless (default: false)

Usage:
  ros2 launch mobile_description gazebo.launch.py
  ros2 launch mobile_description gazebo.launch.py env_name:=office_small gz:=true
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


# =============================================================================
# Helpers
# =============================================================================
def _find_world_file(env_path):
    """Return absolute path to the first *.world file inside <env_path>/worlds/.

    Convention: each environment ships exactly one world file. If none found
    or multiple exist, fall back to the first sorted match (deterministic).
    Returns None if env path is invalid.
    """
    worlds_dir = os.path.join(env_path, 'worlds')
    if not os.path.isdir(worlds_dir):
        return None
    candidates = sorted(
        f for f in os.listdir(worlds_dir)
        if f.endswith('.world') or f.endswith('.sdf')
    )
    if not candidates:
        return None
    return os.path.join(worlds_dir, candidates[0])


def _build_resource_path(env_root):
    """Walk every env subfolder and collect dirs Gazebo needs to search.

    Required for:
      - model:// URI resolution (models/ root + each models/<name>/)
      - bare texture filenames embedded in .dae meshes (texture dirs)
    """
    dirs = []
    for sub in sorted(os.listdir(env_root)):
        env_path = os.path.join(env_root, sub)
        if not os.path.isdir(env_path):
            continue
        for dirpath, _, filenames in os.walk(env_path):
            if filenames:
                dirs.append(dirpath)
            if os.path.basename(dirpath) == 'models':
                dirs.append(dirpath)

    # Deduplicate, preserve order
    seen, unique = set(), []
    for d in dirs:
        if d not in seen:
            seen.add(d)
            unique.append(d)
    return ':'.join(unique)


# =============================================================================
# Launch entry
# =============================================================================
def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_description')
    env_root  = os.path.join(pkg_share, 'config', 'env')

    declare_env_name = DeclareLaunchArgument(
        'env_name',
        default_value='cpr_office',
        description=(
            'Environment to load. Subfolder under config/env/. Options: '
            'cpr_office, cpr_office_construction, office_small, '
            'office_env_large, office_earthquake'
        ),
    )
    declare_gz_gui = DeclareLaunchArgument(
        'gz',
        default_value='false',
        description='Launch Gazebo GUI (true) or run headless (false)',
    )

    def launch_setup(context, *args, **kwargs):
        env_name = LaunchConfiguration('env_name').perform(context)
        gz_gui   = LaunchConfiguration('gz').perform(context).lower() == 'true'

        env_path   = os.path.join(env_root, env_name)
        world_file = _find_world_file(env_path)
        if world_file is None:
            world_file = 'empty.sdf'  # graceful fallback

        # gz_args: -r runs physics on start, -s headless server, --gui-config off
        gz_flags = '-r'
        if not gz_gui:
            gz_flags += ' -s --headless-rendering'
        gz_args_str = f'{gz_flags} {world_file} --render-engine ogre'

        gz_resource_path = _build_resource_path(env_root)

        return [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

            # 1. Robot State Publisher (sim time on)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'rsp.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),

            # 2. Gazebo Harmonic
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )),
                launch_arguments={'gz_args': gz_args_str}.items(),
            ),

            # 3. Spawn robot at world origin (z=0.1 to avoid ground clipping)
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

            # 4. ros_gz_bridge — clock + lidar (config-file form is most reliable)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{
                    'config_file': os.path.join(pkg_share, 'config', 'robot', 'ros_gz_bridge.yaml'),
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }],
                output='screen',
            ),

            # 5a. Joint State Broadcaster (publishes /joint_states from wheel encoders)
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad'],
                output='screen',
            ),

            # 5b. Differential Drive Controller (consumes /diff_cont/cmd_vel)
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont'],
                output='screen',
            ),
        ]

    return LaunchDescription([
        declare_env_name,
        declare_gz_gui,
        OpaqueFunction(function=launch_setup),
    ])
