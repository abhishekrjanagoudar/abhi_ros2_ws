"""
Gazebo Harmonic Launch File
===========================
Brings up Gazebo Harmonic with the selected environment, spawns the robot,
bridges Gazebo <-> ROS topics, and starts ros2_control controllers.

This file is the simulation backbone — it is included by robot.launch.py.

Behavior:
  - env resolves automatically to:
        config/env/<env>/worlds/*.world   (first .world file found)
        config/env/<env>/models/          (added to Gazebo resource paths)

Launch arguments:
  env      : Environment subfolder under config/env/  (default: cpr_office)
  env_name : Backward-compatible alias for env
  gz       : 'true' to launch Gazebo GUI, 'false' for headless (default: false)

Usage:
  ros2 launch mobile_description gazebo.launch.py
  ros2 launch mobile_description gazebo.launch.py env:=office_small gz:=true
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
    """Return the absolute path to the first world file in ``<env_path>/worlds/``.

    Searches for files ending in ``.world`` or ``.sdf`` (Gazebo accepts both).
    Results are sorted alphabetically for determinism when multiple files exist.

    Convention
    ----------
    Each environment folder ships exactly one world file.  Sorting ensures the
    same file is always selected regardless of filesystem ordering.

    Parameters
    ----------
    env_path : str
        Absolute path to the environment subfolder, e.g.
        ``<pkg_share>/config/env/cpr_office``.

    Returns
    -------
    str or None
        Absolute path to the chosen world/SDF file, or ``None`` if the
        ``worlds/`` subdirectory is missing or empty.
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


def _build_resource_path(model_path):
    """Build a colon-separated GZ_SIM_RESOURCE_PATH string for Gazebo.

    Walks ``model_path`` recursively and collects every subdirectory that
    contains at least one file.  This ensures Gazebo can locate mesh assets,
    material scripts, and nested model definitions stored inside the
    environment's ``models/`` folder.

    Duplicate entries are removed while preserving insertion order.

    Parameters
    ----------
    model_path : str
        Root directory of the environment's model assets, e.g.
        ``<pkg_share>/config/env/<env>/models``.

    Returns
    -------
    str
        Colon-delimited list of unique absolute directory paths, ready to be
        assigned to ``GZ_SIM_RESOURCE_PATH``.  Empty string if ``model_path``
        does not exist.
    """
    dirs = []
    if os.path.isdir(model_path):
        # Include the root models directory itself first.
        dirs.append(model_path)
        # Walk all subdirectories; only add those that actually contain files
        # (Gazebo needs the *leaf* directories, not empty intermediate ones).
        for dirpath, _, filenames in os.walk(model_path):
            if filenames:
                dirs.append(dirpath)

    # Deduplicate while preserving order (dict trick pre-Python 3.7 equiv).
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
    """Build and return the top-level LaunchDescription for Gazebo Harmonic.

    Declares the three launch arguments, then registers an ``OpaqueFunction``
    (``launch_setup``) that defers the remaining construction until the launch
    context is available.  This pattern is required because Gazebo's ``gz_args``
    string must embed the resolved world-file path, which is only known after
    the ``env`` argument has been evaluated.

    Returns
    -------
    LaunchDescription
        Contains three argument declarations and one OpaqueFunction action.
    """
    pkg_share = get_package_share_directory('mobile_description')
    # Root directory that holds all named environment subfolders.
    env_root = os.path.join(pkg_share, 'config', 'env')

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
    declare_gz_gui = DeclareLaunchArgument(
        'gz',
        default_value='false',
        description='Launch Gazebo GUI (true) or run headless (false)',
    )

    def launch_setup(context, *args, **kwargs):
        """Construct and return all simulation-related actions.

        Called by OpaqueFunction after the launch context is initialised so
        that all ``LaunchConfiguration`` values are fully resolved to strings.

        Build order
        -----------
        1. Set GZ_SIM_RESOURCE_PATH / GAZEBO_MODEL_PATH environment variables
           so Gazebo can locate mesh and model assets.
        2. Launch Robot State Publisher (rsp.launch.py) with sim time enabled.
        3. Launch Gazebo Harmonic (gz_sim.launch.py) with the world file.
        4. Spawn the robot URDF into the running Gazebo world.
        5a. Spawn the Joint State Broadcaster controller.
        5b. Spawn the Differential Drive controller.

        Parameters
        ----------
        context : LaunchContext
            Active launch context; used to resolve ``LaunchConfiguration``
            substitutions to plain Python strings.

        Returns
        -------
        list[Action]
            Ordered list of SetEnvironmentVariable, IncludeLaunchDescription,
            and Node actions.
        """
        # Resolve env name: prefer env_name alias (backward compat) over env.
        env_name_alias = LaunchConfiguration('env_name').perform(context)
        env_name = env_name_alias or LaunchConfiguration('env').perform(context)
        gz_gui = LaunchConfiguration('gz').perform(context).lower() == 'true'

        env_path = os.path.join(env_root, env_name)
        model_path = os.path.join(env_path, 'models')
        world_file = _find_world_file(env_path)
        if world_file is None:
            # Graceful fallback: Gazebo ships a built-in empty world.
            world_file = 'empty.sdf'

        # gz_args flags:
        #   -r  : start physics simulation immediately on launch
        #   -s  : server-only mode (no GUI window)
        #   --headless-rendering : off-screen rendering (no display required)
        gz_flags = '-r'
        if not gz_gui:
            gz_flags += ' -s --headless-rendering'
        gz_args_str = f'{gz_flags} {world_file}'

        # Build the colon-separated resource path for Gazebo model discovery.
        gz_resource_path = _build_resource_path(model_path)

        return [
            # Export resource paths before Gazebo starts so the process inherits them.
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
            SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

            # 1. Robot State Publisher (sim time on)
            #    Must start before Gazebo spawns the robot so TF is ready.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'rsp.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),

            # 2. Gazebo Harmonic
            #    gz_args embeds the world file path and GUI/headless flags.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )),
                launch_arguments={'gz_args': gz_args_str}.items(),
            ),

            # 3. Spawn robot at world origin
            #    z=0.1 keeps the chassis clear of the ground plane:
            #      wheel_zoff (0.05 m) + wheel_radius (0.05 m) = 0.1 m clearance.
            #    Reads URDF from /robot_description published by RSP above.
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

            # 4. ros_gz_bridge — clock + lidar (config-file driven, most reliable)
            #    Bridges Gazebo Harmonic topics to ROS 2:
            #      /clock     (Gazebo -> ROS)  — sim time for all nodes
            #      /scan      (Gazebo -> ROS)  — gpu_lidar on laser_frame, 10 Hz,
            #                                   360 rays, 0.3–12.0 m, Gaussian σ=0.01
            #      /tf        (bidirectional)  — dynamic transforms (wheel joints)
            #      /tf_static (Gazebo -> ROS)  — fixed transforms (chassis, laser_frame)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{
                    'config_file': os.path.join(pkg_share, 'config', 'robot', 'ros_gz_bridge.yaml'),
                    # Ensure /tf_static is latched so late subscribers get transforms.
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }],
                output='screen',
            ),

            # 5a. Joint State Broadcaster  (controller: joint_broad)
            #     Reads velocity + position state interfaces from the
            #     gz_ros2_control/GazeboSimSystem plugin for:
            #       left_wheel_joint  (continuous, ±10 rad/s)
            #       right_wheel_joint (continuous, ±10 rad/s)
            #     Publishes /joint_states so RSP keeps left_wheel / right_wheel TF live.
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad'],
                output='screen',
            ),

            # 5b. Differential Drive Controller  (controller: diff_cont)
            #     Subscribes: /diff_cont/cmd_vel  (geometry_msgs/Twist)
            #     Publishes:  /diff_cont/odom     (nav_msgs/Odometry)
            #                 /tf odom -> base_link transform
            #     Sends velocity commands to left_wheel_joint and right_wheel_joint
            #     via the ros2_control command interface.
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont'],
                output='screen',
            ),
        ]

    return LaunchDescription([
        declare_env,
        declare_env_name_alias,
        declare_gz_gui,
        OpaqueFunction(function=launch_setup),
    ])
