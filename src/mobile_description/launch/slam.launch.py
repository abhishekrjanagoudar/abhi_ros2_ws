"""
SLAM Toolbox Launch File
========================
Starts a SLAM Toolbox node in one of four operational modes, driven by the
``mode`` launch argument.  The correct launch file and parameter YAML are
selected at runtime inside an OpaqueFunction so that string resolution
happens after the launch context is fully populated.

Supported modes
---------------
  async        -- online_async_launch.py  (default, recommended for real-time)
  sync         -- online_sync_launch.py   (processes every scan; slower)
  localization -- localization_launch.py  (localise against an existing map)
  lifelong     -- lifelong_launch.py      (continuous lifelong mapping)

Prerequisites (must already be running)
----------------------------------------
  /scan topic  : sensor_msgs/LaserScan published on frame ``laser_frame``
                 (gpu_lidar sensor at 10 Hz, 360 rays, 0.3–12.0 m range)
  TF tree      : map -> odom -> base_link, with base_link -> laser_frame
                 available before SLAM starts processing scans.

Launch arguments
----------------
  use_sim_time : 'true' when Gazebo publishes /clock (default: true)
  mode         : SLAM mode string — async | sync | localization | lifelong
                 (default: async)

Usage
-----
  ros2 launch mobile_description slam.launch.py
  ros2 launch mobile_description slam.launch.py mode:=sync
  ros2 launch mobile_description slam.launch.py mode:=localization use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build and return the LaunchDescription for SLAM Toolbox.

    Steps
    -----
    1. Resolve the ``mobile_description`` and ``slam_toolbox`` install paths.
    2. Declare ``use_sim_time`` and ``mode`` launch arguments with defaults.
    3. Register an OpaqueFunction (``launch_setup``) that runs *after* the
       launch context is ready so the mode string can be read and used to
       pick the correct sub-launch file and YAML params file.

    Returns
    -------
    LaunchDescription
        Contains the two argument declarations and the deferred setup function.
    """
    pkg_share = get_package_share_directory('mobile_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # LaunchConfiguration objects are lazy — they resolve only when the
    # launch context is active (inside OpaqueFunction or at execution time).
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mode = LaunchConfiguration('mode', default='async')

    def launch_setup(context, *args, **kwargs):
        """Select and include the appropriate SLAM Toolbox sub-launch.

        Called by OpaqueFunction after the launch context is initialised,
        which means ``mode.perform(context)`` returns a concrete string.

        Parameters
        ----------
        context : LaunchContext
            Populated launch context; used to evaluate ``LaunchConfiguration``
            substitutions into plain strings.

        Returns
        -------
        list[Action]
            A single-element list containing the ``IncludeLaunchDescription``
            for the chosen SLAM Toolbox variant.
        """
        # Resolve mode to a plain string so we can branch on its value.
        slam_mode = mode.perform(context)

        # Map the mode string to the corresponding slam_toolbox launch file.
        # 'async' is the default — processes scans as fast as they arrive.
        launch_file = 'online_async_launch.py'
        if slam_mode == 'sync':
            # Sync mode processes every scan sequentially; useful for post-
            # processing or when timing accuracy is more important than speed.
            launch_file = 'online_sync_launch.py'
        elif slam_mode == 'localization':
            # Localization-only mode: loads a pre-built map and localises
            # the robot within it (no new mapping is performed).
            launch_file = 'localization_launch.py'
        elif slam_mode == 'lifelong':
            # Lifelong mapping: continuously updates an existing map as the
            # environment changes over time.
            launch_file = 'lifelong_launch.py'

        # Prefer a mode-specific params file; fall back to async params when
        # the requested variant hasn't been tuned yet.
        params_file = os.path.join(
            pkg_share, 'config', 'nav2', f'mapper_params_online_{slam_mode}.yaml'
        )
        if not os.path.exists(params_file):
            # Graceful fallback: async params work reasonably well for all modes.
            params_file = os.path.join(
                pkg_share, 'config', 'nav2', 'mapper_params_online_async.yaml'
            )

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_dir, 'launch', launch_file)
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,   # propagate sim-time flag
                    'slam_params_file': params_file, # tuning parameters for SLAM
                }.items(),
            )
        ]

    return LaunchDescription([
        # Declare use_sim_time — must match Gazebo's /clock when simulating.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description=(
                'Use /clock topic from Gazebo for time synchronisation. '
                'Set false for real robot deployment.'
            ),
        ),
        # Declare mode — controls which SLAM Toolbox variant is launched.
        DeclareLaunchArgument(
            'mode',
            default_value='async',
            description='SLAM mode: async | sync | localization | lifelong',
        ),
        # OpaqueFunction defers launch_setup until the context is ready,
        # allowing us to branch on runtime launch argument values.
        OpaqueFunction(function=launch_setup),
    ])
