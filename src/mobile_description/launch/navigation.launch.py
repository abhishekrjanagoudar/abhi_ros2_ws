"""
Navigation Launch File
======================
Brings up the full autonomous navigation stack by composing two sub-launches:

  1. SLAM Toolbox (online_async) — builds a live map from lidar scans and
     broadcasts the ``map -> odom`` TF transform that Nav2 depends on.

  2. Nav2 Bringup (navigation_launch.py) — starts all Nav2 lifecycle nodes:
     amcl (or slam for localisation), bt_navigator, planner_server,
     controller_server, recoveries_server, waypoint_follower, etc.

Both sub-launches share the same ``use_sim_time`` and ``params_file`` values
so the whole stack is time-synchronised and uses a single tuning point.

Prerequisites
-------------
  - A running ``robot_state_publisher`` broadcasting ``base_link`` TF.
  - An active lidar publisher on ``/scan`` (bridged from Gazebo or real HW).
  - ``gazebo.launch.py`` (or equivalent) already up when used in simulation.

Launch arguments
----------------
  use_sim_time : 'true' when Gazebo publishes /clock  (default: true)
  params_file  : absolute path to a Nav2 YAML parameter file
                 (default: config/nav2/nav2_params.yaml)

Usage
-----
  ros2 launch mobile_description navigation.launch.py
  ros2 launch mobile_description navigation.launch.py use_sim_time:=false
  ros2 launch mobile_description navigation.launch.py \\
      params_file:=/path/to/custom_nav2_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # noqa: F401 — available for future nodes


def generate_launch_description():
    """Build and return the LaunchDescription for the navigation stack.

    Resolves package paths, declares launch arguments, constructs the two
    ``IncludeLaunchDescription`` actions (SLAM + Nav2), and assembles them
    into the final ``LaunchDescription``.

    Returns
    -------
    LaunchDescription
        Ordered list of actions:
          - DeclareLaunchArgument × 2  (use_sim_time, params_file)
          - IncludeLaunchDescription   (SLAM Toolbox — async online mapping)
          - IncludeLaunchDescription   (Nav2 full navigation bringup)
    """
    # Resolve install-space share directories for this package and Nav2.
    pkg_share = get_package_share_directory('mobile_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Lazy launch-argument references ──────────────────────────────────────
    # These are NOT evaluated here — they resolve when the launch graph runs.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml'),
    )

    # ── SLAM Toolbox sub-launch ───────────────────────────────────────────────
    # Starts online_async_launch.py from slam_toolbox.  This node:
    #   • Subscribes to /scan  (sensor_msgs/LaserScan on frame ``laser_frame``,
    #                           gpu_lidar: 10 Hz, 360 rays, range 0.3–12.0 m,
    #                           Gaussian noise σ=0.01 m)
    #   • Publishes /map (nav_msgs/OccupancyGrid) as the space is explored.
    #   • Broadcasts the map -> odom TF transform that Nav2 depends on.
    #   • Exposes /slam_toolbox/save_map to serialise the map to disk.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            # Synchronise timestamps with Gazebo when simulating.
            'use_sim_time': use_sim_time,
            # SLAM tuning knobs: scan matcher weights, resolution, etc.
            'slam_params_file': os.path.join(
                pkg_share, 'config', 'nav2', 'mapper_params_online_async.yaml'
            ),
        }.items(),
    )

    # ── Nav2 full navigation sub-launch ──────────────────────────────────────
    # Starts the Nav2 lifecycle manager and all navigation servers.
    # Key servers launched by navigation_launch.py:
    #   • planner_server    — global path planner (NavFn, Smac, etc.)
    #   • controller_server — local trajectory follower (DWB, MPPI, etc.)
    #   • bt_navigator      — behaviour-tree-based goal execution
    #   • recoveries_server — spin, backup, wait recovery behaviours
    #   • waypoint_follower — sequential waypoint execution
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            # Must match SLAM and robot_state_publisher sim-time setting.
            'use_sim_time': use_sim_time,
            # Single YAML file configuring all Nav2 nodes (speeds, footprints,
            # cost-map layers, planner/controller algorithms, etc.).
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        # ── Argument declarations (must precede any use of LaunchConfiguration)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description=(
                'Use /clock topic from Gazebo for time synchronisation. '
                'Set false for real-hardware deployment.'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'nav2', 'nav2_params.yaml'
            ),
            description=(
                'Absolute path to the Nav2 parameter YAML file. '
                'Override to switch between tuned parameter sets.'
            ),
        ),

        # ── Sub-launch actions (order matters: SLAM must be ready before Nav2
        #    tries to read the /map topic and map->odom transform).
        slam_toolbox_launch,
        nav2_launch,
    ])
