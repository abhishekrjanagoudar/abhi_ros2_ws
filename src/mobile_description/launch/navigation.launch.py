# navigation.launch.py
# =====================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', 
                                    default=os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml'))
    
    # SLAM Toolbox Launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_share, 'config', 'nav2', 'mapper_params_online_async.yaml')
        }.items()
    )

    # Nav2 Bringup Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml')),
        
        slam_toolbox_launch,
        nav2_launch
    ])
