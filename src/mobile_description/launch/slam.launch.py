# slam.launch.py
# ==============
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mode = LaunchConfiguration('mode', default='async') # async, sync, localization, lifelong

    def launch_setup(context, *args, **kwargs):
        slam_mode = mode.perform(context)
        
        launch_file = 'online_async_launch.py'
        if slam_mode == 'sync':
            launch_file = 'online_sync_launch.py'
        elif slam_mode == 'localization':
            launch_file = 'localization_launch.py'
        elif slam_mode == 'lifelong':
            launch_file = 'lifelong_launch.py'
            
        params_file = os.path.join(pkg_share, 'config', 'nav2', f'mapper_params_online_{slam_mode}.yaml')
        # Fallback to async params if specific ones don't exist
        if not os.path.exists(params_file):
            params_file = os.path.join(pkg_share, 'config', 'nav2', 'mapper_params_online_async.yaml')

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', launch_file)),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': params_file
                }.items()
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='async', description='async, sync, localization, lifelong'),
        OpaqueFunction(function=launch_setup)
    ])
