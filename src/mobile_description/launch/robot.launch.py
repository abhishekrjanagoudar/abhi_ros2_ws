"""
Robot Launch Shim
=================
Forwarder so users can run:
    ros2 launch mobile_description robot.launch.py [args]

ros2 launch resolves files from share/<pkg>/launch/ by default. The real
launch description lives at src/robot.launch.py — this shim re-exports it.
"""

import os
import importlib.util

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    real_path = os.path.join(
        get_package_share_directory('mobile_description'),
        'src', 'robot.launch.py',
    )
    spec = importlib.util.spec_from_file_location('_robot_launch_real', real_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
