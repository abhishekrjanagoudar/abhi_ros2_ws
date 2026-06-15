"""
MoveIt 2 Launch File — mobile_manipulator
==========================================
Starts the move_group node with all MoveIt 2 parameters loaded from
the moveit_config directory.

Parameters loaded:
  robot_description         : combined URDF (from xacro compilation)
  robot_description_semantic: SRDF (planning groups, named states, collisions)
  robot_description_kinematics: kinematics plugin per planning group
  planning_pipelines          : OMPL configuration
  moveit_controller_manager   : maps MoveIt groups to ros2_control servers
  joint_limits               : per-joint vel/acc limits for time parameterisation

What move_group provides:
  Action server: /move_group (moveit_msgs/MoveGroup)
  Topics:        /move_group/monitored_planning_scene
                 /move_group/display_planned_path
  Services:      /compute_cartesian_path
                 /compute_fk  /compute_ik (MoveIt native)
  RViz panel:    MotionPlanning plugin connects here

Launch arguments:
  use_sim_time : true (default) for Gazebo integration

Usage:
  ros2 launch mobile_manipulator moveit.launch.py
  (normally included by simulation.launch.py)
"""

import os
import tempfile
import yaml
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    """Load a YAML file and return its contents as a dict."""
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    """Build the move_group LaunchDescription."""

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo.',
    )

    def launch_setup(context, *args, **kwargs):
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        pkg_mm = get_package_share_directory('mobile_manipulator')

        # ── Compile combined URDF ────────────────────────────────────────
        xacro_file = os.path.join(
            pkg_mm, 'urdf', 'mobile_manipulator.urdf.xacro'
        )
        robot_description_str = subprocess.check_output(
            ['xacro', xacro_file], text=True
        )

        # ── Load SRDF ────────────────────────────────────────────────────
        srdf_file = os.path.join(
            pkg_mm, 'moveit_config', 'srdf', 'mobile_manipulator.srdf'
        )
        with open(srdf_file, 'r') as f:
            robot_description_semantic = f.read()

        # ── Load config YAMLs ────────────────────────────────────────────
        config_dir = os.path.join(pkg_mm, 'moveit_config', 'config')

        kinematics_yaml   = _load_yaml(os.path.join(config_dir, 'kinematics.yaml'))
        joint_limits_yaml = _load_yaml(os.path.join(config_dir, 'joint_limits.yaml'))
        ompl_yaml         = _load_yaml(os.path.join(config_dir, 'ompl_planning.yaml'))
        controllers_yaml  = _load_yaml(os.path.join(config_dir, 'moveit_controllers.yaml'))

        # ── Build a single comprehensive parameter file ──────────────────
        # Writing everything into one temp YAML under move_group.ros__parameters
        # avoids all launch_ros dict-expansion bugs with nested types.
        params = {
            # Core robot model
            'robot_description': robot_description_str,
            'robot_description_semantic': robot_description_semantic,
            'use_sim_time': use_sim_time,

            # Planning pipelines
            'planning_pipelines': ['ompl'],
            'default_planning_pipeline': 'ompl',

            # The 'ompl' sub-namespace: planning_plugin MUST be here
            'ompl': ompl_yaml,

            # Kinematics (per-group IK solver)
            'robot_description_kinematics': kinematics_yaml,

            # Joint limits for time parameterisation
            'robot_description_planning': joint_limits_yaml,

            # Controller manager bindings
            'moveit_controller_manager': controllers_yaml.get(
                'moveit_controller_manager',
                'moveit_simple_controller_manager/MoveItSimpleControllerManager'
            ),
            'moveit_simple_controller_manager': controllers_yaml.get(
                'moveit_simple_controller_manager', {}
            ),
            'arm_controller': controllers_yaml.get('arm_controller', {}),
            'gripper_controller': controllers_yaml.get('gripper_controller', {}),

            # Planning scene monitor
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,

            # Trajectory execution
            'moveit_manage_controllers': True,
            'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.2,
                'allowed_goal_duration_margin': 0.5,
                'allowed_start_tolerance': 0.01,
                'wait_for_trajectory_completion': True,
            },

            # Capabilities
            'capabilities': (
                'move_group/MoveGroupCartesianPathService '
                'move_group/MoveGroupExecuteTrajectoryAction '
                'move_group/MoveGroupKinematicsService '
                'move_group/MoveGroupMoveAction '
                'move_group/MoveGroupPlanService '
                'move_group/MoveGroupQueryPlannersService '
                'move_group/MoveGroupStateValidationService '
                'move_group/MoveGroupGetPlanningSceneService '
                'move_group/ApplyPlanningSceneService '
                'move_group/ClearOctomapService '
            ),
        }

        # Wrap under move_group.ros__parameters for proper ROS 2 namespacing
        full_config = {
            'move_group': {
                'ros__parameters': params
            }
        }

        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False
        )
        yaml.dump(full_config, tmp, default_flow_style=False)
        tmp.close()

        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[tmp.name],
        )

        return [move_group_node]

    return LaunchDescription([
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup),
    ])
