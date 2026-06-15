#!/usr/bin/env python3
"""
Arm Planner Node — MoveIt 2 Python Interface
============================================
High-level motion planning interface for the 5-DOF arm.
Communicates with the move_group node via moveit_msgs action servers.

Capabilities
------------
- Plan and execute joint-space goals
- Plan and execute Cartesian pose goals (IK via move_group)
- Plan Cartesian path (straight-line tool motion)
- Named pose execution (home, ready, stow, pre_grasp)
- Gripper open / close

Topics subscribed (Phase 3 AI hooks)
--------------------------------------
  /target_pose   (geometry_msgs/PoseStamped) — AI vision sends EE goal here
  /task_goal     (std_msgs/String)           — text command: "pick", "place", "home"

Topics published
-----------------
  /arm_planner/status  (std_msgs/String) — current planner state

Action clients
--------------
  /arm_controller/follow_joint_trajectory      (control_msgs/FollowJointTrajectory)
  /gripper_controller/follow_joint_trajectory  (control_msgs/FollowJointTrajectory)
  /move_group  (moveit_msgs/MoveGroup) via MoveGroupInterface

Usage
-----
    ros2 run mobile_manipulator arm_planner.py

Then send a named pose:
    ros2 topic pub /task_goal std_msgs/String "data: home" --once

Send a pose goal:
    ros2 topic pub /target_pose geometry_msgs/PoseStamped \\
      "{header: {frame_id: arm_base_link},
        pose: {position: {x: 0.3, y: 0, z: 0.3},
               orientation: {x: 0, y: 0.707, z: 0, w: 0.707}}}" --once
"""

import math
import os
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# Constants — single source of truth lives in the rclpy-free arm_joints module
# so arm_planner and arm_teleop never duplicate joint names (Req 13.1 parity).
# Re-exported here so existing imports (`from arm_planner import ARM_JOINTS`)
# keep working.
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_joints import ARM_JOINTS, GRIPPER_JOINTS, JOINT_LIMITS  # noqa: E402,F401

# Named poses — joint angles in radians
NAMED_POSES = {
    'home':      [0.0, -1.5708, 0.0, 0.0, 0.0],
    'ready':     [0.0, -0.7854, -1.5708, -0.7854, 0.0],
    'stow':      [0.0, -2.6180, -0.5236, 0.0, 0.0],
    'pre_grasp': [0.0, -0.3491, -1.2217, -1.5708, 0.0],
}

GRIPPER_OPEN   = [0.04, 0.04]
GRIPPER_CLOSED = [0.00, 0.00]

# Default trajectory duration multipliers
ARM_DEFAULT_DURATION    = 3.0  # seconds for a full-range motion
GRIPPER_DEFAULT_DURATION = 1.5


# ---------------------------------------------------------------------------
# Utility: build a single-point JointTrajectory message
# ---------------------------------------------------------------------------

def _make_trajectory(
    joints: List[str],
    positions: List[float],
    duration_sec: float,
    velocities: Optional[List[float]] = None,
) -> JointTrajectory:
    """Build a JointTrajectory with a single waypoint at ``duration_sec``."""
    traj = JointTrajectory()
    traj.joint_names = joints

    point = JointTrajectoryPoint()
    point.positions = [float(p) for p in positions]
    point.velocities = velocities if velocities else [0.0] * len(joints)
    point.time_from_start = DurationMsg(
        sec=int(duration_sec),
        nanosec=int((duration_sec % 1) * 1e9),
    )
    traj.points = [point]
    return traj


def _duration_from_current(
    current: List[float],
    target: List[float],
    max_vel: float = 1.5,
    padding: float = 0.5,
) -> float:
    """Estimate required duration from max joint displacement."""
    if not current:
        return ARM_DEFAULT_DURATION
    max_disp = max(abs(t - c) for c, t in zip(current, target))
    return max(max_disp / max_vel + padding, 0.5)


# ---------------------------------------------------------------------------
# Arm Planner Node
# ---------------------------------------------------------------------------

class ArmPlannerNode(Node):
    """Motion planning interface node for the 5-DOF arm and gripper."""

    def __init__(self) -> None:
        super().__init__('arm_planner')

        # ── Action clients ──────────────────────────────────────────────
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

        # ── State ───────────────────────────────────────────────────────
        self._current_arm_positions: List[float] = []
        self._is_busy = False

        # ── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # Phase 3 AI hooks
        self.create_subscription(
            PoseStamped, '/target_pose', self._target_pose_cb, 10
        )
        self.create_subscription(
            String, '/task_goal', self._task_goal_cb, 10
        )

        # ── Publishers ──────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/arm_planner/status', 10)

        self.get_logger().info('ArmPlannerNode ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        """Cache current joint positions from /joint_states."""
        positions = dict(zip(msg.name, msg.position))
        self._current_arm_positions = [
            positions.get(j, 0.0) for j in ARM_JOINTS
        ]

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        """Phase 3: receive EE pose goal from vision pipeline, execute IK plan."""
        if self._is_busy:
            self.get_logger().warn('Arm busy — ignoring /target_pose')
            return

        self.get_logger().info(
            f'/target_pose received: '
            f'x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} '
            f'z={msg.pose.position.z:.3f}'
        )

        # Resolve IK synchronously via the ik.py module
        try:
            import sys, os
            sys.path.insert(0, os.path.dirname(__file__))
            from ik import inverse_kinematics
            import numpy as np

            q = msg.pose.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            rot = np.array([
                [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw),   1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
            ])
            pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            solution, err = inverse_kinematics(pos, rot)
            if solution is not None and err < 0.01:
                self.move_arm_to_joint_goal(solution)
            else:
                self.get_logger().warn(f'IK failed (err={err:.4f})')
        except Exception as e:
            self.get_logger().error(f'IK error: {e}')

    def _task_goal_cb(self, msg: String) -> None:
        """Phase 3: receive text task command."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Task goal received: "{cmd}"')

        if cmd in NAMED_POSES:
            self.move_to_named_pose(cmd)
        elif cmd == 'open_gripper':
            self.open_gripper()
        elif cmd == 'close_gripper':
            self.close_gripper()
        else:
            self.get_logger().warn(
                f'Unknown task goal: "{cmd}". '
                f'Valid: {list(NAMED_POSES.keys()) + ["open_gripper", "close_gripper"]}'
            )

    # ── Public API ─────────────────────────────────────────────────────────

    def move_arm_to_joint_goal(
        self,
        positions: List[float],
        duration_sec: Optional[float] = None,
        wait: bool = True,
    ) -> bool:
        """Send the arm to the given joint positions.

        Parameters
        ----------
        positions   : 5 joint angles in radians
        duration_sec: trajectory duration; auto-computed if None
        wait        : block until the trajectory completes

        Returns
        -------
        True if the goal was accepted and (if wait=True) succeeded.
        """
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller action server not available')
            return False

        if duration_sec is None:
            duration_sec = _duration_from_current(
                self._current_arm_positions, positions
            )

        traj = _make_trajectory(ARM_JOINTS, positions, duration_sec)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = DurationMsg(sec=1, nanosec=0)

        self._is_busy = True
        self._publish_status('moving')

        future = self._arm_client.send_goal_async(goal)
        if wait:
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error('Arm goal rejected')
                self._is_busy = False
                self._publish_status('error')
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration_sec + 5.0
            )
            self._is_busy = False
            self._publish_status('idle')
            return True

        self._is_busy = False
        return True

    def move_to_named_pose(self, name: str, wait: bool = True) -> bool:
        """Move the arm to a predefined named pose."""
        if name not in NAMED_POSES:
            self.get_logger().error(
                f'Unknown named pose: {name}. Available: {list(NAMED_POSES.keys())}'
            )
            return False
        self.get_logger().info(f'Moving arm to named pose: {name}')
        return self.move_arm_to_joint_goal(NAMED_POSES[name], wait=wait)

    def open_gripper(self, wait: bool = True) -> bool:
        """Open the gripper fully (40 mm gap per finger)."""
        return self._send_gripper_goal(GRIPPER_OPEN, wait=wait)

    def close_gripper(self, wait: bool = True) -> bool:
        """Close the gripper fully."""
        return self._send_gripper_goal(GRIPPER_CLOSED, wait=wait)

    def move_cartesian_path(
        self,
        waypoints_joints: List[List[float]],
        step_duration: float = 2.0,
    ) -> bool:
        """Execute a sequence of joint configurations (Cartesian-style).

        Parameters
        ----------
        waypoints_joints : list of joint-angle lists for each waypoint
        step_duration    : time between waypoints [seconds]
        """
        for i, wp in enumerate(waypoints_joints):
            self.get_logger().info(f'Executing waypoint {i+1}/{len(waypoints_joints)}')
            success = self.move_arm_to_joint_goal(wp, duration_sec=step_duration)
            if not success:
                return False
        return True

    # ── Private helpers ────────────────────────────────────────────────────

    def _send_gripper_goal(
        self,
        positions: List[float],
        wait: bool = True,
    ) -> bool:
        """Send a trajectory goal to the gripper controller."""
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('gripper_controller action server not available')
            return False

        traj = _make_trajectory(GRIPPER_JOINTS, positions, GRIPPER_DEFAULT_DURATION)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = DurationMsg(sec=1, nanosec=0)

        future = self._gripper_client.send_goal_async(goal)
        if wait:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error('Gripper goal rejected')
                return False
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=GRIPPER_DEFAULT_DURATION + 2.0
            )
        return True

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
