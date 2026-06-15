#!/usr/bin/env python3
"""
Pick-and-Place Demo Node
========================
Orchestrates a full pick-and-place operation using a state machine.
Integrates Nav2 (navigation) + arm_planner.py (arm motion) + gripper.

State Machine
-------------
  INIT
    │  (startup delay, wait for all services)
    ▼
  STOW_ARM
    │  (arm folds for safe navigation)
    ▼
  NAVIGATE_TO_PICK
    │  (Nav2: drive to (0.65, 0.0))
    ▼
  OPEN_GRIPPER
    │  (gripper fully open before approach)
    ▼
  MOVE_PRE_GRASP
    │  (arm to pre_grasp pose: above cube)
    ▼
  MOVE_TO_GRASP
    │  (arm descends to grasp pose)
    ▼
  CLOSE_GRIPPER
    │  (grasp the cube)
    ▼
  LIFT_OBJECT
    │  (arm lifts object clear of table)
    ▼
  NAVIGATE_TO_PLACE
    │  (Nav2: drive to (0.65, 0.20))
    ▼
  MOVE_TO_PLACE
    │  (arm positions object over place marker)
    ▼
  OPEN_GRIPPER (release)
    │
    ▼
  MOVE_HOME
    │
    ▼
  DONE

Phase 3 hooks
-------------
  /detections  (reserved — bounding boxes from vision pipeline)
               When populated, NAVIGATE_TO_PICK uses the detected object
               position instead of the hardcoded (1.2, 0, 0.525).

  /target_pose (reserved — can override grasp pose from external AI)

  /task_goal   (reserved — high-level trigger: "start", "abort", "restart")

Topics published
-----------------
  /task_status  (std_msgs/String) — current state name for monitoring

Usage
-----
    ros2 run mobile_manipulator pick_place_node.py
"""

import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Constants — robot geometry and demo targets
# ---------------------------------------------------------------------------

ARM_JOINTS    = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint']
GRIPPER_JOINTS = ['left_finger_joint', 'right_finger_joint']

# Pre-computed joint angles for key poses [radians]
# These are validated against the FK to reach the table at x=1.2m from x=0.65m
POSE_HOME = [0.0, -1.5708, 0.0, 0.0, 0.0]
POSE_STOW = [0.0, -2.6180, -0.5236, 0.0, 0.0]

# Pre-grasp: arm positioned directly above the cube centre
# shoulder_pan=0 (straight ahead), shoulder_lift=-20°, elbow=-70°, wrist=-90°
POSE_PRE_GRASP = [0.0, -0.3491, -1.2217, -1.5708, 0.0]

# Grasp: arm lowered to cube height (additional -15° on shoulder, -10° elbow)
POSE_GRASP = [0.0, -0.5236, -1.3963, -1.5708, 0.0]

# Lift: after grasping, raise the arm back to pre-grasp height
POSE_LIFT = POSE_PRE_GRASP

# Place above: arm swung slightly to +Y for the place marker at y=0.20m
POSE_PLACE_ABOVE = [0.3491, -0.3491, -1.2217, -1.5708, 0.0]   # pan 20° left
POSE_PLACE       = [0.3491, -0.5236, -1.3963, -1.5708, 0.0]   # lowered

GRIPPER_OPEN   = [0.04, 0.04]
GRIPPER_CLOSED = [0.00, 0.00]

# Nav2 waypoints (map frame, robot heading = 0)
# x=0.65: robot is ~0.55m from table at x=1.2 — arm can reach cube at dx≈0.5m
NAV_PICK_GOAL  = (0.65, 0.00, 0.0)   # (x, y, yaw_rad)
NAV_PLACE_GOAL = (0.65, 0.00, 0.0)   # same location, arm pans to reach place target


# ---------------------------------------------------------------------------
# State machine states
# ---------------------------------------------------------------------------

class State(Enum):
    INIT              = auto()
    STOW_ARM          = auto()
    NAVIGATE_TO_PICK  = auto()
    OPEN_GRIPPER_PRE  = auto()
    MOVE_PRE_GRASP    = auto()
    MOVE_TO_GRASP     = auto()
    CLOSE_GRIPPER     = auto()
    LIFT_OBJECT       = auto()
    NAVIGATE_TO_PLACE = auto()
    MOVE_PLACE_ABOVE  = auto()
    MOVE_TO_PLACE     = auto()
    OPEN_GRIPPER_POST = auto()
    MOVE_HOME         = auto()
    DONE              = auto()
    ERROR             = auto()


# ---------------------------------------------------------------------------
# Helper: build a single-waypoint JointTrajectory goal
# ---------------------------------------------------------------------------

def _make_traj_goal(
    joints: list,
    positions: list,
    duration_sec: float,
) -> FollowJointTrajectory.Goal:
    traj = JointTrajectory()
    traj.joint_names = joints
    pt = JointTrajectoryPoint()
    pt.positions  = [float(p) for p in positions]
    pt.velocities = [0.0] * len(joints)
    pt.time_from_start = DurationMsg(
        sec=int(duration_sec),
        nanosec=int((duration_sec % 1) * 1e9),
    )
    traj.points = [pt]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.goal_time_tolerance = DurationMsg(sec=5, nanosec=0)
    return goal


# ---------------------------------------------------------------------------
# Pick-and-Place Node
# ---------------------------------------------------------------------------

class PickPlaceNode(Node):
    """State-machine orchestrator for the pick-and-place demo."""

    def __init__(self) -> None:
        super().__init__('pick_place_node')

        # ── Action clients ──────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )

        # ── State ───────────────────────────────────────────────────────
        self._state = State.INIT
        self._goal_handle_nav  = None
        self._goal_handle_arm  = None
        self._result_future    = None
        self._step_start_time  = self.get_clock().now()
        self._action_in_flight = False

        # ── Phase 3 hook subscriptions (reserved, no-op until Phase 3) ──
        self._detections_received = False
        self._override_pose: Optional[PoseStamped] = None

        self.create_subscription(
            String, '/detections',
            lambda _: None,   # Phase 3: parse vision bounding boxes here
            10,
        )
        self.create_subscription(
            PoseStamped, '/target_pose',
            self._on_target_pose_override,
            10,
        )
        self.create_subscription(
            String, '/task_goal',
            self._on_task_goal,
            10,
        )

        # ── Status publisher ─────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/task_status', 10)

        # ── Main state machine timer (10 Hz tick) ───────────────────────
        self._timer = self.create_timer(0.1, self._tick)
        self._startup_ticks = 0

        self.get_logger().info('PickPlaceNode started — state: INIT')

    # ── Phase 3 hook callbacks ────────────────────────────────────────────

    def _on_target_pose_override(self, msg: PoseStamped) -> None:
        """Phase 3: allow external AI to override the grasp target pose."""
        self._override_pose = msg
        self.get_logger().info(
            f'[Phase3] /target_pose override received: '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, '
            f'{msg.pose.position.z:.3f})'
        )

    def _on_task_goal(self, msg: String) -> None:
        """Phase 3: high-level task command override."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'[Phase3] /task_goal: "{cmd}"')
        if cmd == 'abort' and self._state not in (State.DONE, State.ERROR):
            self._transition(State.ERROR)
        elif cmd == 'start' and self._state == State.DONE:
            self._transition(State.STOW_ARM)

    # ── Status publisher ──────────────────────────────────────────────────

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._state.name
        self._status_pub.publish(msg)

    def _transition(self, new_state: State) -> None:
        self.get_logger().info(f'{self._state.name} → {new_state.name}')
        self._state = new_state
        self._action_in_flight = False
        self._step_start_time  = self.get_clock().now()
        self._publish_status()

    # ── State machine tick ────────────────────────────────────────────────

    def _tick(self) -> None:
        """Main state machine — called at 10 Hz."""
        if self._action_in_flight:
            self._check_action_done()
            return

        match self._state:
            case State.INIT:
                self._startup_ticks += 1
                if self._startup_ticks > 50:   # 5-second startup delay
                    self._transition(State.STOW_ARM)

            case State.STOW_ARM:
                self._send_arm_goal(POSE_STOW, 5.0)
                self._pending_next = State.NAVIGATE_TO_PICK

            case State.NAVIGATE_TO_PICK:
                self._send_nav_goal(*NAV_PICK_GOAL)
                self._pending_next = State.OPEN_GRIPPER_PRE

            case State.OPEN_GRIPPER_PRE:
                self._send_gripper_goal(GRIPPER_OPEN, 1.5)
                self._pending_next = State.MOVE_PRE_GRASP

            case State.MOVE_PRE_GRASP:
                self._send_arm_goal(POSE_PRE_GRASP, 3.0)
                self._pending_next = State.MOVE_TO_GRASP

            case State.MOVE_TO_GRASP:
                self._send_arm_goal(POSE_GRASP, 2.0)
                self._pending_next = State.CLOSE_GRIPPER

            case State.CLOSE_GRIPPER:
                self._send_gripper_goal(GRIPPER_CLOSED, 1.5)
                self._pending_next = State.LIFT_OBJECT

            case State.LIFT_OBJECT:
                self._send_arm_goal(POSE_LIFT, 2.5)
                self._pending_next = State.NAVIGATE_TO_PLACE

            case State.NAVIGATE_TO_PLACE:
                # Place target is in reach from same robot position (arm pans)
                # so we stay at same nav goal; arm_pan handles the offset
                self._send_arm_goal(POSE_PLACE_ABOVE, 2.0)
                self._pending_next = State.MOVE_TO_PLACE

            case State.MOVE_TO_PLACE:
                self._send_arm_goal(POSE_PLACE, 2.0)
                self._pending_next = State.OPEN_GRIPPER_POST

            case State.OPEN_GRIPPER_POST:
                self._send_gripper_goal(GRIPPER_OPEN, 1.5)
                self._pending_next = State.MOVE_HOME

            case State.MOVE_HOME:
                self._send_arm_goal(POSE_HOME, 5.0)
                self._pending_next = State.DONE

            case State.DONE:
                self.get_logger().info(
                    'Pick-and-place complete! '
                    'Send "/task_goal: start" to repeat.',
                    throttle_duration_sec=10.0,
                )

            case State.ERROR:
                self.get_logger().error(
                    'Pick-and-place failed. Check arm and navigation.',
                    throttle_duration_sec=5.0,
                )

    def _check_action_done(self) -> None:
        """Poll the pending action future and advance state when complete."""
        if self._result_future is None:
            return
        if self._result_future.done():
            self._transition(self._pending_next)

    # ── Action senders ────────────────────────────────────────────────────

    def _send_arm_goal(self, positions: list, duration_sec: float) -> None:
        if not self._arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('arm_controller not available')
            self._transition(State.ERROR)
            return

        goal = _make_traj_goal(ARM_JOINTS, positions, duration_sec)
        send_future = self._arm_client.send_goal_async(goal)
        self._action_in_flight = True
        self._result_future = None

        def _on_goal_response(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error('Arm goal rejected')
                self._transition(State.ERROR)
                return
            self._result_future = gh.get_result_async()

        send_future.add_done_callback(_on_goal_response)

    def _send_gripper_goal(self, positions: list, duration_sec: float) -> None:
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('gripper_controller not available')
            self._transition(State.ERROR)
            return

        goal = _make_traj_goal(GRIPPER_JOINTS, positions, duration_sec)
        send_future = self._gripper_client.send_goal_async(goal)
        self._action_in_flight = True
        self._result_future = None

        def _on_goal_response(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error('Gripper goal rejected')
                self._transition(State.ERROR)
                return
            self._result_future = gh.get_result_async()

        send_future.add_done_callback(_on_goal_response)

    def _send_nav_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            self._transition(State.ERROR)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import math
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        send_future = self._nav_client.send_goal_async(goal)
        self._action_in_flight = True
        self._result_future = None

        def _on_goal_response(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error('Navigation goal rejected')
                self._transition(State.ERROR)
                return
            self._result_future = gh.get_result_async()

        send_future.add_done_callback(_on_goal_response)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
