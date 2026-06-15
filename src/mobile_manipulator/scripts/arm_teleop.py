#!/usr/bin/env python3
"""
Arm Teleop Node — Keyboard Joint-Jogger (Package A)
===================================================
A direct keyboard joint-jogger for the 5-DOF arm and gripper (no MoveIt Servo).

Each keypress jogs a selected joint by a configurable step, clamped to that
joint's URDF limits, and sends a short single-point ``FollowJointTrajectory``
goal to ``/arm_controller/follow_joint_trajectory`` (gripper keys, added in
subtask 5.3, target ``/gripper_controller/follow_joint_trajectory``).

This module is import-safe without a running ROS graph: ``rclpy`` and the other
ROS packages (including ``arm_planner`` and its ``_make_trajectory`` helper) are
imported lazily inside :func:`main` so that the pure helpers
(:func:`clamp_to_limits`) and constants (:data:`JOINT_LIMITS`,
:data:`ARM_JOINTS`, :data:`GRIPPER_JOINTS`) can be imported under pytest
without a ROS runtime. The ``ArmTeleop`` node class is defined inside
:func:`main` after those imports so that ``import arm_teleop`` never requires a
ROS graph (the joint-name parity test in task 2 and the property-based test in
task 6 both import this module).

Usage
-----
    ros2 run mobile_manipulator arm_teleop.py
"""

import os
import select
import sys
import termios
import threading
import time
import tty

# ---------------------------------------------------------------------------
# Joint constants — single source of truth lives in the rclpy-free arm_joints
# module, shared with arm_planner.py (Req 13.1 / joint parity, Req 8.x). This
# import is ROS-free, so it is safe at module top: `import arm_teleop` never
# requires a ROS graph (the joint-name parity test in task 2 and the
# property-based test in task 6 both import this module without ROS sourced).
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_joints import ARM_JOINTS, GRIPPER_JOINTS, JOINT_LIMITS  # noqa: E402
# Teleop tuning constants.
# ---------------------------------------------------------------------------
DEFAULT_JOG_STEP = 0.05      # radians per keypress
MIN_JOG_STEP = 0.01          # smallest selectable step
MAX_JOG_STEP = 0.50          # largest selectable step
STEP_INCREMENT = 0.01        # how much '[' / ']' change the step
JOG_DURATION_SEC = 0.5       # single-point trajectory duration per jog

# Gripper tuning constants. Finger positions are in meters (limits 0.0–0.04),
# so the gripper uses its own small jog step independent of the arm's radian
# step. Open/closed setpoints mirror arm_planner.GRIPPER_OPEN / GRIPPER_CLOSED.
GRIPPER_OPEN_POS = 0.04      # fully open finger position [m]
GRIPPER_CLOSED_POS = 0.0     # fully closed finger position [m]
GRIPPER_JOG_STEP = 0.005     # meters per gripper jog keypress
GRIPPER_DURATION_SEC = 0.5   # single-point trajectory duration per gripper cmd

# ---------------------------------------------------------------------------
# Keyboard bindings (pure data; safe to import without ROS).
# For each arm joint: one key jogs +step, a paired key jogs -step.
#   key -> (arm_joint_index, direction)
# ---------------------------------------------------------------------------
ARM_JOG_BINDINGS = {
    '1': (0, +1), 'q': (0, -1),   # shoulder_pan_joint
    '2': (1, +1), 'w': (1, -1),   # shoulder_lift_joint
    '3': (2, +1), 'e': (2, -1),   # elbow_joint
    '4': (3, +1), 'r': (3, -1),   # wrist_1_joint
    '5': (4, +1), 't': (4, -1),   # wrist_2_joint
}

# Gripper jog bindings: jog BOTH fingers symmetrically by +/- GRIPPER_JOG_STEP.
#   key -> direction (+1 opens, -1 closes)
# Keys are kept distinct from the arm jog/step keys (1/q 2/w 3/e 4/r 5/t, [, ],
# space) and from the gripper open/close keys ('o' / 'c').
GRIPPER_JOG_BINDINGS = {
    '=': +1,   # open fingers by one step
    '-': -1,   # close fingers by one step
}

BANNER = """
╔══════════════════════════════════════════════╗
║ Arm Teleop — Keyboard Joint-Jogger             ║
╠══════════════════════════════════════════════╣
║ 1 / q : shoulder_pan_joint   +/-               ║
║ 2 / w : shoulder_lift_joint  +/-               ║
║ 3 / e : elbow_joint          +/-               ║
║ 4 / r : wrist_1_joint        +/-               ║
║ 5 / t : wrist_2_joint        +/-               ║
║ [ / ] : decrease / increase jog step           ║
║ o / c : open / close gripper (full)            ║
║ = / - : jog gripper open / closed (step)       ║
║ space : resend current target (hold)           ║
║ Ctrl+C: quit                                   ║
╚══════════════════════════════════════════════╝
"""


# ---------------------------------------------------------------------------
# Pure clamp helper (importable without ROS).
# ---------------------------------------------------------------------------

def clamp_to_limits(joint: str, value: float) -> float:
    """Clamp ``value`` into the URDF ``[lower, upper]`` range for ``joint``.

    Parameters
    ----------
    joint : str
        The joint name; must be a key in :data:`JOINT_LIMITS`.
    value : float
        The desired joint position.

    Returns
    -------
    float
        ``value`` clamped to ``[lower, upper]``.

    Raises
    ------
    KeyError
        If ``joint`` is not a known joint in :data:`JOINT_LIMITS`.
    """
    lower, upper = JOINT_LIMITS[joint]
    return max(lower, min(upper, float(value)))


# ---------------------------------------------------------------------------
# Terminal helper: read a single keypress in raw mode (non-blocking).
# Same pattern as base_teleop.py / teleop_controller.py. Stdlib only, so this
# stays import-safe without a ROS runtime.
# ---------------------------------------------------------------------------

def _get_key(timeout: float = 0.1) -> str:
    """Read one keypress from stdin in raw mode, or '' on timeout."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if rlist else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ---------------------------------------------------------------------------
# Main — ROS imports are deferred here to keep the module import-safe.
# The ArmTeleop node class is defined inside main() so that it can reference
# the ROS symbols without requiring a ROS runtime at module import time.
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    from builtin_interfaces.msg import Duration as DurationMsg
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState

    # Reuse the single-point trajectory builder from arm_planner.py (Req 13.1).
    # arm_planner imports rclpy at module top, so this import is deferred here
    # (module top must stay ROS-free). sys.path already includes this dir.
    from arm_planner import _make_trajectory

    class ArmTeleop(Node):
        """Keyboard joint-jogger for the 5-DOF arm + 2-finger gripper (Req 8.1–8.5)."""

        def __init__(
            self,
            jog_step: float = DEFAULT_JOG_STEP,
            jog_duration: float = JOG_DURATION_SEC,
        ) -> None:
            super().__init__('arm_teleop')

            self.jog_step = float(jog_step)
            self.jog_duration = float(jog_duration)

            # Target position vector for the 5 arm joints; seeded from
            # /joint_states on the first complete message, 0.0 until then.
            self._targets = [0.0] * len(ARM_JOINTS)
            self._seeded = False

            # Target position vector for the 2 gripper finger joints; seeded
            # from /joint_states on the first complete message, closed (0.0)
            # until then (Req 8.1).
            self._gripper_targets = [GRIPPER_CLOSED_POS] * len(GRIPPER_JOINTS)
            self._gripper_seeded = False

            self.running = True

            # Action client to the arm controller (Req 8.2). Never base topics.
            self._arm_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/arm_controller/follow_joint_trajectory',
            )

            # Action client to the gripper controller (Req 8.2). Never base
            # topics. Availability is best-effort (see wait_for_server).
            self._gripper_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/gripper_controller/follow_joint_trajectory',
            )

            # Seed / track current arm joint positions (Req 8.1).
            self.create_subscription(
                JointState, '/joint_states', self._js_cb, 10
            )

            self.get_logger().info('ArmTeleop ready.')

        # ── Callbacks ──────────────────────────────────────────────────────

        def _js_cb(self, msg: 'JointState') -> None:
            """Seed arm + gripper targets from the first /joint_states msgs."""
            positions = dict(zip(msg.name, msg.position))

            if not self._seeded:
                seeded = [positions.get(j) for j in ARM_JOINTS]
                if all(p is not None for p in seeded):
                    self._targets = [float(p) for p in seeded]
                    self._seeded = True
                    self.get_logger().info(
                        'Seeded arm targets from /joint_states: '
                        + ', '.join(f'{p:.3f}' for p in self._targets)
                    )

            if not self._gripper_seeded:
                gseeded = [positions.get(j) for j in GRIPPER_JOINTS]
                if all(p is not None for p in gseeded):
                    self._gripper_targets = [
                        clamp_to_limits(j, p)
                        for j, p in zip(GRIPPER_JOINTS, gseeded)
                    ]
                    self._gripper_seeded = True
                    self.get_logger().info(
                        'Seeded gripper targets from /joint_states: '
                        + ', '.join(
                            f'{p:.3f}' for p in self._gripper_targets
                        )
                    )

        # ── Action server handshake ─────────────────────────────────────────

        def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
            """Block until the arm controller action server is available.

            Also waits (best-effort) for the gripper controller server; if the
            gripper server is unavailable we warn but do not fail, so arm-only
            simulations still run.
            """
            self.get_logger().info(
                'Waiting for /arm_controller/follow_joint_trajectory ...'
            )
            ok = self._arm_client.wait_for_server(timeout_sec=timeout_sec)
            if not ok:
                self.get_logger().error(
                    'arm_controller action server not available'
                )

            self.get_logger().info(
                'Waiting for /gripper_controller/follow_joint_trajectory ...'
            )
            gok = self._gripper_client.wait_for_server(timeout_sec=timeout_sec)
            if not gok:
                self.get_logger().warn(
                    'gripper_controller action server not available — '
                    'gripper keys (o/c/=/-) will be inert'
                )
            return ok

        # ── Jogging ──────────────────────────────────────────────────────────

        def jog(self, index: int, direction: int) -> float:
            """Jog arm joint ``index`` by ``direction * jog_step`` (clamped).

            Returns the new (clamped) target for that joint and sends a goal.
            """
            joint = ARM_JOINTS[index]
            desired = self._targets[index] + direction * self.jog_step
            clamped = clamp_to_limits(joint, desired)
            self._targets[index] = clamped
            self._send_arm_goal()
            return clamped

        def open_gripper(self) -> list:
            """Open both fingers fully (clamped to limits) and send a goal."""
            self._gripper_targets = [
                clamp_to_limits(j, GRIPPER_OPEN_POS) for j in GRIPPER_JOINTS
            ]
            self._send_gripper_goal()
            return list(self._gripper_targets)

        def close_gripper(self) -> list:
            """Close both fingers fully (clamped to limits) and send a goal."""
            self._gripper_targets = [
                clamp_to_limits(j, GRIPPER_CLOSED_POS) for j in GRIPPER_JOINTS
            ]
            self._send_gripper_goal()
            return list(self._gripper_targets)

        def jog_gripper(self, direction: int) -> list:
            """Jog both fingers by ``direction * GRIPPER_JOG_STEP`` (clamped).

            ``direction`` is +1 to open, -1 to close. Returns the new
            (clamped) gripper targets and sends a goal.
            """
            self._gripper_targets = [
                clamp_to_limits(j, t + direction * GRIPPER_JOG_STEP)
                for j, t in zip(GRIPPER_JOINTS, self._gripper_targets)
            ]
            self._send_gripper_goal()
            return list(self._gripper_targets)

        def increase_step(self) -> None:
            self.jog_step = round(
                min(MAX_JOG_STEP, self.jog_step + STEP_INCREMENT), 3
            )

        def decrease_step(self) -> None:
            self.jog_step = round(
                max(MIN_JOG_STEP, self.jog_step - STEP_INCREMENT), 3
            )

        def _send_arm_goal(self) -> None:
            """Send a single-point FollowJointTrajectory goal to the arm."""
            traj = _make_trajectory(
                ARM_JOINTS, self._targets, self.jog_duration
            )
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goal.goal_time_tolerance = DurationMsg(sec=1, nanosec=0)
            # Fire-and-forget; rclpy.spin runs in a background thread and
            # processes the goal/result futures. Avoids blocking the keyloop.
            self._arm_client.send_goal_async(goal)

        def _send_gripper_goal(self) -> None:
            """Send a single-point FollowJointTrajectory goal to the gripper."""
            if not self._gripper_client.server_is_ready():
                self.get_logger().warn(
                    'gripper_controller unavailable — gripper command ignored'
                )
                return
            traj = _make_trajectory(
                GRIPPER_JOINTS, self._gripper_targets, GRIPPER_DURATION_SEC
            )
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goal.goal_time_tolerance = DurationMsg(sec=1, nanosec=0)
            self._gripper_client.send_goal_async(goal)

        # ── Interactive loop ─────────────────────────────────────────────────

        def _print_status(self) -> None:
            targets = ' '.join(f'{p:+.3f}' for p in self._targets)
            grip = ' '.join(f'{p:.3f}' for p in self._gripper_targets)
            sys.stdout.write(
                f'\rstep={self.jog_step:.3f} rad | targets: [{targets}] '
                f'| grip: [{grip}]   '
            )
            sys.stdout.flush()

        def run(self) -> None:
            """Read keypresses and jog joints until Ctrl+C."""
            print(BANNER)
            self._print_status()
            try:
                while rclpy.ok() and self.running:
                    key = _get_key()
                    if key in ARM_JOG_BINDINGS:
                        index, direction = ARM_JOG_BINDINGS[key]
                        self.jog(index, direction)
                        self._print_status()
                    elif key == '[':
                        self.decrease_step()
                        self._print_status()
                    elif key == ']':
                        self.increase_step()
                        self._print_status()
                    elif key == ' ':
                        # Resend / hold current target.
                        self._send_arm_goal()
                        self._print_status()
                    elif key == 'o':
                        # Open gripper fully (Req 8.2 / 8.3).
                        self.open_gripper()
                        self._print_status()
                    elif key == 'c':
                        # Close gripper fully (Req 8.2 / 8.3).
                        self.close_gripper()
                        self._print_status()
                    elif key in GRIPPER_JOG_BINDINGS:
                        # Jog both fingers by one step (Req 8.2 / 8.3).
                        self.jog_gripper(GRIPPER_JOG_BINDINGS[key])
                        self._print_status()
                    elif key == '\x03':  # Ctrl+C
                        break
            finally:
                print()  # leave the status line cleanly

    rclpy.init(args=args)
    node = ArmTeleop()

    # Spin in the background so subscriptions and action futures are serviced
    # while the main thread blocks on keyboard input.
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        node.wait_for_server(timeout_sec=10.0)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
