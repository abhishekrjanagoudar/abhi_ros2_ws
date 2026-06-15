#!/usr/bin/env python3
"""
Forward Kinematics — 5-DOF Mobile Manipulator Arm
==================================================
Computes end-effector pose (position + rotation matrix) from joint angles
using the Standard Denavit-Hartenberg (DH) convention.

DH Parameter Table
------------------
The arm is parameterised as a standard DH chain from arm_base_link to
wrist_2_link (end-effector parent).

  Joint i | a_i (m) | d_i (m) | alpha_i (rad) | theta_i
  --------|---------|---------|---------------|--------
    1     |  0.000  |  0.090  |   +π/2        |  θ1  (shoulder_pan)
    2     |  0.300  |  0.000  |   0.000       |  θ2  (shoulder_lift)
    3     |  0.250  |  0.000  |   0.000       |  θ3  (elbow)
    4     |  0.000  |  0.000  |   +π/2        |  θ4  (wrist_1)
    5     |  0.000  |  0.120  |   0.000       |  θ5  (wrist_2)

  Notes:
    d1 = 0.02 (plate) + 0.06 (shoulder cylinder) + 0.01 (pan joint z) = 0.09 m
    a2 = 0.30 m (upper arm box length)
    a3 = 0.25 m (forearm box length)
    d4 = 0 (wrist_1 at forearm end, no offset along Z4)
    d5 = 0.07 (wrist_1 cylinder) + 0.05 (wrist_2 cylinder) = 0.12 m total EE offset

Standard DH transform from frame i-1 to frame i:
    T_i = Tz(d_i) · Rz(θ_i) · Tx(a_i) · Rx(α_i)

Usage
-----
As a standalone script (prints result):
    python3 fk.py 0 -1.5708 0 0 0

As a ROS 2 node (subscribes to joint_states, publishes /arm/fk_result):
    ros2 run mobile_manipulator fk.py

As a library:
    from mobile_manipulator.fk import forward_kinematics
    pos, rot = forward_kinematics([0, -1.57, 0, 0, 0])
"""

import math
import sys
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# DH Parameters (Standard convention)
# ---------------------------------------------------------------------------
# Rows: [a_i, d_i, alpha_i]  (theta_i is the joint variable)
DH_PARAMS = [
    #    a       d      alpha
    [0.000,  0.090,  math.pi / 2],   # Joint 1 — shoulder_pan
    [0.300,  0.000,  0.000],          # Joint 2 — shoulder_lift
    [0.250,  0.000,  0.000],          # Joint 3 — elbow
    [0.000,  0.000,  math.pi / 2],   # Joint 4 — wrist_1
    [0.000,  0.120,  0.000],          # Joint 5 — wrist_2
]

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
]


def _dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute the 4×4 homogeneous DH transform matrix.

    Standard DH convention:
        T = Tz(d) · Rz(θ) · Tx(a) · Rx(α)

    Parameters
    ----------
    a     : link length (m)
    d     : link offset along Z (m)
    alpha : twist angle between Z axes (rad)
    theta : joint angle variable (rad)

    Returns
    -------
    np.ndarray  shape (4, 4)
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)

    return np.array([
        [ct,       -st,       0,        a],
        [st * ca,   ct * ca, -sa,  -sa * d],
        [st * sa,   ct * sa,  ca,   ca * d],
        [0,         0,        0,        1],
    ], dtype=float)


def forward_kinematics(
    joint_angles: List[float],
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute FK: end-effector position and rotation matrix.

    Multiplies the five DH matrices (one per joint) in order from
    base (arm_base_link frame) to tip (wrist_2_link frame).

    Parameters
    ----------
    joint_angles : list of 5 floats [θ1 … θ5] in radians

    Returns
    -------
    position : np.ndarray shape (3,)  — [x, y, z] in metres
    rotation : np.ndarray shape (3,3) — rotation matrix (EE orientation)

    Raises
    ------
    ValueError if not exactly 5 joint angles are provided
    """
    if len(joint_angles) != 5:
        raise ValueError(f'Expected 5 joint angles, got {len(joint_angles)}')

    T = np.eye(4)
    for i, (a, d, alpha) in enumerate(DH_PARAMS):
        T = T @ _dh_matrix(a, d, alpha, joint_angles[i])

    position = T[:3, 3]
    rotation = T[:3, :3]
    return position, rotation


def pose_to_str(pos: np.ndarray, rot: np.ndarray) -> str:
    """Format FK result as a human-readable string."""
    rpy = rotation_matrix_to_rpy(rot)
    return (
        f'Position : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f} [m]\n'
        f'Rotation : roll={math.degrees(rpy[0]):.2f}°  '
        f'pitch={math.degrees(rpy[1]):.2f}°  '
        f'yaw={math.degrees(rpy[2]):.2f}°'
    )


def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract ZYX Euler angles (roll, pitch, yaw) from a rotation matrix."""
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if abs(math.cos(pitch)) < 1e-6:
        roll = math.atan2(R[0, 1], R[1, 1])
        yaw  = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw  = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# ROS 2 node wrapper
# ---------------------------------------------------------------------------

def _run_as_ros2_node() -> None:
    """Run as a ROS 2 node: subscribe to /joint_states, publish FK result."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from tf_transformations import quaternion_from_matrix
    import numpy as np

    class FKNode(Node):
        def __init__(self) -> None:
            super().__init__('arm_fk_node')
            self._joint_positions: dict = {}

            self._sub = self.create_subscription(
                JointState,
                '/joint_states',
                self._js_callback,
                10,
            )
            self._pub = self.create_publisher(PoseStamped, '/arm/fk_result', 10)
            self._timer = self.create_timer(0.05, self._publish_fk)  # 20 Hz
            self.get_logger().info('arm_fk_node started — subscribed to /joint_states')

        def _js_callback(self, msg: JointState) -> None:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

        def _publish_fk(self) -> None:
            try:
                angles = [self._joint_positions[n] for n in JOINT_NAMES]
            except KeyError:
                return  # not all joints received yet

            pos, rot = forward_kinematics(angles)

            # Build 4×4 matrix for quaternion extraction
            T = np.eye(4)
            T[:3, :3] = rot
            quat = quaternion_from_matrix(T)  # [x, y, z, w]

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'arm_base_link'
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.x = float(quat[0])
            msg.pose.orientation.y = float(quat[1])
            msg.pose.orientation.z = float(quat[2])
            msg.pose.orientation.w = float(quat[3])
            self._pub.publish(msg)

    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """CLI: compute FK for given joint angles (degrees), or run as ROS 2 node."""
    args = sys.argv[1:]

    # If 5 numeric arguments given → CLI mode
    if len(args) == 5:
        try:
            angles_deg = [float(a) for a in args]
        except ValueError:
            print('Error: all 5 arguments must be numeric joint angles (degrees).')
            sys.exit(1)

        angles_rad = [math.radians(a) for a in angles_deg]
        pos, rot = forward_kinematics(angles_rad)
        print('\n=== Forward Kinematics Result ===')
        for name, angle in zip(JOINT_NAMES, angles_deg):
            print(f'  {name}: {angle:.2f}°')
        print()
        print(pose_to_str(pos, rot))

        # Workspace reachability check
        reach = float(np.linalg.norm(pos))
        max_reach = sum(row[0] for row in DH_PARAMS) + DH_PARAMS[4][1]
        print(f'\nReach: {reach:.4f} m  (max theoretical: {max_reach:.4f} m)')
        return

    # Otherwise → ROS 2 node mode
    if len(args) == 0:
        _run_as_ros2_node()
        return

    print(__doc__)
    print(f'Usage: fk.py [θ1 θ2 θ3 θ4 θ5]  (degrees)')
    print(f'       fk.py                      (ROS 2 node mode)')
    sys.exit(1)


if __name__ == '__main__':
    main()
