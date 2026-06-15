#!/usr/bin/env python3
"""
Arm Joint Constants — Single Source of Truth (Package A)
========================================================
Pure, ROS-free joint-name and joint-limit constants shared by
``arm_planner.py`` and ``arm_teleop.py``.

This module deliberately imports **only** the Python standard library (``math``)
so it can be imported without a running ROS graph (no ``rclpy``). Both the
joint-name parity test (task 2) and the property-based test (task 6) import the
teleop module — which in turn imports these constants — without sourcing ROS,
so keeping this module rclpy-free is a hard requirement.

Constants
---------
ARM_JOINTS : list[str]
    The 5 actuated arm joints, ordered shoulder -> wrist. Must match the
    ``ArmSystem`` ros2_control block in the combined URDF (Req 13.1 parity).
GRIPPER_JOINTS : list[str]
    The 2 gripper finger joints. Must match the ``GripperSystem`` block.
JOINT_LIMITS : dict[str, tuple[float, float]]
    Maps joint name -> ``(lower, upper)`` from the URDF (radians for arm
    joints, meters for the prismatic finger joints).
"""

import math

# ---------------------------------------------------------------------------
# Joint names — single source of truth (Req 13.1 / joint parity).
# Must match arm_planner.py and the combined URDF ros2_control blocks exactly.
# ---------------------------------------------------------------------------
ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
]

GRIPPER_JOINTS = ['left_finger_joint', 'right_finger_joint']

# ---------------------------------------------------------------------------
# URDF joint limits table (from arm.urdf.xacro / gripper.urdf.xacro).
# Maps joint_name -> (lower, upper) in radians (meters for finger joints).
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    'shoulder_pan_joint':  (-math.pi, math.pi),
    'shoulder_lift_joint': (-math.pi / 2, math.pi / 2),
    'elbow_joint':         (-math.pi, 0.0),
    'wrist_1_joint':       (-math.pi, math.pi),
    'wrist_2_joint':       (-math.pi, math.pi),
    'left_finger_joint':   (0.0, 0.04),
    'right_finger_joint':  (0.0, 0.04),
}
