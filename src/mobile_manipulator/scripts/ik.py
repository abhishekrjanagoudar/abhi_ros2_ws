#!/usr/bin/env python3
"""
Inverse Kinematics — 5-DOF Mobile Manipulator Arm
==================================================
Provides a Jacobian-based numerical IK solver for the 5-DOF arm,
exposed both as a library function and as a ROS 2 service server.

Algorithm
---------
Damped Least Squares (DLS) Jacobian method:
    Δθ = Jᵀ (J Jᵀ + λ²I)⁻¹ · Δx

Where:
    J    : 6×5 geometric Jacobian (position + orientation rows)
    Δx   : 6-vector error [position_error(3), orientation_error(3)]
    λ    : damping coefficient (avoids singularity blow-up)
    Δθ   : joint angle update

The solver iterates up to ``MAX_ITER`` times with a step size ``ALPHA``.
It performs ``NUM_SEEDS`` attempts with random seeds and returns the
solution with the smallest final error.

ROS 2 service
-------------
Service name : ~/compute_ik
Service type : moveit_msgs/srv/GetPositionIK

The node also subscribes to /target_pose (geometry_msgs/PoseStamped)
and publishes the computed joint angles to /arm/ik_joint_goal
(sensor_msgs/JointState) — this is the Phase 3 hook used by arm_planner.py.

Usage
-----
As a service node:
    ros2 run mobile_manipulator ik.py

Test the service:
    ros2 service call /ik_node/compute_ik moveit_msgs/srv/GetPositionIK \\
      "{ik_request: {group_name: arm, pose_stamped: {header: {frame_id: arm_base_link},
         pose: {position: {x: 0.3, y: 0.0, z: 0.3},
                orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}}}"
"""

import math
import sys
from typing import List, Optional, Tuple

import numpy as np

# Re-use DH parameters and FK from fk.py (installed in the same scripts dir)
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from fk import DH_PARAMS, JOINT_NAMES, _dh_matrix, forward_kinematics, rotation_matrix_to_rpy  # noqa: E402

# ---------------------------------------------------------------------------
# Solver parameters
# ---------------------------------------------------------------------------
MAX_ITER    = 500       # maximum iterations per seed
ALPHA       = 0.5       # step size (gradient descent gain)
LAMBDA      = 0.05      # damping coefficient (singularity robustness)
POS_TOL     = 1e-3      # position error threshold [m]
ORI_TOL     = 1e-2      # orientation error threshold [rad]
NUM_SEEDS   = 10        # number of random restarts

# Joint limits [min, max] in radians — must match URDF
JOINT_LIMITS = [
    (-math.pi,      math.pi),       # shoulder_pan
    (-math.pi / 2,  math.pi / 2),   # shoulder_lift
    (-math.pi,      0.0),           # elbow
    (-math.pi,      math.pi),       # wrist_1
    (-math.pi,      math.pi),       # wrist_2
]


# ---------------------------------------------------------------------------
# Geometric Jacobian
# ---------------------------------------------------------------------------

def _compute_jacobian(joint_angles: List[float]) -> np.ndarray:
    """Compute the 6×5 geometric Jacobian at the given joint configuration.

    The Jacobian relates joint velocities to end-effector velocities:
        ẋ = J(q) q̇

    For each joint i, the columns are:
        J_pos_i  = z_{i-1} × (p_e - p_{i-1})   [3-vector, linear velocity]
        J_ori_i  = z_{i-1}                       [3-vector, angular velocity]

    Where z_{i-1} is the Z-axis of frame i-1 and p is the position.

    Parameters
    ----------
    joint_angles : list of 5 floats [θ1 … θ5] in radians

    Returns
    -------
    J : np.ndarray shape (6, 5)
    """
    n = len(DH_PARAMS)
    T_all = [np.eye(4)]
    for i, (a, d, alpha) in enumerate(DH_PARAMS):
        T_all.append(T_all[-1] @ _dh_matrix(a, d, alpha, joint_angles[i]))

    p_e = T_all[-1][:3, 3]  # end-effector position

    J = np.zeros((6, n))
    for i in range(n):
        z_i = T_all[i][:3, 2]   # Z-axis of frame i (before joint i+1)
        p_i = T_all[i][:3, 3]   # origin of frame i
        J[:3, i] = np.cross(z_i, p_e - p_i)   # linear velocity column
        J[3:, i] = z_i                           # angular velocity column

    return J


# ---------------------------------------------------------------------------
# Pose error computation
# ---------------------------------------------------------------------------

def _pose_error(
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    current_pos: np.ndarray,
    current_rot: np.ndarray,
) -> np.ndarray:
    """Compute the 6D pose error vector [Δpos(3), Δori(3)].

    Position error: straightforward vector subtraction.
    Orientation error: axis-angle from the rotation error matrix
        R_err = R_target · R_current.T
    """
    pos_err = target_pos - current_pos

    # Rotation error matrix
    R_err = target_rot @ current_rot.T

    # Extract axis-angle from rotation matrix
    # angle = arccos((trace(R)-1)/2)
    trace = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(trace)

    if abs(angle) < 1e-6:
        ori_err = np.zeros(3)
    else:
        # Axis = [R32-R23, R13-R31, R21-R12] / (2 sin θ)
        ori_err = (angle / (2.0 * math.sin(angle))) * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])

    return np.concatenate([pos_err, ori_err])


# ---------------------------------------------------------------------------
# Core IK solver
# ---------------------------------------------------------------------------

def inverse_kinematics(
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    seed: Optional[List[float]] = None,
) -> Tuple[Optional[List[float]], float]:
    """Solve IK: find joint angles to reach the target pose.

    Uses Damped Least Squares (DLS) Jacobian iteration.
    Performs NUM_SEEDS attempts with different seeds and returns the best.

    Parameters
    ----------
    target_pos : np.ndarray shape (3,) — desired EE position [m]
    target_rot : np.ndarray shape (3,3) — desired EE rotation matrix
    seed       : optional initial joint angles [rad]; if None, uses home pose
                 as first seed and random seeds for remaining attempts

    Returns
    -------
    (joint_angles, final_error)
        joint_angles : list of 5 floats [rad] or None if no solution found
        final_error  : scalar final position+orientation error norm
    """
    best_solution: Optional[List[float]] = None
    best_error = float('inf')

    # Build seed list
    if seed is not None:
        seeds = [list(seed)] + _random_seeds(NUM_SEEDS - 1)
    else:
        # Start from home pose then try random seeds
        home = [0.0, -math.pi / 2, 0.0, 0.0, 0.0]
        seeds = [home] + _random_seeds(NUM_SEEDS - 1)

    for q_init in seeds:
        q = np.array(q_init, dtype=float)

        for _ in range(MAX_ITER):
            pos_curr, rot_curr = forward_kinematics(q.tolist())
            err = _pose_error(target_pos, target_rot, pos_curr, rot_curr)
            err_norm = np.linalg.norm(err)

            if err_norm < 1e-6:
                break

            J = _compute_jacobian(q.tolist())

            # Damped Least Squares: Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ · e
            JJt = J @ J.T + (LAMBDA ** 2) * np.eye(6)
            delta_q = ALPHA * (J.T @ np.linalg.solve(JJt, err))

            q = q + delta_q

            # Clamp to joint limits
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                q[i] = np.clip(q[i], lo, hi)

        # Evaluate final error
        pos_curr, rot_curr = forward_kinematics(q.tolist())
        pos_err = np.linalg.norm(target_pos - pos_curr)
        ori_err = np.linalg.norm(_pose_error(
            target_pos, target_rot, pos_curr, rot_curr
        )[3:])

        total_err = pos_err + 0.5 * ori_err

        if total_err < best_error:
            best_error = total_err
            best_solution = q.tolist()

        # Early exit if within tolerance
        if pos_err < POS_TOL and ori_err < ORI_TOL:
            break

    return best_solution, best_error


def _random_seeds(n: int) -> List[List[float]]:
    """Generate n random joint configurations within limits."""
    seeds = []
    for _ in range(n):
        q = [
            np.random.uniform(lo, hi)
            for lo, hi in JOINT_LIMITS
        ]
        seeds.append(q)
    return seeds


# ---------------------------------------------------------------------------
# ROS 2 service node
# ---------------------------------------------------------------------------

def _run_as_ros2_node() -> None:
    """Expose IK as a ROS 2 service + /target_pose subscriber."""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from tf_transformations import euler_from_quaternion
    import numpy as np

    # Lazy import MoveIt service type — optional dependency
    try:
        from moveit_msgs.srv import GetPositionIK
        _HAS_MOVEIT = True
    except ImportError:
        _HAS_MOVEIT = False

    class IKNode(Node):
        def __init__(self) -> None:
            super().__init__('arm_ik_node')

            # Publish IK result as joint angles for arm_planner.py / pick_place_node.py
            self._pub = self.create_publisher(JointState, '/arm/ik_joint_goal', 10)

            # Phase 3 hook: external vision pipeline sends target poses here
            self._sub = self.create_subscription(
                PoseStamped,
                '/target_pose',
                self._target_pose_callback,
                10,
            )

            # MoveIt-compatible IK service
            if _HAS_MOVEIT:
                self._srv = self.create_service(
                    GetPositionIK,
                    '~/compute_ik',
                    self._ik_service_callback,
                )
                self.get_logger().info(
                    'arm_ik_node ready — service: ~/compute_ik, '
                    'topic: /target_pose'
                )
            else:
                self.get_logger().warn(
                    'moveit_msgs not found — IK service disabled. '
                    'Only /target_pose subscription active.'
                )

        def _pose_stamped_to_arrays(
            self, pose_msg
        ) -> Tuple[np.ndarray, np.ndarray]:
            """Convert geometry_msgs/Pose to (position, rotation_matrix)."""
            pos = np.array([
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
            ])
            q = pose_msg.orientation
            # Build rotation matrix from quaternion
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            rot = np.array([
                [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw),    1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)],
            ])
            return pos, rot

        def _publish_joint_goal(self, angles: List[float]) -> None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [float(a) for a in angles]
            self._pub.publish(msg)

        def _target_pose_callback(self, msg: PoseStamped) -> None:
            """Handle /target_pose — compute IK and publish joint goal."""
            target_pos, target_rot = self._pose_stamped_to_arrays(msg.pose)
            solution, err = inverse_kinematics(target_pos, target_rot)

            if solution is not None and err < (POS_TOL * 5):
                self.get_logger().info(
                    f'IK solved: error={err:.4f} m+rad → publishing joint goal'
                )
                self._publish_joint_goal(solution)
            else:
                self.get_logger().warn(
                    f'IK failed to converge: best error={err:.4f}'
                )

        def _ik_service_callback(self, request, response):
            """Handle moveit_msgs/srv/GetPositionIK service calls."""
            pose = request.ik_request.pose_stamped.pose
            target_pos, target_rot = self._pose_stamped_to_arrays(pose)

            seed_js = request.ik_request.robot_state.joint_state
            seed = None
            if seed_js.name:
                name_to_idx = {n: i for i, n in enumerate(JOINT_NAMES)}
                seed = [0.0] * 5
                for name, pos in zip(seed_js.name, seed_js.position):
                    if name in name_to_idx:
                        seed[name_to_idx[name]] = pos

            solution, err = inverse_kinematics(target_pos, target_rot, seed)

            from moveit_msgs.msg import MoveItErrorCodes
            if solution is not None and err < (POS_TOL * 5):
                response.solution.joint_state.name = JOINT_NAMES
                response.solution.joint_state.position = solution
                response.error_code.val = MoveItErrorCodes.SUCCESS
            else:
                response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response

    rclpy.init()
    node = IKNode()
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
    # If launched with ROS 2 arguments, start as node
    if '--ros-args' in sys.argv or len(sys.argv) == 1:
        _run_as_ros2_node()
        return

    # Filter out anything after --ros-args if mixed (though normally they shouldn't mix)
    try:
        idx = sys.argv.index('--ros-args')
        args = sys.argv[1:idx]
    except ValueError:
        args = sys.argv[1:]

    if len(args) == 7:
        # CLI mode: x y z qx qy qz qw
        try:
            vals = [float(a) for a in args]
        except ValueError:
            print('Error: arguments must be numeric: x y z qx qy qz qw')
            sys.exit(1)

        x, y, z, qx, qy, qz, qw = vals
        target_pos = np.array([x, y, z])
        # Build rotation matrix from quaternion
        rot = np.array([
            [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),    1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)],
        ])

        print(f'\n=== Inverse Kinematics ===')
        print(f'Target: x={x:.3f}  y={y:.3f}  z={z:.3f} [m]')
        print(f'        qx={qx:.3f}  qy={qy:.3f}  qz={qz:.3f}  qw={qw:.3f}')

        solution, err = inverse_kinematics(target_pos, rot)
        if solution is not None:
            print(f'\nSolution (error={err:.4f}):')
            for name, angle in zip(JOINT_NAMES, solution):
                print(f'  {name}: {math.degrees(angle):.2f}°')
        else:
            print('\nNo IK solution found.')
        return

    if len(args) == 0:
        _run_as_ros2_node()
        return

    print(__doc__)
    print('Usage: ik.py [x y z qx qy qz qw]  (CLI, angles in metres/quaternion)')
    print('       ik.py                         (ROS 2 service node)')
    sys.exit(1)


if __name__ == '__main__':
    main()
