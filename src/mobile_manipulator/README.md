<div align="center">

# 🤖 mobile_manipulator (Package A)

**Unified mobile manipulator: a 5-DOF arm + 2-finger gripper composed onto the `mobile_description` AGV, with MoveIt 2, ros2_control, and a single-command launch.**

*ROS 2 Jazzy · Gazebo Harmonic · 5-DOF Mobile Manipulator*

---

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

</div>

---

## Overview

`mobile_manipulator` is the **top-level package** of the unified system. It composes the
`mobile_description` AGV base with a 5-DOF robotic arm and a 2-finger parallel gripper via
`xacro:include` (the AGV's robot/kinematic URDF is reused unmodified), and owns the canonical
single-command launcher:

```bash
ros2 launch mobile_manipulator robot.launch.py
```

This one command brings up Gazebo Harmonic, the combined URDF in a single
`robot_state_publisher`, one spawned Gazebo model, all four controllers, and (optionally) MoveIt 2
and RViz. Base teleop and arm teleop then run as independent commands in their own terminals.

The guiding principle is **minimal change / maximum reuse**: existing working assets (combined
URDF, MoveIt config, controllers, simulation launch sequencing) are reused as-is; only the missing
pieces — the canonical entry point and the keyboard arm joint-jogger — were added.

---

## ✨ Features

| Capability | Details |
|---|---|
| **Single-command launch** | `robot.launch.py` brings up Gazebo + combined robot + controllers + RViz/MoveIt |
| **Integrated robot** | One combined URDF feeds both RSP and `ros_gz_sim create` → one RSP, one spawned model |
| **5-DOF arm + gripper** | Arm mounts on `chassis` (`xyz="0 0 0.15"`); gripper mounts on `wrist_2_link` |
| **ros2_control stack** | A single `gz_ros2_control` plugin discovers AGV + arm + gripper `ros2_control` blocks |
| **Keyboard arm teleop** | `arm_teleop.py` joint-jogger sends `FollowJointTrajectory` goals (no MoveIt Servo) |
| **MoveIt 2** | Optional `move_group` with SRDF planning groups (`arm`, `gripper`), OMPL + KDL |
| **Pick & Place demo** | `pick_place_node.py` finite-state machine + `pick_place.world` |

---

## 🏗️ Architecture

### Layered composition

```
Package A: mobile_manipulator  (owns the combined robot + top-level launch)
  robot.launch.py  ── single entry point
     ├─ Gazebo Harmonic (ros_gz_sim)
     ├─ robot_state_publisher  ← combined URDF (AGV + arm + gripper)
     ├─ ros_gz create (spawn ONE integrated robot, via -string)
     ├─ ros_gz_bridge (/clock, /scan)
     ├─ spawners: joint_broad, diff_cont, arm_controller, gripper_controller
     ├─ move_group (MoveIt 2)   [optional: use_moveit:=true]
     └─ rviz2                   [optional: use_rviz:=true]
  urdf/mobile_manipulator.urdf.xacro  ── xacro:include of B's URDF (compose only)
  scripts/arm_teleop.py  ── keyboard joint-jogger (arm + gripper)
        │ xacro:include (no edits to B's robot URDF)
        ▼
Package B: mobile_description  (reusable AGV base layer — URDF frozen)
```

### Startup sequencing (handled by `robot.launch.py`)

| t (s) | Action |
|------:|--------|
| 0.0 | Gazebo + `robot_state_publisher` (combined URDF) + env vars |
| 1.0 | Spawn ONE integrated robot via `ros_gz_sim create -string <combined_urdf>` |
| 1.5 | `ros_gz_bridge` (`/clock`, `/scan`) |
| 3.0 | `joint_broad` + `diff_cont` spawners |
| 4.0 | `arm_controller` spawner |
| 4.5 | `gripper_controller` spawner |
| 9.0 | RViz2 (if `use_rviz`) |
| 10.0 | `move_group` (if `use_moveit`) |

### Controllers

| Controller | Type | Joints |
|---|---|---|
| `joint_broad` | JointStateBroadcaster | all state interfaces |
| `diff_cont` | DiffDriveController | `left_wheel_joint`, `right_wheel_joint` |
| `arm_controller` | JointTrajectoryController | 5 arm joints |
| `gripper_controller` | JointTrajectoryController | 2 finger joints |

---

## 📋 Dependencies

### System

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo Sim | Harmonic |
| MoveIt | MoveIt 2 |

### Declared in `package.xml`

`mobile_description` (the AGV base), `urdf`, `xacro`, `robot_state_publisher`,
`controller_manager`, `ros2_control`, `ros2_controllers`, `gz_ros2_control`,
`joint_trajectory_controller`, `joint_state_broadcaster`, `ros_gz_sim`, `ros_gz_bridge`,
`rviz2`, `teleop_twist_keyboard`, the MoveIt 2 stack (`moveit_ros_move_group`,
`moveit_ros_planning_interface`, `moveit_ros_planning`, `moveit_core`, `moveit_msgs`,
`moveit_simple_controller_manager`), `nav2_msgs`, and Python message deps (`rclpy`,
`geometry_msgs`, `sensor_msgs`, `std_msgs`, `trajectory_msgs`, `control_msgs`, `action_msgs`,
`tf2_ros`, `tf2_geometry_msgs`).

### Install the heavy runtime deps

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-controller-manager \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-teleop-twist-keyboard
```

Or resolve everything from the manifests:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🛠️ Build

```bash
cd ~/abhi_ros2_ws
colcon build --packages-select mobile_description mobile_manipulator
source install/setup.bash
```

`mobile_description` is built first because the combined URDF includes it.

---

## 🚀 Launch — single entry point

```bash
ros2 launch mobile_manipulator robot.launch.py
```

### Launch arguments (with defaults)

| Argument | Default | Description |
|---|:---:|---|
| `gz` | `true` | Gazebo GUI on. `false` ⇒ headless server (`-s --headless-rendering`) |
| `use_rviz` | `true` | Start RViz2 with the manipulator config |
| `use_moveit` | `true` | Start `move_group`. `false` ⇒ Gazebo + controllers + teleop only |
| `world` | `pick_place` | World name selecting `<name>.world` under `worlds/` (or an absolute path) |
| `env` | `cpr_office` | AGV environment model path for `GZ_SIM_RESOURCE_PATH` |

Inspect args at any time:

```bash
ros2 launch mobile_manipulator robot.launch.py --show-args
```

Examples:

```bash
# Headless Gazebo
ros2 launch mobile_manipulator robot.launch.py gz:=false

# Without MoveIt (arm + controllers + teleop only)
ros2 launch mobile_manipulator robot.launch.py use_moveit:=false
```

---

## 🎮 Three-terminal workflow

**Terminal 1 — bring up the robot**

```bash
ros2 launch mobile_manipulator robot.launch.py
```

**Terminal 2 — base teleop** (drives the AGV; see Package B for full details)

```bash
ros2 run mobile_description base_teleop.py
# publishes geometry_msgs/Twist on /diff_cont/cmd_vel
```

**Terminal 3 — arm teleop** (keyboard joint-jogger)

```bash
ros2 run mobile_manipulator arm_teleop.py
```

### Arm teleop key map

`arm_teleop.py` is a direct keyboard joint-jogger (no MoveIt Servo). Each keypress jogs the
selected joint by the current step (default `0.05` rad), clamps to the URDF limit, and sends a
single-point `FollowJointTrajectory` goal.

| Key | Action |
|:---:|---|
| `1` / `q` | `shoulder_pan_joint` + / − |
| `2` / `w` | `shoulder_lift_joint` + / − |
| `3` / `e` | `elbow_joint` + / − |
| `4` / `r` | `wrist_1_joint` + / − |
| `5` / `t` | `wrist_2_joint` + / − |
| `[` / `]` | decrease / increase jog step |
| `o` / `c` | open / close gripper (full) |
| `=` / `-` | jog gripper open / closed (one step) |
| `space` | resend current target (hold) |
| `Ctrl+C` | quit |

Goals are sent to:
- `/arm_controller/follow_joint_trajectory` (5 arm joints)
- `/gripper_controller/follow_joint_trajectory` (2 finger joints)

### Joint limits (clamped by the jogger)

| Joint | Lower | Upper |
|---|:---:|:---:|
| `shoulder_pan_joint` | −π | π |
| `shoulder_lift_joint` | −π/2 | π/2 |
| `elbow_joint` | −π | 0 |
| `wrist_1_joint` | −π | π |
| `wrist_2_joint` | −π | π |
| `left_finger_joint` / `right_finger_joint` | 0.0 | 0.04 |

---

## 📡 Key interfaces

| Interface | Type | Producer | Consumer |
|---|---|---|---|
| `/diff_cont/cmd_vel` | `geometry_msgs/Twist` | base teleop | `diff_cont` |
| `/diff_cont/odom` | `nav_msgs/Odometry` | `diff_cont` | TF / Nav |
| `/joint_states` | `sensor_msgs/JointState` | `joint_broad` | RSP, teleop, MoveIt |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | arm teleop / MoveIt | `arm_controller` |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | arm teleop / MoveIt | `gripper_controller` |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (bridged) | all (`use_sim_time`) |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (bridged) | RViz / SLAM |

---

## 🗂️ Project structure

```
mobile_manipulator/
├── config/              # arm_controllers.yaml (arm_controller + gripper_controller)
├── launch/              # robot.launch.py (canonical), simulation/moveit/bringup/pick_place
├── moveit_config/       # SRDF, kinematics, planning, controller mapping
├── scripts/             # arm_teleop.py, arm_planner.py, arm_joints.py, fk.py, ik.py, pick_place_node.py
├── urdf/                # mobile_manipulator.urdf.xacro (+ arm/gripper macros)
├── worlds/              # pick_place.world
├── rviz/                # mobile_manipulator.rviz
├── CMakeLists.txt
└── package.xml
```

---

## 🐛 Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo GUI / RViz crash under WSL2 | The launch sets `LIBGL_ALWAYS_SOFTWARE=1` and `GALLIUM_DRIVER=llvmpipe`; keep them, or run `gz:=false` |
| A controller never reaches `active` | Verify with `ros2 control list_controllers`; the launch sequences spawners after `controller_manager` is up |
| `arm_teleop.py` keys do nothing | Wait for `Waiting for /arm_controller/follow_joint_trajectory ...` to clear; the node waits for the action server |
| Gripper keys inert | The node warns if `/gripper_controller` is unavailable; confirm the gripper controller spawned |
| MoveIt not needed | Launch with `use_moveit:=false` — Gazebo, controllers, and teleop still work |

---

## 📦 Package info

| Field | Value |
|---|---|
| **Package name** | `mobile_manipulator` |
| **Version** | `0.1.0` |
| **Build system** | `ament_cmake` + `ament_cmake_python` |
| **License** | Apache-2.0 |
| **Maintainer** | abhishek-janagoudar |

---

<div align="center">

*Built with ROS 2 Jazzy · Gazebo Harmonic · MoveIt 2*

</div>
