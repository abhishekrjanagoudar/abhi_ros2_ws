<div align="center">

# 🤖 Unified Mobile Manipulator Workspace

**One robot, one command: a differential-drive AGV base with a 5-DOF arm and 2-finger gripper, driven and jogged from separate terminals.**

*ROS 2 Jazzy · Gazebo Harmonic · Ubuntu 24.04 LTS (WSL2)*

---

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

</div>

---

## System overview

This workspace unifies two ROS 2 packages into a single mobile manipulator that launches with one
command and is teleoperated across separate terminals:

- **Package B — `mobile_description`**: the reusable differential-drive AGV base (chassis, wheels,
  caster, LiDAR, `gz_ros2_control`, `diff_cont` + `joint_broad`, Gazebo environments, SLAM/Nav2,
  base teleop). Its robot/kinematic URDF is **frozen** and reused unchanged.
- **Package A — `mobile_manipulator`**: composes the AGV with a 5-DOF arm + 2-finger gripper via
  `xacro:include`, adds MoveIt 2 and the arm/gripper controllers, and owns the **canonical
  single-command launcher**.

```bash
ros2 launch mobile_manipulator robot.launch.py
```

This spawns the **integrated robot** (AGV + arm) as one Gazebo model with one
`robot_state_publisher`, a connected TF tree, all four controllers active, and (optionally)
RViz + MoveIt. Base teleop and arm teleop then run independently in their own terminals.

---

## Package structure

```
abhi_ros2_ws/
├── README.md                       # ← this file (top-level)
└── src/
    ├── mobile_description/          # Package B — AGV base (frozen URDF)
    │   ├── config/                  # robot/ (URDF, my_controllers.yaml, ros_gz_bridge.yaml), env/, nav2/
    │   ├── launch/                  # robot.launch.py, gazebo/rviz/slam launch files
    │   ├── rviz/                    # default.rviz
    │   ├── scripts/                 # base_teleop.py, teleop_controller.py, utilities
    │   └── README.md                # Package B docs
    │
    └── mobile_manipulator/          # Package A — combined robot + top-level launch
        ├── config/                  # arm_controllers.yaml (arm_controller + gripper_controller)
        ├── launch/                  # robot.launch.py (canonical), moveit/simulation/pick_place
        ├── moveit_config/           # SRDF, kinematics, planning, controller mapping
        ├── scripts/                 # arm_teleop.py, arm_planner.py, arm_joints.py, fk/ik, pick_place_node.py
        ├── urdf/                    # mobile_manipulator.urdf.xacro (includes B's URDF)
        ├── worlds/                  # pick_place.world
        ├── rviz/                    # mobile_manipulator.rviz
        └── README.md                # Package A docs
```

---

## Installation

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS (WSL2 supported) |
| ROS 2 | Jazzy Jalisco |
| Gazebo Sim | Harmonic |
| MoveIt | MoveIt 2 |

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-controller-manager \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard
```

Or resolve everything from the package manifests:

```bash
cd ~/abhi_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
cd ~/abhi_ros2_ws
colcon build --packages-select mobile_description mobile_manipulator
source install/setup.bash
```

`mobile_description` builds first because the combined URDF includes it.

---

## Quick start (three terminals)

> Source `install/setup.bash` in every terminal first.

**Terminal 1 — bring up the integrated robot**

```bash
ros2 launch mobile_manipulator robot.launch.py
```

**Terminal 2 — AGV base teleop**

```bash
ros2 run mobile_description base_teleop.py
```

**Terminal 3 — arm + gripper teleop**

```bash
ros2 run mobile_manipulator arm_teleop.py
```

---

## Launch commands & arguments

Canonical single entry point (Package A):

```bash
ros2 launch mobile_manipulator robot.launch.py
ros2 launch mobile_manipulator robot.launch.py --show-args
```

| Argument | Default | Description |
|---|:---:|---|
| `gz` | `true` | Gazebo GUI on. `false` ⇒ headless server |
| `use_rviz` | `true` | Start RViz2 with the manipulator config |
| `use_moveit` | `true` | Start `move_group`. `false` ⇒ Gazebo + controllers + teleop only |
| `world` | `pick_place` | World name selecting `<name>.world` under `worlds/` (or an absolute path) |
| `env` | `cpr_office` | AGV environment model path for `GZ_SIM_RESOURCE_PATH` |

Examples:

```bash
ros2 launch mobile_manipulator robot.launch.py gz:=false          # headless
ros2 launch mobile_manipulator robot.launch.py use_moveit:=false  # no MoveIt
```

AGV-only launch (Package B) with arguments `env_name` (default `cpr_office`), `ros_ui`
(default `True`), `gz` (default `True`), `rviz` (default `True`):

```bash
ros2 launch mobile_description robot.launch.py
ros2 launch mobile_description robot.launch.py env_name:=office_small gz:=true
```

Controllers brought up by `robot.launch.py`: `joint_broad`, `diff_cont`, `arm_controller`,
`gripper_controller`. Verify with:

```bash
ros2 control list_controllers
```

---

## AGV teleop (Terminal 2)

```bash
ros2 run mobile_description base_teleop.py
```

Publishes `geometry_msgs/Twist` on `/diff_cont/cmd_vel` (matches `diff_cont`,
`use_stamped_vel: false`).

| Key | Action |
|:---:|---|
| `w` / `i` | forward |
| `x` / `,` | backward |
| `a` / `j` | rotate left |
| `d` / `l` | rotate right |
| `s` / `k` | stop |
| `q` / `z` | increase / decrease speed |
| `e` / `c` | increase / decrease angular scale |
| `Ctrl+C` | quit |

Alternative community-standard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel
```

---

## Arm teleop (Terminal 3)

```bash
ros2 run mobile_manipulator arm_teleop.py
```

A direct keyboard joint-jogger (no MoveIt Servo). Each keypress jogs the selected joint by the
current step (default `0.05` rad), clamps to the URDF limit, and sends a single-point
`FollowJointTrajectory` goal to `/arm_controller/follow_joint_trajectory`
(gripper goals go to `/gripper_controller/follow_joint_trajectory`).

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

Joint limits enforced by the jogger:

| Joint | Lower | Upper |
|---|:---:|:---:|
| `shoulder_pan_joint` | −π | π |
| `shoulder_lift_joint` | −π/2 | π/2 |
| `elbow_joint` | −π | 0 |
| `wrist_1_joint` | −π | π |
| `wrist_2_joint` | −π | π |
| `left_finger_joint` / `right_finger_joint` | 0.0 | 0.04 |

---

## RViz & Gazebo usage

- **Gazebo Harmonic** starts with `robot.launch.py` (GUI when `gz:=true`, default). The world is
  selected by `world:=<name>` (default `pick_place`); the AGV environment models come from
  `env:=<name>` (default `cpr_office`).
- **RViz2** starts when `use_rviz:=true` (default) using `mobile_manipulator.rviz`. With
  `use_moveit:=true` the MotionPlanning display connects to `move_group`.
- All nodes run on `use_sim_time: true`; `/clock` is bridged from Gazebo via `ros_gz_bridge`.
- **Headless** operation: `gz:=false` runs the Gazebo server with `--headless-rendering`.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo GUI / RViz crash under WSL2 ("Failed to create OpenGL context") | `robot.launch.py` sets `LIBGL_ALWAYS_SOFTWARE=1` and `GALLIUM_DRIVER=llvmpipe`; keep them or run `gz:=false` |
| A controller is not `active` | Check `ros2 control list_controllers`; the launch sequences spawners after `controller_manager` is ready (TimerActions) |
| Base teleop doesn't move the robot | Confirm `base_teleop.py` publishes `Twist` on `/diff_cont/cmd_vel` and `diff_cont` is active |
| Arm teleop keys do nothing | Wait for the action server message to clear; the node calls `wait_for_server` on `/arm_controller/follow_joint_trajectory` |
| Gripper keys inert | The node warns if `/gripper_controller` is unavailable; confirm the gripper controller spawned |
| `ros2 run ... <script>.py` not found | Rebuild + re-source; scripts install to `lib/<package>` |
| Don't need motion planning | Launch with `use_moveit:=false` |
| Missing `teleop_twist_keyboard` | `sudo apt install ros-jazzy-teleop-twist-keyboard`, or use the in-package `base_teleop.py` |

---

## Package documentation

- [`src/mobile_manipulator/README.md`](src/mobile_manipulator/README.md) — Package A (combined robot, launch, arm teleop)
- [`src/mobile_description/README.md`](src/mobile_description/README.md) — Package B (AGV base, base teleop, SLAM/Nav2)

---

<div align="center">

*Built with ROS 2 Jazzy · Gazebo Harmonic · MoveIt 2*

</div>
