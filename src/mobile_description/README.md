<div align="center">

# 🤖 mobile_description (Package B)

**Reusable differential-drive AGV base: chassis, wheels, caster, LiDAR, `gz_ros2_control`, `diff_cont` + `joint_broad`, Gazebo environments, SLAM/Nav2, and base teleop.**

*ROS 2 Jazzy · Gazebo Harmonic · Differential-Drive AGV*

---

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.12-yellow?style=flat-square&logo=python)
![License](https://img.shields.io/badge/License-Apache_2.0-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

</div>

---

## Overview

`mobile_description` is the **reusable AGV base layer (Package B)** of the unified mobile
manipulator. It provides the differential-drive chassis URDF/xacro, the `gz_ros2_control` hardware
interface, the `diff_cont` (DiffDriveController) and `joint_broad` (JointStateBroadcaster)
controllers, Gazebo environment worlds, SLAM/Nav2 assets, and keyboard base teleop.

This package's robot/kinematic URDF logic is **frozen**: Package A (`mobile_manipulator`) composes
it via `xacro:include` only and never modifies it. You can run this package standalone (AGV-only)
or let `mobile_manipulator/robot.launch.py` consume it as part of the integrated robot.

---

## ✨ Features

| Capability | Details |
|---|---|
| **Differential drive** | 2 drive wheels + caster, driven by `diff_cont` (DiffDriveController) |
| **ros2_control** | `gz_ros2_control` hardware; `joint_broad` publishes `/joint_states` |
| **Base teleop** | `base_teleop.py` publishes `geometry_msgs/Twist` on `/diff_cont/cmd_vel` |
| **Gazebo environments** | Selectable worlds/models under `config/env/<env>/` |
| **SLAM Toolbox** | Online async mapping from LiDAR `/scan` |
| **Nav2** | AMCL localization against a saved occupancy grid |

---

## 🏗️ Architecture

```
   [slam_toolbox / AMCL] ─────► (map → odom transform)
            │
            ▼
   [joint_broad] ──► /joint_states ──► [robot_state_publisher] ──► (tf frames)
            │
            ▼
   /diff_cont/cmd_vel (Twist) ──► [diff_cont] ──► wheel joints (Gazebo physics)
                                       │
                                       └──► /diff_cont/odom (odom → base_link)
```

Within the unified system, this package supplies the AGV URDF (with the single
`gz_ros2_control` plugin), `my_controllers.yaml` (`diff_cont`, `joint_broad`), and the
`ros_gz_bridge.yaml` (`/clock`, `/scan`) that Package A reuses unchanged.

---

## 📋 Dependencies

### System

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo Sim | Harmonic |

### Declared in `package.xml`

`urdf`, `xacro`, `robot_state_publisher`, `controller_manager`, `ros2_control`,
`ros2_controllers`, `gz_ros2_control`, `diff_drive_controller`, `joint_state_broadcaster`,
`ros_gz_sim`, `ros_gz_bridge`, `nav2_bringup`, `navigation2`, `slam_toolbox`,
`teleop_twist_keyboard`, plus Python runtime deps `rclpy`, `geometry_msgs`, `sensor_msgs`.

### Install runtime deps

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-teleop-twist-keyboard
```

Or: `rosdep install --from-paths src --ignore-src -r -y`.

---

## 🛠️ Build

```bash
cd ~/abhi_ros2_ws
colcon build --packages-select mobile_description mobile_manipulator
source install/setup.bash
```

To build the AGV base on its own:

```bash
colcon build --packages-select mobile_description
source install/setup.bash
```

---

## 🚀 Launch (AGV-only)

```bash
# Full AGV stack (Gazebo + spawn + controllers + RViz + SLAM)
ros2 launch mobile_description robot.launch.py

# Choose an environment / enable the Gazebo GUI
ros2 launch mobile_description robot.launch.py env_name:=office_small gz:=true
```

### `robot.launch.py` arguments

| Argument | Default | Description |
|---|:---:|---|
| `env_name` | `cpr_office` | Environment subfolder under `config/env/` (e.g. `cpr_office`, `office_small`, `office_env_large`) |
| `ros_ui` | `True` | `true` ⇒ RViz; `false` ⇒ optional NiceGUI web UI |
| `gz` | `True` | Enable the Gazebo GUI (`false` ⇒ headless) |
| `rviz` | `True` | Launch RViz independently of `ros_ui` |

For the **integrated robot** (AGV + arm + gripper), use Package A's single entry point instead:
`ros2 launch mobile_manipulator robot.launch.py`.

---

## 🎮 Base teleop (Terminal 2)

The in-package keyboard teleop publishes `geometry_msgs/Twist` on `/diff_cont/cmd_vel` —
exactly the topic/type `diff_cont` consumes (`use_stamped_vel: false`):

```bash
ros2 run mobile_description base_teleop.py
```

### Key map

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

The publisher uses a short burst timeout: tap a movement key to command motion; when no key is
pressed the command decays to zero (the robot stops on key release).

### Alternative — `teleop_twist_keyboard` (community standard)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel
```

- It publishes `geometry_msgs/Twist`, matching `diff_cont`'s configuration (no `TwistStamped`).
- The `-r /cmd_vel:=/diff_cont/cmd_vel` remap retargets its default `/cmd_vel` to the topic
  `diff_cont` consumes.
- Requires `teleop_twist_keyboard` (`sudo apt install ros-jazzy-teleop-twist-keyboard`).

> The base teleop never commands the arm or gripper.

---

## 📡 Key interfaces

| Topic | Type | Direction | Description |
|---|---|:---:|---|
| `/diff_cont/cmd_vel` | `geometry_msgs/Twist` | Input | Linear/angular velocity command |
| `/diff_cont/odom` | `nav_msgs/Odometry` | Output | Wheel odometry (`odom → base_link`) |
| `/scan` | `sensor_msgs/LaserScan` | Output | LiDAR scan in `laser_frame` |
| `/joint_states` | `sensor_msgs/JointState` | Output | Joint states from `joint_broad` |
| `/clock` | `rosgraph_msgs/Clock` | Output | Bridged simulation clock |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Output | Coordinate frame transforms |

---

## 🔍 Monitoring & debugging

```bash
# Save active SLAM map
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# Record bags for drift analysis
ros2 bag record -o scan_bag /tf /tf_static /scan /diff_cont/odom /joint_states /clock
```

---

## 🗂️ Project structure

```
mobile_description/
├── config/              # robot/ (URDF, my_controllers.yaml, ros_gz_bridge.yaml), env/, nav2/
├── launch/              # robot.launch.py, gazebo.launch.py, rviz.launch.py, slam.launch.py, ...
├── rviz/                # default.rviz
├── scripts/             # base_teleop.py, teleop_controller.py, data/AI utilities
├── CMakeLists.txt
└── package.xml
```

---

## 🐛 Troubleshooting

| Symptom | Fix |
|---|---|
| Robot does not move on teleop | Confirm `base_teleop.py` is publishing `Twist` on `/diff_cont/cmd_vel` and `diff_cont` is `active` |
| `base_teleop.py` not found | Rebuild and re-source; it installs to `lib/mobile_description` |
| Gazebo/RViz crash under WSL2 | Use software GL: `export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe`, or run headless |
| No `/scan` | Confirm `ros_gz_bridge` is running and the LiDAR sensor is in the spawned model |

---

## 📦 Package info

| Field | Value |
|---|---|
| **Package name** | `mobile_description` |
| **Version** | `0.1.0` |
| **Build system** | `ament_cmake` + `ament_cmake_python` |
| **License** | Apache-2.0 |
| **Maintainer** | abhishek-janagoudar |

---

<div align="center">

*Built with ROS 2 Jazzy · Gazebo Harmonic*

</div>
