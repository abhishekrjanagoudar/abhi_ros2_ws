# 🤖 mobile_description

ROS 2 package for a differential-drive mobile robot with LiDAR sensor. Includes URDF, Gazebo Harmonic simulation, and RViz visualization.

---

## 🛠️ Building

```bash
cd ~/abhi_ros2_ws
colcon build --packages-select mobile_description
source install/setup.bash
```

---

## 🚀 Quick Start & Examples

Launch the complete robot stack (Gazebo GUI + RViz):
```bash
ros2 launch mobile_description robot.launch.py
```

Simulation only (headless):
```bash
ros2 launch mobile_description robot.launch.py gz:=false rviz:=false ros_ui:=false
```

Different environment:
```bash
ros2 launch mobile_description robot.launch.py env_name:=office_small
```

Keyboard teleop (drive the robot):
```bash
bash src/mobile_description/launch/teleop_interactive.sh
```

---

## ⚙️ Launch Arguments

- `env_name`: Gazebo environment models to load (Default: `cpr_office`). Options: `cpr_office`, `cpr_office_construction`, `office_small`, `office_env_large`, `office_earthquake`.
- `ros_ui`: `true` (default) for RViz, `false` for NiceGUI web UI.
- `gz`: `true` (default) to launch Gazebo UI, `false` for headless.
- `rviz`: `true` (default) to launch RViz independently.
