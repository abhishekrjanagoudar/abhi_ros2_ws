# 🤖 mobile_description

ROS 2 package for a differential-drive mobile robot with LiDAR sensor. Includes URDF, simulation (Gazebo Harmonic), and visualization (RViz) support.

---

## 🚀 Quick Start

Launch the complete robot stack (Gazebo simulation + Gazebo UI + RViz):

```bash
ros2 launch mobile_description robot.launch.py env_name:=cpr_office ros_ui:=true gz:=true
```

Or headless (Gazebo physics runs, but no GUI):

```bash
ros2 launch mobile_description robot.launch.py
```

---

## ⚙️ Launch Arguments

### 1. `env_name`

Sets the Gazebo environment (world file + 3D models auto-resolve from subfolder):

- `cpr_office` *(default)*: Clearpath office environment
- `cpr_office_construction`: CPR office under construction
- `office_small`: Small office with furniture
- `office_env_large`: Large open-plan office
- `office_earthquake`: Earthquake scenario test environment

**Each environment has:**
- `config/env/<env_name>/worlds/*.world` (auto-selected: first .world file)
- `config/env/<env_name>/models/` (Gazebo model library)

---

### 2. `ros_ui`

Selects the user interface:

- `false` *(default)*: NiceGUI web UI (browser-accessible, limited ROS data)
- `true`: RViz (full ROS visualization; requires host display access)

---

### 3. `gz`

Enable/disable Gazebo GUI:

- `false` *(default)*: Headless (physics runs, no GUI)
- `true`: Gazebo GUI launched (interactive camera, object manipulation)

---

### 4. `rviz`

Launch RViz independently of `ros_ui`:

- `false` *(default)*: Off
- `true`: RViz starts (even if `ros_ui=false`)

---

## 📋 Examples

### Simulation only (headless)
```bash
ros2 launch mobile_description robot.launch.py
```

### Gazebo GUI + RViz
```bash
ros2 launch mobile_description robot.launch.py env_name:=office_small gz:=true ros_ui:=true
```

### Different environment + web UI
```bash
ros2 launch mobile_description robot.launch.py env_name:=cpr_office_construction
```

### RViz standalone (no Gazebo)
```bash
ros2 launch mobile_description rsp.launch.py
rviz2
```

---

## 🎮 Control the Robot

Once simulation is running, drive the robot via keyboard teleop:

```bash
bash src/mobile_description/launch/teleop_interactive.sh
```

Use arrow keys or WASD to drive. Press `q` to quit.

---

## 📦 Package Structure

```
mobile_description/
├── src/
│   └── robot.launch.py           # Top-level orchestrator (env + UI selection)
├── launch/
│   ├── robot.launch.py           # Shim (re-exports src/robot.launch.py)
│   ├── gazebo.launch.py          # Gazebo + spawn + bridge + controllers
│   ├── rsp.launch.py             # Robot State Publisher (broadcasts TF tree)
│   ├── rviz.launch.py            # RViz visualization
│   ├── sim.launch.py             # Legacy wrapper (backward compat)
│   └── teleop_interactive.sh     # Keyboard teleop script
├── config/
│   ├── robot/
│   │   ├── mobile_robot.urdf.xacro  # Robot URDF (links, joints, LiDAR, ros2_control)
│   │   ├── my_controllers.yaml       # diff_drive_controller + joint_state_broadcaster config
│   │   └── ros_gz_bridge.yaml        # Topic bridges (clock, /scan)
│   └── env/
│       ├── cpr_office/
│       │   ├── worlds/           # .world SDF files (auto-selected)
│       │   └── models/           # 3D models (meshes, collada)
│       ├── office_small/
│       │   ├── worlds/
│       │   └── models/
│       └── [other environments...]
├── rviz/
│   └── default.rviz              # Default RViz configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🦾 Robot Structure

**TF Tree:**
```
odom
└── base_link (robot root reference frame)
    ├── chassis (main body, blue box)
    │   ├── caster_wheel (rear support, low-friction sphere)
    │   └── laser_frame (LiDAR sensor, top-front)
    ├── left_wheel (front-left continuous joint)
    └── right_wheel (front-right continuous joint)
```

**Components:**
- **chassis**: 0.4m (X) × 0.3m (Y) × 0.15m (Z) box, mass 2.0 kg
- **wheels**: 0.05m radius cylinders, 0.04m width, high friction (μ=1.0)
- **caster_wheel**: 0.025m radius sphere at rear, low friction (μ=0.001)
- **laser_frame**: 0.05m radius cylinder (LiDAR puck, top-front of chassis)

---

## 📡 Topics & Transforms

### Robot Control
- **Input**: `/diff_cont/cmd_vel` (geometry_msgs/TwistStamped)
  - Consumed by `diff_drive_controller`; remapped from `/cmd_vel` if using teleop

### State Publishing
- **Output**: `/joint_states` (sensor_msgs/JointState)
  - Published by `joint_state_broadcaster` (wheel encoders)
- **Output**: `/diff_cont/odom` (nav_msgs/Odometry)
  - Odometry from wheel velocities + odometry → base_link TF
- **Output**: `/tf` (geometry_msgs/TransformStamped)
  - TF tree (odom → base_link → wheels, chassis, laser)

### Sensing
- **Output**: `/scan` (sensor_msgs/LaserScan)
  - 360° LiDAR, 10 Hz, range 0.3–12.0 m
  - Bridged from Gazebo `/scan` topic via ros_gz_bridge

### System
- **Input**: `/clock` (rosgraph_msgs/Clock)
  - Gazebo simulation time, bridged to ROS via ros_gz_bridge
  - All nodes use `use_sim_time: true` to synchronize

---

## 🔧 Key ROS 2 Components

### Controllers (gz_ros2_control)
- **diff_cont** (DiffDriveController): Velocity commands → wheel joint velocities
  - Consumes: `/diff_cont/cmd_vel` (TwistStamped)
  - Publishes: `/diff_cont/odom` + `odom → base_link` TF
- **joint_broad** (JointStateBroadcaster): Wheel state → `/joint_states`

### Bridges (ros_gz_bridge)
- `/clock`: Gazebo simulation time → ROS `/clock`
- `/scan`: Gazebo LiDAR rays → ROS `sensor_msgs/LaserScan`

---

## 🛠️ Building

```bash
cd ~/abhi_ros2_ws
colcon build --packages-select mobile_description
source install/setup.bash
```

---

## 📚 Learning Resources

- **URDF**: User-defined robot format (XML specification for links, joints, collision, inertia)
- **Xacro**: XML macro language for parameterized/reusable URDF snippets
- **ros2_control**: ROS 2 framework for hardware interfacing and controller management
- **Gazebo Harmonic**: Physics simulation engine with ROS 2 plugins
- **RViz2**: 3D visualization tool for transforms, sensor data, and robot models
- **robot_state_publisher**: Broadcasts TF tree from URDF + `/joint_states`

---

## 🐛 Troubleshooting

### Robot doesn't move when I publish to `/diff_cont/cmd_vel`
- Ensure `diff_cont` spawner has started. Check:
  ```bash
  ros2 controller list
  ```
  Should show `diff_cont [diff_drive_controller/DiffDriveController]` as *active*.

### `/scan` topic is empty or not bridging
- Verify `ros_gz_bridge` is running:
  ```bash
  ros2 node list | grep bridge
  ```
- Check Gazebo: GPU LiDAR sensor should be visualized as green rays (if `visualize: true`).

### RViz shows TF tree but robot doesn't move
- Verify `/joint_states` is being published:
  ```bash
  ros2 topic echo /joint_states --once
  ```

### Gazebo crashes on startup
- Check `GZ_SIM_RESOURCE_PATH` includes all model directories:
  ```bash
  echo $GZ_SIM_RESOURCE_PATH
  ```
  Should contain all `config/env/*/models` paths.

---

## 📝 Notes

- **Odometry Drift**: Wheel-only odometry drifts over long distances. Use LiDAR-based localization (Nav2 + SLAM) for accurate navigation.
- **Physics Timestep**: Gazebo runs at 30 Hz control update rate. Controller publishes odometry at 50 Hz.
- **Environment Auto-Resolution**: World file selection is automatic (first `.world` or `.sdf` in `config/env/<env_name>/worlds/`). No manual world argument needed.

---

## 🎓 Next Steps

1. **Autonomous Navigation**: Integrate Nav2 for SLAM-based mapping and autonomous movement.
2. **Sensor Fusion**: Add IMU, wheel odometry filtering via Extended Kalman Filter (robot_localization).
3. **Path Planning**: Implement Dijkstra / A* on the occupancy grid for navigation.
4. **Multi-Robot**: Extend to support swarms with unique namespaces (`robot_1/`, `robot_2/`).
