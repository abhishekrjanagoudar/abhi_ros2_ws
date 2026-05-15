# 🤖 mobile_description

The `mobile_description` package contains the URDF robot model, launch files, and configurations to simulate and navigate a **differential-drive mobile robot** in **ROS 2 Jazzy + Gazebo Harmonic**.

It brings together the full simulation stack — Gazebo world, robot spawning, ros2_control controllers, ROS–Gazebo topic bridge, SLAM Toolbox mapping, and Nav2 navigation — from a single entry-point launch file.

---

## 🏗️ Robot Overview

| Property | Value |
|---|---|
| Drive type | Differential drive |
| Body | 0.4 × 0.3 × 0.15 m box (chassis) |
| Drive wheels | `left_wheel` / `right_wheel` — radius 0.05 m, continuous joints |
| Rear support | `caster_wheel` — passive sphere (μ=0.001) |
| LiDAR | `laser_frame` — GPU LiDAR, 360 rays, 10 Hz, 0.3–12.0 m, σ=0.01 m |
| Controllers | `joint_broad` (joint states) · `diff_cont` (differential drive) |
| Command topic | `/diff_cont/cmd_vel` (geometry_msgs/Twist) |
| Odometry topic | `/diff_cont/odom` (nav_msgs/Odometry) |
| Scan topic | `/scan` (sensor_msgs/LaserScan) |

### TF Tree

```
map
└── odom                  (published by diff_cont / SLAM Toolbox)
    └── base_link
        ├── chassis
        │   ├── caster_wheel
        │   └── laser_frame   ← LiDAR sensor
        ├── left_wheel
        └── right_wheel
```

---

## 🚀 Quick Start

Launch the full simulation stack (Gazebo + RViz + controllers):

```bash
ros2 launch mobile_description robot.launch.py
```

To also open the Gazebo GUI:

```bash
ros2 launch mobile_description robot.launch.py gz:=true
```

---

## ⚙️ Launch Files

| Launch File | Purpose |
|---|---|
| `robot.launch.py` | **Top-level entry point** — composes Gazebo + RViz/NiceGUI |
| `gazebo.launch.py` | Gazebo world, robot spawn, bridge, ros2_control controllers |
| `slam.launch.py` | SLAM Toolbox in async / sync / localization / lifelong mode |
| `navigation.launch.py` | Full Nav2 stack + SLAM Toolbox (autonomous navigation) |
| `rsp.launch.py` | Robot State Publisher — compiles URDF and broadcasts TF |
| `rviz.launch.py` | RViz2 with the default display configuration |
| `sim.launch.py` | Backward-compatible wrapper around `gazebo.launch.py` |

---

## ⚙️ Launch Arguments

### `robot.launch.py`

#### 1. `env_name`

Selects the Gazebo simulation environment. Maps to a subfolder under `config/env/`:

- `cpr_office` *(default)* — Standard CPR office environment
- `cpr_office_construction` — CPR office with construction obstacles
- `office_small` — Compact office layout
- `office_env_large` — Large open office floor
- `office_earthquake` — Post-earthquake scenario

---

#### 2. `gz`

Enable or disable the Gazebo GUI window:

- `false` *(default)* — Headless rendering (no GUI, lower resource use)
- `true` — Opens the Gazebo GUI

---

#### 3. `ros_ui`

Select the visualisation front-end:

- `true` *(default)* — Launches **RViz2** with full ROS data access
- `false` — Launches the **NiceGUI** web interface (browser-accessible at `localhost`), limited controls

> 💡 RViz requires a display (X11 / Wayland). On remote machines use `ros_ui:=false` or forward the display.

---

#### 4. `rviz`

Launch RViz2 independently of `ros_ui`:

- `true` *(default)* — RViz is always started
- `false` — RViz is skipped (only effective when `ros_ui:=false`)

---

### `slam.launch.py`

#### 5. `mode`

SLAM Toolbox operational mode:

- `async` *(default)* — Online asynchronous mapping (recommended, real-time)
- `sync` — Online synchronous (processes every scan; slower)
- `localization` — Localise against an existing serialized pose graph
- `lifelong` — Continuous lifelong mapping

---

#### 6. `use_sim_time`

Controls the time source across all nodes:

- `true` *(default)* — Use Gazebo `/clock` topic (simulation time)
- `false` — Use wall-clock (real robot deployment)

---

### 🧪 Examples

```bash
# Default: cpr_office, RViz on, headless Gazebo
ros2 launch mobile_description robot.launch.py

# Large office, Gazebo GUI open
ros2 launch mobile_description robot.launch.py env_name:=office_env_large gz:=true

# NiceGUI web UI only (no RViz), headless
ros2 launch mobile_description robot.launch.py ros_ui:=false rviz:=false

# SLAM async mapping session
ros2 launch mobile_description slam.launch.py mode:=async use_sim_time:=true
```

---

## 🗺️ Mapping Workflow

1. **Start the simulation** with the robot at the map origin.

2. **Launch SLAM** in async mode (or your preferred mode):

```bash
ros2 launch mobile_description slam.launch.py mode:=async use_sim_time:=true
```

3. **Drive the robot** to explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/diff_cont/cmd_vel
```

4. **Monitor the map** in RViz (add a `Map` display on topic `/map`).

5. **Save the static map** for AMCL / Nav2 localisation:

```bash
ros2 run nav2_map_server map_saver_cli -f <map-name>
```

> 💡 This produces `<map-name>.yaml` + `<map-name>.pgm`. Place them in `config/env/<env_name>/maps/`.

6. **Serialize the SLAM pose graph** for SLAM Toolbox localisation:

```bash
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '<map-name>'}"
```

> 💡 This produces `<map-name>.posegraph` + `<map-name>.data`. Also place in `config/env/<env_name>/maps/`.

---

## 📍 Localisation Guide

Run localisation (no new mapping) using SLAM Toolbox in localization mode:

```bash
ros2 launch mobile_description slam.launch.py mode:=localization use_sim_time:=true
```

**Required files:** `<map-name>.posegraph` + `<map-name>.data` in `config/nav2/`.

Alternatively, use **AMCL** via Nav2 directly with a static `.yaml` + `.pgm` map:

```bash
ros2 launch mobile_description navigation.launch.py use_sim_time:=true
```

---

## 🧭 Autonomous Navigation

The `navigation.launch.py` file starts the **complete Nav2 stack** alongside SLAM Toolbox:

```bash
ros2 launch mobile_description navigation.launch.py
```

To use a custom Nav2 parameter file:

```bash
ros2 launch mobile_description navigation.launch.py \
    params_file:=/path/to/my_nav2_params.yaml
```

Nav2 servers started automatically:

| Server | Role |
|---|---|
| `planner_server` | Global path planner (NavFn / Smac) |
| `controller_server` | Local trajectory follower (DWB / MPPI) |
| `bt_navigator` | Behaviour-tree goal execution |
| `recoveries_server` | Spin, backup, wait recovery behaviours |
| `waypoint_follower` | Sequential waypoint execution |

Send a goal from the RViz **2D Nav Goal** tool, or via the action interface:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 📦 Bag File Recording

Record the mandatory topics for SLAM / navigation replay:

```bash
ros2 bag record -o <bag_name> \
    /tf /tf_static \
    /scan \
    /diff_cont/odom \
    /joint_states \
    /clock
```

| Topic | Type | Purpose |
|---|---|---|
| `/tf` + `/tf_static` | tf2_msgs/TFMessage | Robot kinematic transforms |
| `/scan` | sensor_msgs/LaserScan | LiDAR data from `laser_frame` |
| `/diff_cont/odom` | nav_msgs/Odometry | Wheel odometry |
| `/joint_states` | sensor_msgs/JointState | Wheel encoder positions |
| `/clock` | rosgraph_msgs/Clock | Simulation time |

**Replay with clock:**

```bash
ros2 bag play <bag_directory> --clock -r 1.0
```

> ⚠️ Always use `--clock` when replaying bags against a SLAM node to avoid TF timestamp mismatches.

---

## 📁 Package Structure

```
mobile_description/
├── config/
│   ├── env/                        # Environment worlds + models
│   │   ├── cpr_office/
│   │   ├── cpr_office_construction/
│   │   ├── office_earthquake/
│   │   ├── office_env_large/
│   │   └── office_small/
│   ├── nav2/                       # Nav2 + SLAM Toolbox parameter files
│   │   ├── nav2_params.yaml
│   │   ├── mapper_params_online_async.yaml
│   │   ├── mapper_params_online_sync.yaml
│   │   └── mapper_params_localization.yaml
│   └── robot/                      # Robot model + controller configs
│       ├── mobile_robot.urdf.xacro
│       ├── my_controllers.yaml
│       └── ros_gz_bridge.yaml
├── launch/
│   ├── robot.launch.py             ← Top-level entry point
│   ├── gazebo.launch.py
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   ├── rsp.launch.py
│   ├── rviz.launch.py
│   └── sim.launch.py
├── rviz/
│   └── default.rviz
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Dependencies

| Package | Role |
|---|---|
| `xacro` | Compiles `mobile_robot.urdf.xacro` → URDF at launch time |
| `robot_state_publisher` | Broadcasts the TF tree from the URDF |
| `ros_gz_sim` | Gazebo Harmonic integration (spawn, bridge) |
| `gz_ros2_control` | Gazebo plugin for ros2_control hardware interface |
| `controller_manager` | Manages `joint_broad` + `diff_cont` controllers |
| `ros2_controllers` | Provides DiffDriveController + JointStateBroadcaster |
| `slam_toolbox` | Online mapping and localisation |
| `nav2_bringup` | Full autonomous navigation stack |

Build and source the workspace:

```bash
cd ~/abhi_ros2_ws
colcon build --symlink-install --packages-select mobile_description
source install/setup.bash
```

---

## Notes

1. The `env_name` argument controls both the Gazebo world file and the model asset paths. Each environment folder must contain a `worlds/` subdirectory with exactly one `.world` or `.sdf` file.
2. The spawn `z` offset of `0.1 m` is intentional: `wheel_zoff (0.05 m) + wheel_radius (0.05 m) = 0.1 m` keeps the chassis clear of the ground plane without gap.
3. When using SLAM Toolbox in `localization` mode, the `.posegraph` and `.data` files must be referenced correctly in `mapper_params_localization.yaml`.
4. NiceGUI web UI requires `pip install nicegui` and a user-supplied `src/nicegui_app.py` script; the node is silently skipped if either is absent.
