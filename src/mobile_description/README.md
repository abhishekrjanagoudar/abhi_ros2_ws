# 🤖 mobile_description

The `mobile_description` package contains launch files and configurations to simplify and speed up starting the differential-drive mobile robot — whether in simulation (Gazebo Harmonic) or on the real hardware.

---

## 🚀 Quick Start

To launch the full simulation stack (Gazebo + RViz + controllers), run:

```bash
ros2 launch mobile_description robot.launch.py
```

To also open the Gazebo GUI:

```bash
ros2 launch mobile_description robot.launch.py gz:=true
```

---

## ⚙️ Launch Arguments

You can customize the robot and environment with the following arguments:

### 1. `env_name`

Selects the Gazebo simulation environment:

- `cpr_office` *(default)*: Standard CPR office environment.
- `cpr_office_construction`: CPR office with construction obstacles.
- `office_small`: Compact office layout.
- `office_env_large`: Large open office floor.
- `office_earthquake`: Post-earthquake scenario.

---

### 2. `gz`

Enable/disable the graphical user interface of gazebo:

- `false` *(default)*: no GUI is launched (headless rendering).
- `true`: GUI is launched.

---

### 3. `ros_ui`

Select the type of the user interface:

- `true` *(default)*: RVIZ is launched, all ROS data can be accessed.
- `false`: NiceGUI (webgui), can be accessed in the browser. However, it has just limited options.

---

### 4. `rviz`

Launch RViz2 independently of `ros_ui`:

- `true` *(default)*: RViz is always started.
- `false`: RViz is skipped (only effective when `ros_ui:=false`).

---

### 5. `mode` (slam.launch.py)

Specifies the SLAM Toolbox operational mode:

- `async` *(default)*: Online asynchronous mapping (recommended, real-time).
- `sync`: Online synchronous mapping.
- `localization`: Localise against an existing serialized pose graph.
- `lifelong`: Continuous lifelong mapping.

---

### 6. `use_sim_time`

Controls time source:

- `true` *(default)*: Use simulation time (e.g., Gazebo).
- `false`: Use real robot clock.

---

### 🧪 Example

```bash
ros2 launch mobile_description robot.launch.py env_name:=office_env_large gz:=true

ros2 launch mobile_description slam.launch.py mode:=async use_sim_time:=true
```

---

## 🗺️ Mapping Workflow

1. **Start the simulation** with the robot at the map origin.

2. **Launch SLAM** in async mode (or your preferred mode), set `use_sim_time` depending on your setup:

```bash
ros2 launch mobile_description slam.launch.py mode:=async use_sim_time:=true
```

3. **Check the `/map` topic** and drive the robot to explore the environment using a joystick or keyboard:

```bash
ros2 run mobile_description teleop_controller.py
```

The custom teleop controller in this package automatically publishes to `/diff_cont/cmd_vel` using the correct QoS settings for the diff-drive controller. Keep focus on the teleop terminal and hold `w` to move forward.

4. **Save the map** to your target directory:

```bash
ros2 run nav2_map_server map_saver_cli -f <map-name>
```

> 💡 This produces `<map-name>.yaml` and `<map-name>.pgm`.

---

## 📍 Localization Guide

Localization allows the robot to determine its pose within a known map. The `mobile_description` stack supports two primary methods:

1. **AMCL (Adaptive Monte Carlo Localization)**

Using Nav2 directly with a static map:

```bash
ros2 launch mobile_description navigation.launch.py use_sim_time:=true
```

`Mechanism:` Uses the particle filter method to match real-time laser scans against a static 2D occupancy grid.

`Required Files:` Requires a .yaml and a .pgm (or .png) image file.

2. **SLAM Toolbox Localization**

Used for high-precision localization using the serialized pose graph from a previous mapping session:

```bash
ros2 launch mobile_description slam.launch.py mode:=localization use_sim_time:=true
```

`Mechanism:` Matches current LIDAR data against the optimized pose graph rather than a static image.

`Required Files:` Requires both .posegraph and .data files in `config/nav2/`.

> 💡 For localization to function correctly, ensure the `.posegraph` and `.data` files are appropriately referenced in `mapper_params_localization.yaml`.

---

## 📦 Bag File Management

To reduce hardware dependency and setup time, always use ROS 2 Bags for parameter tuning and drift validation.

1. **Recording a Bag**

To record a high-quality dataset for mapping, use the following syntax. These topics are mandatory for `slam_toolbox` mapping:

```bash
ros2 bag record -o <bag_name> /tf /tf_static /scan /diff_cont/odom /joint_states /clock
```

`/tf` & `/tf_static`: Robot kinematic transforms.
`/scan`: LiDAR data from `laser_frame`.
`/diff_cont/odom`: Wheel odometry.
`/joint_states`: Wheel encoder positions.
`/clock`: Simulation time.

2. **Playing a Bag**

Always use the `--clock` flag when replaying data for SLAM to avoid time-sync errors:

```bash
ros2 bag play <bag_directory> --clock -r 1.0
```

---

## 🗺️ Save and Deploy the Map

You must run two commands to save the map fully. This ensures you have both a visual map for navigation and the internal SLAM state for future editing.

1. **Save Static Map (for AMCL Localization)**

```bash
ros2 run nav2_map_server map_saver_cli -f <map-name>
```
`Required Files:` .yaml and .pgm (or .png).

`Deployment:` Copy these to `config/env/<env_name>/maps/`

2. **Serialize Pose Graph (for SlamToolbox Localization)**

SlamToolbox localization requires the serialized internal state.

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<map-name>'}"
```
`Required Files:` .posegraph and .data.

`Deployment:` These files must be moved to `config/env/<env_name>/maps/`.

---

## Notes

1. The `env_name` argument controls both the Gazebo world file and the model asset paths. Each environment folder must contain a `worlds/` subdirectory with exactly one `.world` or `.sdf` file.
2. The spawn `z` offset of `0.1 m` is intentional: `wheel_zoff (0.05 m) + wheel_radius (0.05 m) = 0.1 m` keeps the chassis clear of the ground plane without gap.
3. When using SLAM Toolbox in `localization` mode, the `.posegraph` and `.data` files must be referenced correctly in `mapper_params_localization.yaml`.
4. NiceGUI web UI requires `pip install nicegui` and a user-supplied `src/nicegui_app.py` script; the node is silently skipped if either is absent.
