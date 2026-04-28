# abhi_ros2_ws

ROS 2 workspace for learning robot description and visualization.

## Overview

This workspace contains `mobile_description` package: a simple 2-wheel differential drive robot with caster wheel.

## Package Structure

```
mobile_description/
├── urdf/
│   └── mobile_robot.urdf.xacro   # Robot structure (links, joints, properties)
├── launch/
│   └── rsp.launch.py             # Robot State Publisher (broadcasts TF tree)
├── rviz/                         # Visualization configs
├── meshes/                       # 3D models (if any)
├── CMakeLists.txt               # Build config
└── package.xml                  # Package dependencies
```

## Robot Structure

**Robot Tree:**
```
base_link (reference frame)
├── chassis (blue box body)
├── left_wheel (cylinder, rotates on Y-axis)
└── right_wheel (cylinder, rotates on Y-axis)
```

**Components:**
- **base_link**: Reference frame (no geometry)
- **chassis**: Main body (0.4m × 0.3m × 0.15m box)
- **wheels**: 0.05m radius cylinders

## Building and Running

### Build workspace
```bash
cd ~/abhi_ros2_ws
colcon build
source install/setup.bash
```

### Launch robot visualization
```bash
ros2 launch mobile_description rsp.launch.py
```

### View in RViz
```bash
rviz2
```
Add "RobotModel" display and set Fixed Frame to "base_link"

## Learning Resources

- **URDF**: User-defined format for robot structure
- **Xacro**: XML macros for reusable URDF code
- **robot_state_publisher**: Broadcasts transform tree (TF)
- **RViz**: ROS visualization tool for transforms and models

## Files

- `mobile_robot.urdf.xacro`: Defines robot using Xacro macros (properties, links, joints, wheel macro)
- `rsp.launch.py`: Launch file that loads URDF and starts robot_state_publisher node

## Key Concepts

- **Link**: Physical part of robot (chassis, wheels)
- **Joint**: Connection between links (fixed or rotating)
- **Fixed Joint**: Links move together (chassis ↔ base_link)
- **Continuous Joint**: Unlimited rotation (wheels)
- **Macro**: Template for repeated structures (wheel macro)
- **TF Tree**: Coordinate frame hierarchy (how parts relate spatially)
