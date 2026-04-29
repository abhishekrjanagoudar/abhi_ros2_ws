#!/bin/bash
# Interactive teleop launcher
# Run with: bash launch/teleop_interactive.sh

source /home/abhishek-janagoudar/abhi_ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/diff_cont/cmd_vel
