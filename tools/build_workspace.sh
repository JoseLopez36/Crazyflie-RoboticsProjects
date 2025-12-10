#!/bin/bash

# Script to build the ROS2 workspace
echo "Building ROS2 workspace..."
cd /root/Crazyflie-RoboticsProjects/ros2_ws
colcon build --symlink-install --merge-install
echo "Build complete! Source now!"