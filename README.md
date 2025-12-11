# Crazyflie Robotics Projects

This project is part of the **Robotics Projects** subject in the MSc. in Electronics, Robotics and Automation Engineering. It focuses on working with the Crazyflie 2.x drone platform in both simulation and real-world environments. The project uses the Crazyflie 2.0 for simulation and the Crazyflie 2.1 for real hardware, equipped with the multi-ranger deck (available in both simulation and real hardware) and optical flow sensor (real hardware only).

## Overview

This repository contains a Docker-based development environment for working with the Crazyflie drone using ROS2 and Gazebo simulation. It is based on the [crazyswarm2](https://github.com/IMRCLab/crazyswarm2) simulator, which provides a full software-in-the-loop (SITL) implementation of the Crazyflie firmware.

## Project Structure

- docker/ : Docker configuration files
- ros2_ws/: ROS2 workspace
- tools/: Utility scripts

## Prerequisites

### For Simulation
- Docker installed and running
- X11 forwarding support (for GUI applications like Gazebo)

### For Real Hardware
- Docker installed and running
- Crazyflie 2.1 drone
- Crazy Radio PA or Crazy Radio dongle connected to your computer

## Quick Start

### Set project environmental variable
```bash
export CRAZYFLIE_ROBOTICSPROJECTS_PATH="/absolute/path/to/this/repo/root/on/host"
```

### Build the Docker Image

```bash
$CRAZYFLIE_ROBOTICSPROJECTS_PATH/docker/build.sh # This might take a while
```

### Run the Container (includes GUI support)

For basic usage:
```bash
$CRAZYFLIE_ROBOTICSPROJECTS_PATH/docker/run.sh
```

### Inside the Container

Once inside the container:
1. **Build ROS2 Workspace:**
   ```bash
   $HOME/Crazyflie-RoboticsProjects/tools/build_workspace.sh
   ```

2. **Source ROS2 Workspace:**
   ```bash
   $HOME/Crazyflie-RoboticsProjects/tools/source_workspace.sh
   ```

### Run Simulation

**Start Gazebo simulation:**:
```bash
$HOME/Crazyflie-RoboticsProjects/tools/run_simulation.sh
```

### Run Hardware

**Start Hardware:**:
```bash
$HOME/Crazyflie-RoboticsProjects/tools/run_hardware.sh
```