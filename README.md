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
$ROBOTICSPROJECTS_CRAZYFLIE_PATH/docker/build.sh # This might take a while
```

### Run the Container (includes GUI support)

For basic usage:
```bash
$ROBOTICSPROJECTS_CRAZYFLIE_PATH/docker/run.sh
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

3. **Start Gazebo simulation:**:
   ```bash
   $HOME/Crazyflie-RoboticsProjects/tools/run_simulation.sh
   ```

3. **Run Crazyflie firmware SITL instances**:
   ```bash
   cd ~/crazyflie-firmware/scripts/sim_cf2
   ./run_cfs.sh 1  # Runs 1 instance
   ```

**5**. **Run example script**:
   ```bash
   cd ~/crazyflie-lib-python/examples/sim_cf2
   python autonomousSequence.py
   ```

## Related Repositories

- [sim_cf2](https://github.com/CrazyflieTHI/sim_cf2) - ROS2 Gazebo Flight Simulator for Crazyflie
- [Crazyflie Firmware (SITL)](https://github.com/CrazyflieTHI/crazyflie-firmware) - Modified firmware for SITL
- [Crazyflie Python Library](https://github.com/CrazyflieTHI/crazyflie-lib-python) - Python library with simlink driver

## Development

### Custom Package: crazyflie_rp_pkg

The `ros2_ws/src/crazyflie_rp_pkg` package provides:
- Launch files for sim_cf2 with one simulated Crazyflie
- SLAM Toolbox integration for mapping
- Navigation2 integration for autonomous navigation
- Configuration files for SLAM and navigation

See `ros2_ws/src/crazyflie_rp_pkg/README.md` for detailed usage instructions.

### Utility Scripts

Simple scripts in `tools/` directory:
- **`build.sh`** - Build the ROS2 workspace
- **`run_sim_cf2.sh`** - Launch Crazyflie SITL simulation (sim_cf2) with automatic setup
- **`run_hardware.sh`** - Launch Python scripts with real Crazyflie hardware

#### Simulation Examples:
```bash
# Build workspace
~/Crazyflie-RoboticsProjects/tools/build.sh

# Run Crazyflie SITL simulation (automatically sources workspace, starts Gazebo, runs SITL instances, and unpauses)
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh

# Run with 2 Crazyflie instances
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh "" 2

# Run simulation and execute a Python script at the end
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh takeoff_land.py

# Run with custom script and 2 instances
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh takeoff_land.py 2
```

**Note:** `run_sim_cf2.sh` automatically:
- Sources the ROS2 workspace
- Launches Gazebo simulation
- Runs Crazyflie firmware SITL instances
- Unpauses Gazebo after firmware initialization
- Optionally executes a user-provided script/command at the end

#### Real Hardware Examples:
```bash
# Run takeoff_land.py with real hardware (default script)
~/Crazyflie-RoboticsProjects/tools/run_hardware.sh

# Run takeoff_land.py with real hardware
~/Crazyflie-RoboticsProjects/tools/run_hardware.sh takeoff_land.py

# Run position_control.py with real hardware
~/Crazyflie-RoboticsProjects/tools/run_hardware.sh position_control.py

# Run with specific Crazyflie ID (if multiple drones are available)
~/Crazyflie-RoboticsProjects/tools/run_hardware.sh takeoff_land.py --cf-id 2
```

**Note:** `run_hardware.sh` automatically:
- Checks for radio dongle availability
- Forces hardware mode (`--hardware` flag)
- Connects to the first available Crazyflie (or specified `--cf-id`)

### Python Scripts for Simulation and Hardware

The `scripts/` directory contains Python scripts that work seamlessly with both **simulation (Gazebo)** and **real hardware**. These scripts automatically detect the connection mode and provide a unified API.

**Key Features:**
- Automatic detection of simulation vs hardware mode
- Same code works for both simulation and hardware
- Example scripts for common operations
- Support for multiple Crazyflie drones

**Available Scripts:**
- **`takeoff_land.py`** - Basic takeoff, hover, and land sequence
- **`position_control.py`** - Position control with square pattern flight