# Crazyflie Robotics Projects

This project is part of the **Robotics Projects** subject in the MSc. in Electronics, Robotics and Automation Engineering. It focuses on working with the Crazyflie 2.x drone platform in both simulation and real-world environments. The project uses the Crazyflie 2.0 for simulation and the Crazyflie 2.1 for real hardware, equipped with the multi-ranger deck (available in both simulation and real hardware) and optical flow sensor (real hardware only).

## Overview

This repository contains a Docker-based development environment for working with the Crazyflie drone using ROS2 and Gazebo simulation. It is based on the [sim_cf2](https://github.com/CrazyflieTHI/sim_cf2) simulator, which provides a full software-in-the-loop (SITL) implementation of the Crazyflie firmware.

## Project Structure

- docker/ : Docker configuration files
- ros2_ws/: ROS2 workspace
- tools/: Utility scripts

## Prerequisites

- Docker installed and running
- X11 forwarding support (for GUI applications like Gazebo)

## Quick Start

### Set project environmental variable
```bash
echo "export CRAZYFLIE_ROBOTICS_PROJECTS=/absolute/path/to/repo" >> ~/.bashrc
source ~/.bashrc # Sets it permanently
```

### Build the Docker Image

```bash
docker build -t crazyflie-robotics-projects -f $CRAZYFLIE_ROBOTICS_PROJECTS/docker/Dockerfile .
```

### Run the Container (includes GUI support)

For basic usage:
```bash
$CRAZYFLIE_ROBOTICS_PROJECTS/docker/run.sh
```

### Inside the Container

Once inside the container:
1. **Build Crazyflie firmware SITL:**
   ```bash
   cd ~/crazyflie-firmware
   make menuconfig
   ```
   - In the gui interface of menuconfig first navigate to Build and debug options
   - Select Build for SITL
   - Switch to Platform configuration
   - Select Build for SITL in the Platform to build window
   - In the menu entry Expansion deck configuration make sure no decks are activated
   - Save the configuration file and exit menuconfig

   ```bash
   make -j
   ```

2. **Start Gazebo simulation:**:
   ```bash
   source ~/Crazyflie-RoboticsProjects/ros2_ws/install/local_setup.bash
   ros2 launch sim_cf2 main.launch.xml
   ```

3. **Run Crazyflie firmware SITL instances**:
   ```bash
   cd ~/crazyflie-firmware/scripts/sim_cf2
   ./run_cfs.sh 1  # Runs 1 instance
   ```

4. **Run example script**:
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

**Examples:**
```bash
# Build workspace
~/Crazyflie-RoboticsProjects/tools/build.sh

# Run Crazyflie SITL simulation (automatically sources workspace, starts Gazebo, runs SITL instances, and unpauses)
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh

# Run with 2 Crazyflie instances
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh "" 2

# Run simulation and execute a Python script at the end
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh ~/crazyflie-lib-python/examples/sim_cf2/autonomousSequence.py

# Run with custom script and 2 instances
~/Crazyflie-RoboticsProjects/tools/run_sim_cf2.sh ~/my_script.py 2
```

**Note:** `run_sim_cf2.sh` automatically:
- Sources the ROS2 workspace
- Launches Gazebo simulation
- Runs Crazyflie firmware SITL instances
- Unpauses Gazebo after firmware initialization
- Optionally executes a user-provided script/command at the end

