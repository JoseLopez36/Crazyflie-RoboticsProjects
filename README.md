# Crazyflie Robotics Projects

ROS2 workspace for Crazyflie drone simulation and development using Gazebo.

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
source ~/.bashrc
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
- **`run_sim.sh`** - Launch simulations (single, slam, or nav)

**Examples:**
```bash
# Build workspace
~/Crazyflie-RoboticsProjects/tools/build.sh

# Run simulation with Navigation2
~/Crazyflie-RoboticsProjects/tools/run_sim.sh
```

