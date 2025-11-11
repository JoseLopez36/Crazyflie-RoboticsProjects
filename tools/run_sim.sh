#!/bin/bash

# Script to run simulations
# Usage: ./run_sim.sh [single|slam|nav]
#   single - Basic simulation with one Crazyflie
#   slam   - Simulation with SLAM
#   nav    - Simulation with SLAM and Navigation2

# Default to single if no argument provided
SIM_TYPE="${1:-single}"

# Map argument to launch file
case "$SIM_TYPE" in
    single)
        LAUNCH_FILE="sim_cf2_single.launch.py"
        ;;
    slam)
        LAUNCH_FILE="sim_cf2_with_slam.launch.py"
        ;;
    nav)
        LAUNCH_FILE="sim_cf2_with_nav.launch.py"
        ;;
    *)
        echo "Usage: $0 [single|slam|nav]"
        echo "  single - Basic simulation with one Crazyflie"
        echo "  slam   - Simulation with SLAM"
        echo "  nav    - Simulation with SLAM and Navigation2"
        exit 1
        ;;
esac

echo "Launching $LAUNCH_FILE..."
cd $HOME/Crazyflie-RoboticsProjects/ros2_ws
source $HOME/Crazyflie-RoboticsProjects/ros2_ws/install/setup.bash
ros2 launch crazyflie_rp_pkg $LAUNCH_FILE