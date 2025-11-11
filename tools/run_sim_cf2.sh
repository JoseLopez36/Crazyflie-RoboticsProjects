#!/bin/bash

# Script to run Crazyflie SITL simulation (sim_cf2)
# Usage: run_sim_cf2.sh [script_to_execute] [num_instances]
#   script_to_execute: Optional script to execute at the end
#   num_instances: Optional number of CF instances (default: 1)

PROJECT_ROOT="$HOME/Crazyflie-RoboticsProjects"
ROS2_WS="$PROJECT_ROOT/ros2_ws"
FIRMWARE_DIR="$HOME/crazyflie-firmware"
PYTHON_LIB_DIR="$HOME/crazyflie-lib-python"
SCRIPTS_DIR="$PROJECT_ROOT/scripts"

# Parse arguments
SCRIPT_TO_EXECUTE="$1"
NUM_CF_INSTANCES=${2:-1}  # Default to 1 instance if not provided

echo "=== Crazyflie SITL Simulation Launcher ==="
echo ""

# Step 1: Source ROS2 workspace
echo "[1/4] Sourcing ROS2 workspace..."
if [ ! -f "$ROS2_WS/install/local_setup.bash" ]; then
    echo "Error: ROS2 workspace not built. Please run build.sh first."
    exit 1
fi
source "$ROS2_WS/install/local_setup.bash"
echo "✓ ROS2 workspace sourced"
echo ""

# Step 2: Start Gazebo simulation
echo "[2/4] Starting Gazebo simulation..."
echo "Note: Gazebo will start in paused mode and automatically unpause after SITL instances connect."
echo ""

# Launch Gazebo in background
ros2 launch sim_cf2 main.launch.xml &
GAZEBO_PID=$!

# Wait a bit for Gazebo to start
echo "Waiting for Gazebo to initialize..."
sleep 5

# Check if Gazebo process is still running
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "Error: Gazebo failed to start"
    exit 1
fi

echo "✓ Gazebo simulation started (PID: $GAZEBO_PID)"
echo ""

# Step 3: Run Crazyflie firmware SITL instances
echo "[3/4] Running Crazyflie firmware SITL instances ($NUM_CF_INSTANCES instance(s))..."
if [ ! -d "$FIRMWARE_DIR/scripts/sim_cf2" ]; then
    echo "Error: Firmware directory not found at $FIRMWARE_DIR"
    echo "Please ensure the firmware is cloned and built."
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

cd "$FIRMWARE_DIR/scripts/sim_cf2"
if [ ! -f "./run_cfs.sh" ]; then
    echo "Error: run_cfs.sh not found in firmware scripts directory"
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

./run_cfs.sh $NUM_CF_INSTANCES &
SITL_PID=$!

echo "✓ SITL instances started (PID: $SITL_PID)"
echo ""

# Wait for SITL instances to connect to Gazebo
echo "Waiting for SITL instances to connect to Gazebo..."
sleep 3

# Automatically unpause Gazebo
echo "Unpausing Gazebo simulation..."
ros2 service call /unpause_physics std_srvs/srv/Empty
echo "✓ Gazebo simulation unpaused"
echo ""

# Step 4: Execute user-provided script if specified
if [ -n "$SCRIPT_TO_EXECUTE" ]; then
    echo "[4/4] Executing user script: $SCRIPT_TO_EXECUTE"
    RUN_SCRIPT=true
    SCRIPT_CMD="python $PROJECT_ROOT/scripts/$SCRIPT_TO_EXECUTE"
else
    echo "[4/4] No script specified to execute"
    RUN_SCRIPT=false
fi

echo ""
echo "=== Simulation Running ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "SITL PID: $SITL_PID"
echo ""

# Set up cleanup trap
cleanup() {
    echo ""
    echo "Stopping simulation..."
    kill $GAZEBO_PID $SITL_PID 2>/dev/null || true
    wait $GAZEBO_PID $SITL_PID 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM

# Execute script if provided, otherwise wait for user interrupt
if [ "$RUN_SCRIPT" = true ]; then
    sleep 2
    echo "Executing script. Press Ctrl+C to stop all processes..."
    echo ""
    # Execute the script/command
    python3 "$SCRIPTS_DIR/$SCRIPT_TO_EXECUTE" --sim --uri "radio://0/80/2M/E7E7E7E701"
    SCRIPT_EXIT_CODE=$?
    echo ""
    if [ $SCRIPT_EXIT_CODE -eq 0 ]; then
        echo "Script finished successfully. Gazebo and SITL are still running."
    else
        echo "Script finished with exit code $SCRIPT_EXIT_CODE. Gazebo and SITL are still running."
    fi
    echo "Press Ctrl+C to stop all processes..."
    wait $GAZEBO_PID $SITL_PID
else
    echo "Gazebo and SITL are running. Press Ctrl+C to stop all processes..."
    wait $GAZEBO_PID $SITL_PID
fi