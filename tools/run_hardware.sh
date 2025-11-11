#!/bin/bash

# Script to run Crazyflie scripts with real hardware
# Usage: run_hardware.sh [script_name] [script_args...]
#   script_name: Script to execute (default: takeoff_land.py)
#   script_args: Additional arguments to pass to the script

PROJECT_ROOT="$HOME/Crazyflie-RoboticsProjects"
SCRIPTS_DIR="$PROJECT_ROOT/scripts"

# Parse arguments
SCRIPT_NAME="${1:-takeoff_land.py}"
shift  # Remove first argument, rest are script args

echo "=== Crazyflie Hardware Launcher ==="
echo ""

# Check if script exists
if [ ! -f "$SCRIPTS_DIR/$SCRIPT_NAME" ]; then
    echo "Error: Script not found: $SCRIPTS_DIR/$SCRIPT_NAME"
    echo ""
    echo "Available scripts:"
    ls -1 "$SCRIPTS_DIR"/*.py 2>/dev/null | xargs -n1 basename
    exit 1
fi

# Check if radio dongle is available
echo "Checking for Crazyflie radio dongle..."
python3 -c "
import cflib.crtp
cflib.crtp.init_drivers()
available = cflib.crtp.scan_interfaces()
radio_available = any(interface for interface in available if 'radio://' in str(interface))
if not radio_available:
    print('Warning: No radio dongle detected. Make sure your Crazy Radio is connected.')
    print('Available interfaces:', [str(i) for i in available])
    exit(1)
else:
    print('Radio dongle detected.')
" || {
    echo ""
    echo "Error: No radio dongle detected. Please connect your Crazy Radio dongle."
    exit 1
}

echo ""

# Execute the script with hardware flag
echo "Executing: python3 $SCRIPTS_DIR/$SCRIPT_NAME --hardware $*"
echo ""
python3 "$SCRIPTS_DIR/$SCRIPT_NAME" --hardware "$@"

SCRIPT_EXIT_CODE=$?

if [ $SCRIPT_EXIT_CODE -eq 0 ]; then
    echo ""
    echo "Script finished successfully."
else
    echo ""
    echo "Script finished with exit code $SCRIPT_EXIT_CODE."
fi

exit $SCRIPT_EXIT_CODE