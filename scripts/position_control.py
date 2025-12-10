#!/usr/bin/env python3
"""
Position Control Example
Demonstrates position control by flying a square pattern.
The square starts at the initial position (top-left corner).
Works with both simulation and real hardware.
"""

import time
import sys
import os
import argparse

# Add scripts directory to path if needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from crazyflie_utils import CrazyflieConnection, detect_connection_mode
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# The trajectory to fly
sequence = [
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 1.2, 0),
    (0.5, -0.5, 1.2, 0),
    (0.5, 0.5, 1.2, 0),
    (-0.5, 0.5, 1.2, 0),
    (-0.5, -0.5, 1.2, 0),
    (0.0, 0.0, 1.2, 0),
    (0.0, 0.0, 0.4, 0),
]


# Global variable to store position
current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

def log_position_callback(timestamp, data, logconf):
    """Callback for logging position data"""
    global current_position
    current_position['x'] = data['stateEstimate.x']
    current_position['y'] = data['stateEstimate.y']
    current_position['z'] = data['stateEstimate.z']
    print(f"[{timestamp}] Position: X={data['stateEstimate.x']:.3f} m, "
          f"Y={data['stateEstimate.y']:.3f} m, Z={data['stateEstimate.z']:.3f} m")

def run_sequence(cf, sequence):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    duration = 2.0
    relative = True
    for position in sequence:
        commander.go_to(position[0],
                        position[1],
                        position[2],
                        position[3],
                        duration,
                        relative)
        time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def main():
    parser = argparse.ArgumentParser(description='Crazyflie position control')
    parser.add_argument('--sim', action='store_true', help='Force simulation mode')
    parser.add_argument('--hardware', action='store_true', help='Force hardware mode')
    parser.add_argument('--uri', type=str, default="radio://0/80/2M/E7E7E7E701", help='Crazyflie URI to connect to (default: radio://0/80/2M/E7E7E7E701)')
    args = parser.parse_args()
    
    print("=== Crazyflie Position Control ===\n")
    
    # Detect connection mode
    mode = detect_connection_mode(force_sim=args.sim, force_hardware=args.hardware)
    print(f"Detected mode: {mode}\n")
    
    # Connect to Crazyflie
    print(f"Connecting to Crazyflie at {args.uri}...")
    with CrazyflieConnection(mode=mode, uri=args.uri, force_sim=args.sim, force_hardware=args.hardware) as connection:
        if connection is None:
            print("Failed to connect!")
            return

        print("Connected successfully!\n")

        # Get cf
        cf = connection.scf.cf

        # Set up logging for position data
        log_conf = LogConfig(name='Position Data', period_in_ms=1000)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        
        try:
            cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(log_position_callback)
            log_conf.start()

            # Reset estimator and run sequence
            connection.reset_estimator()
            run_sequence(cf, sequence)

            # Stop logging
            log_conf.stop()
            time.sleep(0.1)

        except Exception as e:
            print(f"Error during flight: {e}")
            import traceback
            traceback.print_exc()
            # Emergency stop
            try:
                cf.high_level_commander.stop()
            except:
                pass
        
        print("\nDone!")

if __name__ == '__main__':
    main()
