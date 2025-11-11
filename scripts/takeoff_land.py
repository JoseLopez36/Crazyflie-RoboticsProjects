#!/usr/bin/env python3
"""
Takeoff and Land Example
Demonstrates basic flight control: takeoff, hover, and land.
Works with both simulation and real hardware.
"""

import time
import sys
import os
import argparse

# Add scripts directory to path if needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from crazyflie_utils import CrazyflieConnection, detect_connection_mode
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# Global variable to store position
current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
initial_position = {'x': None, 'y': None, 'z': None}

def log_position_callback(timestamp, data, logconf):
    """Callback for logging position data"""
    global current_position
    current_position['x'] = data['stateEstimate.x']
    current_position['y'] = data['stateEstimate.y']
    current_position['z'] = data['stateEstimate.z']
    print(f"[{timestamp}] Position: X={data['stateEstimate.x']:.3f} m, "
          f"Y={data['stateEstimate.y']:.3f} m, Z={data['stateEstimate.z']:.3f} m")

def main():
    parser = argparse.ArgumentParser(description='Crazyflie takeoff and land example')
    parser.add_argument('--sim', action='store_true', help='Force simulation mode')
    parser.add_argument('--hardware', action='store_true', help='Force hardware mode')
    parser.add_argument('--uri', type=str, default="radio://0/80/2M/E7E7E7E701", help='Crazyflie URI to connect to (default: radio://0/80/2M/E7E7E7E701)')
    args = parser.parse_args()
    
    print("=== Crazyflie Takeoff and Land Example ===\n")
    
    # Detect connection mode
    mode = detect_connection_mode(force_sim=args.sim, force_hardware=args.hardware)
    print(f"Detected mode: {mode}\n")
    
    # Connect to Crazyflie
    print(f"Connecting to Crazyflie #{args.uri}...")
    with CrazyflieConnection(mode=mode, uri=args.uri, force_sim=args.sim, force_hardware=args.hardware) as connection:
        if connection is None:
            print("Failed to connect!")
            return

        print("Connected successfully!\n")

        # Set up logging for position data
        log_conf = LogConfig(name='Position Data', period_in_ms=1000)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        
        try:
            connection.scf.cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(log_position_callback)
            log_conf.start()

            # Enable position control
            print("Enabling position control...")
            connection.scf.cf.param.set_value('flightmode.posSet', '1')
            time.sleep(0.1)
            
            # Reset estimator and wait for it to stabilize
            connection.reset_estimator()
            
            # Store initial position (X, Y) from logged position estimate
            global initial_position
            initial_position['x'] = current_position['x']
            initial_position['y'] = current_position['y']
            initial_position['z'] = current_position['z']
            
            print(f"\nInitial position logged:")
            print(f"  X: {initial_position['x']:.3f} m")
            print(f"  Y: {initial_position['y']:.3f} m")
            print(f"  Z: {initial_position['z']:.3f} m")
            
            # Takeoff - continuously send position commands
            print("\n=== Takeoff Sequence ===")
            print(f"Taking off...")
            connection.scf.cf.high_level_commander.takeoff(1.0, 2.0)
            time.sleep(3.0)

            # Hover at the same X, Y position
            # for i in range(100):
            #     connection.scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.5)
            #     time.sleep(0.1)
            # time.sleep(2.0)

            # Land sequence - return to initial position
            print(f"Landing...")
            connection.scf.cf.high_level_commander.land(0.0, 3.0)
            time.sleep(3.0)

            log_conf.stop()
            connection.scf.cf.high_level_commander.stop()
            time.sleep(0.1)
            
        except Exception as e:
            print(f"Error during flight: {e}")
            import traceback
            traceback.print_exc()
            # Emergency stop
            try:
                connection.scf.cf.commander.send_stop_setpoint()
            except:
                pass
        
        print("\nDone!")

if __name__ == '__main__':
    main()

