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
    parser.add_argument('--cf-id', type=int, default=1, help='Crazyflie ID to connect to (default: 1)')
    parser.add_argument('--hover-thrust', type=int, default=35000, help='Hover thrust value (default: 35000)')
    args = parser.parse_args()
    
    print("=== Crazyflie Takeoff and Land Example ===\n")
    
    # Detect connection mode
    mode = detect_connection_mode(force_sim=args.sim, force_hardware=args.hardware)
    print(f"Detected mode: {mode}\n")
    
    # Connect to Crazyflie
    print(f"Connecting to Crazyflie #{args.cf_id}...")
    with CrazyflieConnection(mode=mode, cf_id=args.cf_id, force_sim=args.sim, force_hardware=args.hardware) as connection:
        if connection is None:
            print("Failed to connect!")
            return

        print("Connected successfully!\n")

        # Set up logging for position data
        log_conf = LogConfig(name='Position Data', period_in_ms=100)
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
            
            # Wait a bit more to get stable position reading
            print("Waiting for stable position estimate...")
            time.sleep(2)
            
            # Store initial position (X, Y) from logged position estimate
            global initial_position
            initial_position['x'] = current_position['x']
            initial_position['y'] = current_position['y']
            initial_position['z'] = current_position['z']
            
            print(f"\nInitial position logged:")
            print(f"  X: {initial_position['x']:.3f} m")
            print(f"  Y: {initial_position['y']:.3f} m")
            print(f"  Z: {initial_position['z']:.3f} m")
            
            # Takeoff to 1.0 m - continuously send position commands
            print("\n=== Takeoff Sequence ===")
            print(f"Taking off to 1 m...")
            target_z = 1.0
            takeoff_duration = 3.0  # seconds
            takeoff_steps = int(takeoff_duration / 0.1)  # Send command every 0.1s
            
            for i in range(takeoff_steps):
                connection.scf.cf.commander.send_position_setpoint(
                    initial_position['x'], 
                    initial_position['y'], 
                    target_z, 
                    0
                )
                time.sleep(0.1)
            
            # Verify we're at the target position
            print(f"Current position: X={current_position['x']:.3f} m, "
                  f"Y={current_position['y']:.3f} m, Z={current_position['z']:.3f} m")
            
            # Hover at 1.0 m at the same X, Y position
            print("\nHovering at 1.0 m at same X, Y position...")
            hover_duration = 2.0  # seconds
            hover_steps = int(hover_duration / 0.1)
            for _ in range(hover_steps):
                connection.scf.cf.commander.send_position_setpoint(
                    initial_position['x'], 
                    initial_position['y'], 
                    target_z, 
                    0
                )
                time.sleep(0.1)
            
            # Land sequence - return to initial position - continuously send commands
            print("\n=== Landing Sequence ===")
            print(f"Landing back to initial position...")
            landing_duration = 3.0  # seconds
            landing_steps = int(landing_duration / 0.1)
            
            for i in range(landing_steps):
                connection.scf.cf.commander.send_position_setpoint(
                    initial_position['x'], 
                    initial_position['y'], 
                    initial_position['z'], 
                    0
                )
                time.sleep(0.1)
            
            print(f"Final position: X={current_position['x']:.3f} m, "
                  f"Y={current_position['y']:.3f} m, Z={current_position['z']:.3f} m")
            
            # Send a few more position commands to ensure stability
            for _ in range(5):
                connection.scf.cf.commander.send_position_setpoint(
                    initial_position['x'], 
                    initial_position['y'], 
                    initial_position['z'], 
                    0
                )
                time.sleep(0.1)
            
            log_conf.stop()
            
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

