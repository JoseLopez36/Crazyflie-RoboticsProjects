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
    parser = argparse.ArgumentParser(description='Crazyflie position control - square pattern')
    parser.add_argument('--sim', action='store_true', help='Force simulation mode')
    parser.add_argument('--hardware', action='store_true', help='Force hardware mode')
    parser.add_argument('--uri', type=str, default="radio://0/80/2M/E7E7E7E701", help='Crazyflie URI to connect to (default: radio://0/80/2M/E7E7E7E701)')
    parser.add_argument('--side-length', type=float, default=2.0, help='Square side length in meters (default: 0.5)')
    parser.add_argument('--height', type=float, default=1.5, help='Flight height in meters (default: 1.0)')
    parser.add_argument('--waypoint-duration', type=float, default=3.0, help='Time to reach each waypoint in seconds (default: 3.0)')
    args = parser.parse_args()
    
    print("=== Crazyflie Position Control - Square Pattern ===\n")
    
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
            
            # Store initial position from logged position estimate
            global initial_position
            initial_position['x'] = current_position['x']
            initial_position['y'] = current_position['y']
            initial_position['z'] = current_position['z']
            
            print(f"\nInitial position logged (top-left corner of square):")
            print(f"  X: {initial_position['x']:.3f} m")
            print(f"  Y: {initial_position['y']:.3f} m")
            print(f"  Z: {initial_position['z']:.3f} m")
            print(f"\nSquare parameters:")
            print(f"  Side length: {args.side_length:.2f} m")
            print(f"  Flight height: {args.height:.2f} m")
            
            # Define square waypoints relative to initial position
            # Top-left corner is the initial position
            waypoints = [
                # (x_offset, y_offset, z, description)
                (0.0, 0.0, args.height, "Top-left (initial)"),
                (args.side_length, 0.0, args.height, "Top-right"),
                (args.side_length, -args.side_length, args.height, "Bottom-right"),
                (0.0, -args.side_length, args.height, "Bottom-left"),
                (0.0, 0.0, args.height, "Return to top-left"),
            ]
            
            # Takeoff to flight height - continuously send position commands
            print("\n=== Takeoff Sequence ===")
            print(f"Taking off...")
            connection.scf.cf.high_level_commander.takeoff(args.height, 3.0)
            time.sleep(4.0)
            
            # Fly square pattern
            print("\n=== Flying Square Pattern ===")
            waypoint_duration = args.waypoint_duration
            waypoint_steps = int(waypoint_duration / 0.1)  # Send command every 0.1s
            
            for i, (x_offset, y_offset, z, description) in enumerate(waypoints, 1):
                target_x = initial_position['x'] + x_offset
                target_y = initial_position['y'] + y_offset
                target_z = z
                
                print(f"\nWaypoint {i}/{len(waypoints)}: {description}")
                print(f"  Target: X={target_x:.3f} m, Y={target_y:.3f} m, Z={target_z:.3f} m")
                
                # Continuously send position commands to reach waypoint
                for _ in range(waypoint_steps):
                    connection.scf.cf.commander.send_position_setpoint(
                        target_x,
                        target_y,
                        target_z,
                        0
                    )
                    time.sleep(0.1)
                
                # Show current position after reaching waypoint
                print(f"  Reached: X={current_position['x']:.3f} m, "
                      f"Y={current_position['y']:.3f} m, Z={current_position['z']:.3f} m")
            
            # Land sequence - smooth descent
            print("\n=== Landing Sequence ===")
            
            # Step 1: Return to initial X, Y position at current height (if not already there)
            print("Returning to initial position...")
            return_duration = 2.0  # seconds
            return_steps = int(return_duration / 0.1)
            for _ in range(return_steps):
                connection.scf.cf.commander.send_position_setpoint(
                    initial_position['x'],
                    initial_position['y'],
                    current_position['z'],  # Maintain current height
                    0
                )
                time.sleep(0.1)
            
            # Step 2: Gradually descend in steps using position control
            print("Descending smoothly...")
            start_z = current_position['z']
            target_landing_z = initial_position['z'] + 0.1  # Land slightly above initial Z (10cm buffer)
            descent_steps = 15  # Number of descent steps (more steps = smoother)
            descent_duration = 5.0  # Total descent time in seconds
            step_duration = descent_duration / descent_steps
            
            for i in range(descent_steps):
                # Calculate target Z for this step (linear interpolation)
                progress = (i + 1) / descent_steps
                target_z = start_z - (start_z - target_landing_z) * progress
                
                step_steps = int(step_duration / 0.1)
                for _ in range(step_steps):
                    connection.scf.cf.commander.send_position_setpoint(
                        initial_position['x'],
                        initial_position['y'],
                        target_z,
                        0
                    )
                    time.sleep(0.1)
            
            # Step 3: Final slow descent to ground using land command
            print("Final landing...")
            connection.scf.cf.high_level_commander.land(0.0, 2.5)  # Slow landing (2.5 seconds)
            time.sleep(3.0)  # Wait for landing to complete

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
