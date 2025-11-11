#!/usr/bin/env python3
"""
Crazyflie Utility Module
Provides functions to detect and connect to Crazyflie drones in both simulation and real hardware.
"""

import logging
import sys
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
import cflib.crtp
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Simulation addresses (from sim_cf2 driver)
SIM_ADDRESSES = {
    0xE7E7E7E701: 'radio://0/80/2M/E7E7E7E701',
    0xE7E7E7E702: 'radio://0/80/2M/E7E7E7E702',
    0xE7E7E7E703: 'radio://0/80/2M/E7E7E7E703',
    0xE7E7E7E704: 'radio://0/80/2M/E7E7E7E704',
    0xE7E7E7E705: 'radio://0/80/2M/E7E7E7E705',
    0xE7E7E7E706: 'radio://0/80/2M/E7E7E7E706',
    0xE7E7E7E707: 'radio://0/80/2M/E7E7E7E707',
    0xE7E7E7E708: 'radio://0/80/2M/E7E7E7E708',
    0xE7E7E7E709: 'radio://0/80/2M/E7E7E7E709',
    0xE7E7E7E70A: 'radio://0/80/2M/E7E7E7E70A',
}


def detect_connection_mode(force_sim=False, force_hardware=False):
    """
    Detect whether to use simulation or real hardware connection.
    
    Args:
        force_sim: Force simulation mode (default: False)
        force_hardware: Force hardware mode (default: False)
    
    Returns:
        str: 'simulation' or 'hardware'
    """
    if force_sim:
        return 'simulation'
    if force_hardware:
        return 'hardware'
    
    # Initialize drivers to detect available interfaces
    cflib.crtp.init_drivers()
    
    # Try to scan for real hardware
    available = cflib.crtp.scan_interfaces()
    
    # Check if any radio dongles are available
    radio_available = any(interface for interface in available if 'radio://' in str(interface))
    
    if radio_available:
        logger.info("Radio dongle detected. Using hardware mode.")
        return 'hardware'
    else:
        logger.info("No radio dongle detected. Using simulation mode.")
        return 'simulation'


def scan_crazyflies(mode='auto', force_sim=False, force_hardware=False):
    """
    Scan for available Crazyflie drones.
    
    Args:
        mode: Connection mode ('auto', 'simulation', 'hardware')
        force_sim: Force simulation mode
        force_hardware: Force hardware mode
    
    Returns:
        list: List of (address, uri) tuples for available Crazyflies
    """
    if mode == 'auto':
        mode = detect_connection_mode(force_sim, force_hardware)
    
    if mode == 'simulation':
        # Initialize with sim driver
        cflib.crtp.init_drivers(enable_sim_driver=True)
        
        # Return simulation addresses
        crazyflies = []
        for address, uri in SIM_ADDRESSES.items():
            crazyflies.append((address, uri))
        logger.info(f"Found {len(crazyflies)} simulation Crazyflies")
        return crazyflies
    else:
        # Initialize normal drivers (will auto-detect sim if no radio)
        cflib.crtp.init_drivers()
        
        # Scan for real hardware
        available = cflib.crtp.scan_interfaces()
        crazyflies = []
        
        for interface in available:
            uri = str(interface)
            if 'radio://' in uri:
                # Extract address from URI if possible
                # Format: radio://0/80/2M/E7E7E7E701
                try:
                    parts = uri.split('/')
                    if len(parts) > 0:
                        address_str = parts[-1]
                        # Convert hex string to int
                        address = int(address_str, 16)
                        crazyflies.append((address, uri))
                except (ValueError, IndexError):
                    # If we can't parse, just use the URI
                    crazyflies.append((None, uri))
        
        logger.info(f"Found {len(crazyflies)} hardware Crazyflies")
        return crazyflies


def connect_crazyflie(uri=None, mode='auto', force_sim=False, force_hardware=False, 
                     cf_id=1, link_quality_callback=None):
    """
    Connect to a Crazyflie drone (simulation or hardware).
    
    Args:
        uri: Specific URI to connect to (if None, will scan and use cf_id)
        mode: Connection mode ('auto', 'simulation', 'hardware')
        force_sim: Force simulation mode
        force_hardware: Force hardware mode
        cf_id: Crazyflie ID to use (1-based index, default: 1)
        link_quality_callback: Optional callback for link quality updates
    
    Returns:
        SyncCrazyflie: Connected Crazyflie instance, or None if connection failed
    """
    if mode == 'auto':
        mode = detect_connection_mode(force_sim, force_hardware)
    
    # Get URI if not provided
    if uri is None:
        crazyflies = scan_crazyflies(mode, force_sim, force_hardware)
        
        if not crazyflies:
            logger.error("No Crazyflies found!")
            return None
        
        if cf_id < 1 or cf_id > len(crazyflies):
            logger.warning(f"cf_id {cf_id} out of range. Using first available.")
            cf_id = 1
        
        _, uri = crazyflies[cf_id - 1]
        logger.info(f"Connecting to Crazyflie #{cf_id} at {uri}")
    
    # Create and connect
    try:
        cf = Crazyflie(rw_cache='./cache')
        
        # Set link quality callback if provided
        if link_quality_callback:
            cf.connected.add_callback(lambda uri: cf.param.add_update_callback(
                group='radio', name='rssi', cb=link_quality_callback))
        
        scf = SyncCrazyflie(uri, cf=cf)
        scf.open_link()
        
        logger.info(f"Successfully connected to {uri}")
        return scf
        
    except Exception as e:
        logger.error(f"Failed to connect to {uri}: {e}")
        return None


def disconnect_crazyflie(scf):
    """
    Disconnect from a Crazyflie drone.
    
    Args:
        scf: SyncCrazyflie instance to disconnect
    """
    if scf:
        try:
            scf.close_link()
            logger.info("Disconnected from Crazyflie")
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")


class CrazyflieConnection:
    """
    Context manager for Crazyflie connections.
    Usage:
        with CrazyflieConnection(mode='auto') as scf:
            # Use scf here
            pass
    """
    
    def __init__(self, uri=None, mode='auto', force_sim=False, force_hardware=False, 
                 cf_id=1, link_quality_callback=None):
        self.uri = uri
        self.mode = mode
        self.force_sim = force_sim
        self.force_hardware = force_hardware
        self.cf_id = cf_id
        self.link_quality_callback = link_quality_callback
        self.scf = None
    
    def __enter__(self):
        self.scf = connect_crazyflie(
            uri=self.uri,
            mode=self.mode,
            force_sim=self.force_sim,
            force_hardware=self.force_hardware,
            cf_id=self.cf_id,
            link_quality_callback=self.link_quality_callback
        )
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.scf:
            disconnect_crazyflie(self.scf)
        return False

    def reset_estimator(self):
        """
        Reset the Kalman estimator.
        """
        self.scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf.cf.param.set_value('kalman.resetEstimation', '0')
        
        logger.info('Waiting for estimator to find position...')

        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(self.scf, log_config) as sync_logger:
            for log_entry in sync_logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    break