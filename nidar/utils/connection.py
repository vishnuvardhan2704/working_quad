"""
Vehicle connection management for DroneKit.
"""

from dronekit import connect, VehicleMode
import time
from utils.logger import MissionLogger


class VehicleConnection:
    """Manages connection to ArduPilot vehicle via MAVLink."""
    
    @staticmethod
    def connect_vehicle(connection_string, baud=57600, timeout=60):
        """
        Connect to vehicle and wait for initialization.
        
        Args:
            connection_string: Connection target (e.g., 'udp:127.0.0.1:14550' for SITL)
            baud: Baud rate for serial connections (ignored for UDP)
            timeout: Maximum time to wait for connection
            
        Returns:
            Vehicle object if successful, None otherwise
        """
        MissionLogger.info(f"Connecting to vehicle on: {connection_string}")
        
        try:
            # Connect to vehicle
            if connection_string.startswith('udp:') or connection_string.startswith('tcp:'):
                vehicle = connect(connection_string, wait_ready=True, timeout=timeout)
            else:
                # Serial connection (real Pixhawk)
                vehicle = connect(connection_string, baud=baud, wait_ready=True, timeout=timeout)
            
            MissionLogger.success("Connected to vehicle")
            
            # Display vehicle state
            MissionLogger.info(f"Autopilot Firmware Version: {vehicle.version}")
            MissionLogger.info(f"Vehicle Mode: {vehicle.mode.name}")
            MissionLogger.info(f"Armed: {vehicle.armed}")
            MissionLogger.info(f"GPS: {vehicle.gps_0}")
            MissionLogger.info(f"Battery: {vehicle.battery}")
            MissionLogger.info(f"Home Location: {vehicle.home_location}")
            
            return vehicle
            
        except Exception as e:
            MissionLogger.error(f"Failed to connect: {str(e)}")
            return None
    
    @staticmethod
    def close_vehicle(vehicle):
        """Safely close vehicle connection."""
        if vehicle:
            MissionLogger.info("Closing vehicle connection...")
            vehicle.close()
            MissionLogger.success("Vehicle connection closed")
