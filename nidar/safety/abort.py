"""
Emergency abort and return-to-launch functionality for VTOL aircraft.
Uses QLAND/QRTL for vertical landing/RTL on QuadPlane.
"""

from dronekit import VehicleMode
import time
from utils.logger import MissionLogger

# VTOL Configuration - must match main.py
IS_VTOL = True  # Set to True for QuadPlane VTOL, False for standard quadcopter


class SafetyAbort:
    """Handles emergency abort and RTL procedures for VTOL aircraft."""
    
    def __init__(self, vehicle):
        """
        Initialize safety abort handler.
        
        Args:
            vehicle: DroneKit Vehicle object
        """
        self.vehicle = vehicle
    
    def emergency_land(self):
        """
        Immediately switch to QLAND mode (VTOL vertical landing).
        This is the fastest abort - aircraft lands vertically where it is.
        
        Returns:
            True if mode change successful
        """
        land_mode = "QLAND" if IS_VTOL else "LAND"
        MissionLogger.warning(f"EMERGENCY ABORT - Switching to {land_mode} mode")
        
        try:
            self.vehicle.mode = VehicleMode(land_mode)
            
            # Wait for mode confirmation
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != land_mode:
                if time.time() - start_time > timeout:
                    MissionLogger.error(f"Failed to enter {land_mode} mode")
                    return False
                time.sleep(0.1)
            
            MissionLogger.state(f"{land_mode} mode activated - vertical descent at current position")
            return True
            
        except Exception as e:
            MissionLogger.error(f"Emergency land failed: {str(e)}")
            return False
    
    def return_to_launch(self):
        """
        Switch to QRTL mode (VTOL vertical return to launch).
        Aircraft returns to home and lands vertically.
        
        Returns:
            True if mode change successful
        """
        rtl_mode = "QRTL" if IS_VTOL else "RTL"
        MissionLogger.warning(f"RETURN TO LAUNCH - Switching to {rtl_mode} mode")
        
        try:
            self.vehicle.mode = VehicleMode(rtl_mode)
            
            # Wait for mode confirmation
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != rtl_mode:
                if time.time() - start_time > timeout:
                    MissionLogger.error(f"Failed to enter {rtl_mode} mode")
                    return False
                time.sleep(0.1)
            
            MissionLogger.state(f"{rtl_mode} mode activated - returning to home (vertical)")
            return True
            
        except Exception as e:
            MissionLogger.error(f"RTL failed: {str(e)}")
            return False
    
    def monitor_landing(self):
        """
        Monitor landing progress until touchdown.
        Call this in a loop after initiating QLAND or QRTL.
        
        Returns:
            True when landed and disarmed
        """
        mode = self.vehicle.mode.name
        altitude = self.vehicle.location.global_relative_frame.alt
        armed = self.vehicle.armed
        
        # Check if we've landed (altitude near zero and disarmed)
        if not armed and altitude < 0.5:
            MissionLogger.success("Landing complete - vehicle disarmed")
            return True
        
        # Still descending - check for VTOL landing modes
        landing_modes = ["LAND", "RTL", "QLAND", "QRTL"]
        if mode in landing_modes:
            MissionLogger.info(f"Descending: {altitude:.1f}m altitude, {'armed' if armed else 'disarmed'}")
        
        return False
        
        return False
    
    def force_disarm(self):
        """
        Force disarm the vehicle (CAUTION: use only on ground).
        
        Returns:
            True if disarm successful
        """
        MissionLogger.warning("Forcing vehicle disarm...")
        
        try:
            self.vehicle.armed = False
            
            # Wait for disarm confirmation
            timeout = 10
            start_time = time.time()
            while self.vehicle.armed:
                if time.time() - start_time > timeout:
                    MissionLogger.error("Failed to disarm vehicle")
                    return False
                time.sleep(0.1)
            
            MissionLogger.success("Vehicle disarmed")
            return True
            
        except Exception as e:
            MissionLogger.error(f"Force disarm failed: {str(e)}")
            return False
