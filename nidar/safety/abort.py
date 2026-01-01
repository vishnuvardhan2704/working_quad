"""
Emergency abort and return-to-launch functionality.
"""

from dronekit import VehicleMode
import time
from utils.logger import MissionLogger


class SafetyAbort:
    """Handles emergency abort and RTL procedures."""
    
    def __init__(self, vehicle):
        """
        Initialize safety abort handler.
        
        Args:
            vehicle: DroneKit Vehicle object
        """
        self.vehicle = vehicle
    
    def emergency_land(self):
        """
        Immediately switch to LAND mode at current position.
        This is the fastest abort - drone lands where it is.
        
        Returns:
            True if mode change successful
        """
        MissionLogger.warning("EMERGENCY ABORT - Switching to LAND mode")
        
        try:
            self.vehicle.mode = VehicleMode("LAND")
            
            # Wait for mode confirmation
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != "LAND":
                if time.time() - start_time > timeout:
                    MissionLogger.error("Failed to enter LAND mode")
                    return False
                time.sleep(0.1)
            
            MissionLogger.state("LAND mode activated - descending at current position")
            return True
            
        except Exception as e:
            MissionLogger.error(f"Emergency land failed: {str(e)}")
            return False
    
    def return_to_launch(self):
        """
        Switch to RTL mode - drone returns to home and lands.
        Use this when you want controlled return to launch point.
        
        Returns:
            True if mode change successful
        """
        MissionLogger.warning("RETURN TO LAUNCH - Switching to RTL mode")
        
        try:
            self.vehicle.mode = VehicleMode("RTL")
            
            # Wait for mode confirmation
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != "RTL":
                if time.time() - start_time > timeout:
                    MissionLogger.error("Failed to enter RTL mode")
                    return False
                time.sleep(0.1)
            
            MissionLogger.state("RTL mode activated - returning to home")
            return True
            
        except Exception as e:
            MissionLogger.error(f"RTL failed: {str(e)}")
            return False
    
    def monitor_landing(self):
        """
        Monitor landing progress until touchdown.
        Call this in a loop after initiating LAND or RTL.
        
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
        
        # Still descending
        if mode in ["LAND", "RTL"]:
            MissionLogger.info(f"Descending: {altitude:.1f}m altitude, {'armed' if armed else 'disarmed'}")
        
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
