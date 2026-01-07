"""
Waypoint mission upload and execution using ArduPilot AUTO mode.
"""

from dronekit import VehicleMode, Command
from pymavlink import mavutil
import time
import threading
from utils.logger import MissionLogger


class WaypointMission:
    """Handles mission upload and AUTO mode execution."""
    
    def __init__(self, vehicle):
        """
        Initialize mission handler.
        
        Args:
            vehicle: DroneKit Vehicle object
        """
        self.vehicle = vehicle
        self._upload_result = None
        self._upload_error = None
    
    def _upload_worker(self, cmds):
        """Worker thread for mission upload."""
        try:
            cmds.upload()
            self._upload_result = True
        except Exception as e:
            self._upload_error = str(e)
            self._upload_result = False
    
    def upload_mission(self, waypoints, timeout=30):
        """
        Upload mission to vehicle using MAV_CMD_NAV_WAYPOINT.
        
        Args:
            waypoints: List of LocationGlobalRelative objects
            timeout: Maximum time to wait for upload (seconds)
            
        Returns:
            True if upload successful, False otherwise
        """
        MissionLogger.info(f"Uploading mission with {len(waypoints)} waypoints...")
        
        # Clear existing mission
        cmds = self.vehicle.commands
        cmds.clear()
        
        # Add takeoff command (will be handled separately in GUIDED mode)
        # AUTO mode mission starts from first waypoint
        
        # Add waypoint commands - fly through continuously (no hover)
        # Only log first/last 5 waypoints to avoid flooding
        for i, waypoint in enumerate(waypoints):
            cmd = Command(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0,
                0, 0, 0, 0,  # param1=0 (no hover), param2-4 (accept radius, pass radius, yaw)
                waypoint.lat,
                waypoint.lon,
                waypoint.alt
            )
            cmds.add(cmd)
            # Only log first 5 and last 5 waypoints
            if i < 5 or i >= len(waypoints) - 5:
                MissionLogger.info(f"  WP{i+1}: ({waypoint.lat:.6f}, {waypoint.lon:.6f}, {waypoint.alt}m)")
            elif i == 5:
                MissionLogger.info(f"  ... ({len(waypoints) - 10} more waypoints) ...")
        
        # Upload mission with timeout to prevent blocking
        MissionLogger.info(f"Uploading to Pixhawk (timeout: {timeout}s)...")
        self._upload_result = None
        self._upload_error = None
        
        upload_thread = threading.Thread(target=self._upload_worker, args=(cmds,))
        upload_thread.start()
        upload_thread.join(timeout=timeout)
        
        if upload_thread.is_alive():
            MissionLogger.error(f"Mission upload timed out after {timeout}s")
            return False
        
        if self._upload_result:
            MissionLogger.success(f"Mission uploaded: {len(waypoints)} waypoints")
            return True
        else:
            MissionLogger.error(f"Mission upload failed: {self._upload_error}")
            return False
    
    def start_mission(self):
        """
        Switch to AUTO mode to begin waypoint navigation.
        
        Returns:
            True if mode change successful
        """
        MissionLogger.state("Starting AUTO mode mission execution")
        
        try:
            self.vehicle.mode = VehicleMode("AUTO")
            
            # Wait for mode change confirmation
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != "AUTO":
                if time.time() - start_time > timeout:
                    MissionLogger.error("Failed to enter AUTO mode")
                    return False
                time.sleep(0.1)
            
            MissionLogger.success("AUTO mode activated - mission in progress")
            return True
            
        except Exception as e:
            MissionLogger.error(f"Failed to start mission: {str(e)}")
            return False
    
    def monitor_mission(self):
        """
        Monitor mission progress and report waypoint transitions.
        Call this in a loop to track mission status.
        
        Returns:
            True if mission complete, False if still in progress
        """
        if self.vehicle.mode.name != "AUTO":
            return True  # Mission ended (mode changed)
        
        # Check current waypoint
        current_wp = self.vehicle.commands.next
        total_wp = self.vehicle.commands.count
        
        # Mission complete when we've reached the last waypoint
        if current_wp >= total_wp:
            MissionLogger.success("Mission complete - all waypoints reached")
            return True
        
        return False
    
    def get_mission_status(self):
        """
        Get current mission progress.
        
        Returns:
            Dictionary with mission status information
        """
        return {
            'mode': self.vehicle.mode.name,
            'current_waypoint': self.vehicle.commands.next,
            'total_waypoints': self.vehicle.commands.count,
            'armed': self.vehicle.armed,
            'altitude': self.vehicle.location.global_relative_frame.alt,
            'groundspeed': self.vehicle.groundspeed,
        }
