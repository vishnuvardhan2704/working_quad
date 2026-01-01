"""
Waypoint mission upload and execution using ArduPilot AUTO mode.
"""

from dronekit import VehicleMode, Command
from pymavlink import mavutil
import time
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
    
    def upload_mission(self, waypoints):
        """
        Upload mission to vehicle using MAV_CMD_NAV_WAYPOINT.
        
        Args:
            waypoints: List of LocationGlobalRelative objects
            
        Returns:
            True if upload successful, False otherwise
        """
        MissionLogger.info(f"Uploading mission with {len(waypoints)} waypoints...")
        
        # Clear existing mission
        cmds = self.vehicle.commands
        cmds.clear()
        
        # Add takeoff command (will be handled separately in GUIDED mode)
        # AUTO mode mission starts from first waypoint
        
        # Add waypoint commands with 5-second hover
        for i, waypoint in enumerate(waypoints):
            cmd = Command(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0,
                5, 0, 0, 0,  # param1=5sec hold time, param2-4 (accept radius, pass radius, yaw)
                waypoint.lat,
                waypoint.lon,
                waypoint.alt
            )
            cmds.add(cmd)
            MissionLogger.info(f"  WP{i+1}: ({waypoint.lat:.6f}, {waypoint.lon:.6f}, {waypoint.alt}m) [hover 5s]")
        
        # Upload mission
        try:
            cmds.upload()
            MissionLogger.success(f"Mission uploaded: {len(waypoints)} waypoints")
            return True
        except Exception as e:
            MissionLogger.error(f"Mission upload failed: {str(e)}")
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
