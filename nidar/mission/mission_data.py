"""
Mission waypoint definitions.
Define your autonomous flight path here.
Supports both static waypoints and KML-based area surveys.
"""

import os
from dronekit import LocationGlobalRelative


class MissionData:
    """Defines waypoint mission for autonomous flight."""
    
    # Takeoff altitude (meters above home)
    TAKEOFF_ALTITUDE = 10.0
    
    # Default airspeed (m/s)
    DEFAULT_AIRSPEED = 5.0
    
    # Mission source: "static" for hardcoded waypoints, "kml" for KML file
    MISSION_SOURCE = os.getenv('MISSION_SOURCE', 'static')
    
    # KML file configuration (only used if MISSION_SOURCE="kml")
    KML_FILE = os.getenv('KML_FILE', 'missions/survey_area.kml')
    KML_ALTITUDE = float(os.getenv('KML_ALTITUDE', '15.0'))
    KML_PATTERN = os.getenv('KML_PATTERN', 'curved')  # "curved" or "lawnmower"
    KML_CAMERA_FOV = float(os.getenv('KML_CAMERA_FOV', '57'))  # degrees
    KML_OVERLAP = float(os.getenv('KML_OVERLAP', '0.25'))  # 25% overlap
    
    @staticmethod
    def get_static_waypoints():
        """
        Returns hardcoded static waypoints.
        
        Each waypoint is a LocationGlobalRelative object:
        - latitude (degrees)
        - longitude (degrees)
        - altitude (meters above home)
        
        NOTE: These coordinates are for SITL simulation.
        For real flights, replace with actual GPS coordinates.
        
        HOW TO SET WAYPOINTS:
        1. Get GPS coordinates from Google Maps, QGC, or mission planner
        2. Format: LocationGlobalRelative(latitude, longitude, altitude)
        3. Altitude is in METERS ABOVE HOME (not above sea level)
        4. ~0.0001° latitude/longitude ≈ 11 meters
        5. ~0.00027° latitude/longitude ≈ 30 meters (100 feet)
        
        Current pattern: Hexagon with 6 waypoints, ~30m apart, 10m altitude
        """
        waypoints = [
            # Waypoint 1: North - Starting point
            LocationGlobalRelative(-35.3632607, 149.1652351, 10.0),
            
            # Waypoint 2: Northeast - ~30m at 60° angle
            LocationGlobalRelative(-35.3633757, 149.1654851, 10.0),
            
            # Waypoint 3: Southeast - ~30m south from WP2
            LocationGlobalRelative(-35.3636257, 149.1654851, 10.0),
            
            # Waypoint 4: South - ~30m south from WP3
            LocationGlobalRelative(-35.3637407, 149.1652351, 10.0),
            
            # Waypoint 5: Southwest - ~30m west from WP4
            LocationGlobalRelative(-35.3636257, 149.1649851, 10.0),
            
            # Waypoint 6: Northwest - ~30m north from WP5 (completes hexagon)
            LocationGlobalRelative(-35.3633757, 149.1649851, 10.0),
        ]
        
        return waypoints
    
    @staticmethod
    def get_waypoints():
        """
        Returns mission waypoints based on MISSION_SOURCE configuration.
        
        Returns:
            List of LocationGlobalRelative waypoints
            
        Raises:
            ValueError: If KML file is invalid or MISSION_SOURCE is unknown
            FileNotFoundError: If KML file doesn't exist
        """
        if MissionData.MISSION_SOURCE == 'kml':
            # Load waypoints from KML file
            from .kml_loader import load_waypoints_from_kml
            return load_waypoints_from_kml(
                kml_file=MissionData.KML_FILE,
                altitude_meters=MissionData.KML_ALTITUDE,
                pattern=MissionData.KML_PATTERN,
                camera_fov=MissionData.KML_CAMERA_FOV,
                overlap=MissionData.KML_OVERLAP
            )
        else:
            # Use static waypoints
            return MissionData.get_static_waypoints()
    
    @staticmethod
    def get_mission_summary():
        """Returns human-readable mission summary."""
        if MissionData.MISSION_SOURCE == 'kml':
            from .kml_loader import get_kml_mission_summary
            waypoints = MissionData.get_waypoints()
            return get_kml_mission_summary(waypoints, MissionData.KML_FILE)
        else:
            waypoints = MissionData.get_waypoints()
            return f"Static Mission: {len(waypoints)} waypoints at {MissionData.TAKEOFF_ALTITUDE}m altitude"
