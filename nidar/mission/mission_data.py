"""
Mission waypoint definitions.
Define your autonomous flight path here.
"""

from dronekit import LocationGlobalRelative


class MissionData:
    """Defines waypoint mission for autonomous flight."""
    
    # Takeoff altitude (meters above home)
    TAKEOFF_ALTITUDE = 10.0
    
    # Default airspeed (m/s)
    DEFAULT_AIRSPEED = 5.0
    
    @staticmethod
    def get_waypoints():
        """
        Returns list of mission waypoints.
        
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
    def get_mission_summary():
        """Returns human-readable mission summary."""
        waypoints = MissionData.get_waypoints()
        return f"Mission: {len(waypoints)} waypoints at {MissionData.TAKEOFF_ALTITUDE}m altitude"
