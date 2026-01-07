"""
KML-based waypoint loader for area survey missions.
Integrates KML parsing with NIDAR path planning and DroneKit.
"""

import sys
import os
from dronekit import LocationGlobalRelative

# Add parent directory to path for kml_parsing imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from kml_parsing.kml_parser import extract_boundary_from_kml
from kml_parsing.path_planner import generate_curved_center_coverage, generate_lawnmower_coverage


def load_waypoints_from_kml(kml_file, altitude_meters=15.0, pattern="curved", 
                            camera_fov=57, overlap=0.25):
    """
    Generate coverage path waypoints from KML AOI file.
    
    Args:
        kml_file: Path to KML file containing the area of interest polygon
        altitude_meters: Flight altitude in meters AGL (above home)
        pattern: Coverage pattern - "curved" or "lawnmower"
        camera_fov: Camera field of view in degrees (default 57° for wide-angle)
        overlap: Coverage overlap fraction (default 0.25 = 25% overlap)
        
    Returns:
        List of LocationGlobalRelative objects for mission upload
        
    Raises:
        ValueError: If KML file is invalid or pattern is unknown
        FileNotFoundError: If KML file doesn't exist
    """
    # Extract boundary from KML
    boundary = extract_boundary_from_kml(kml_file)
    
    # Generate coverage path using NIDAR algorithm
    if pattern == "curved":
        waypoints_raw = generate_curved_center_coverage(
            boundary, altitude_meters, camera_fov, overlap
        )
    elif pattern == "lawnmower":
        waypoints_raw = generate_lawnmower_coverage(
            boundary, altitude_meters, camera_fov, overlap
        )
    else:
        raise ValueError(f"Unknown pattern: {pattern}. Use 'curved' or 'lawnmower'")
    
    # Convert to DroneKit LocationGlobalRelative format
    waypoints = []
    for wp in waypoints_raw:
        lat = wp[0]
        lon = wp[1]
        alt = wp[2] if len(wp) > 2 else altitude_meters
        waypoints.append(LocationGlobalRelative(lat, lon, alt))
    
    return waypoints


def get_kml_mission_summary(waypoints, kml_file):
    """
    Get summary of KML-generated mission.
    
    Args:
        waypoints: List of LocationGlobalRelative objects
        kml_file: Path to source KML file
        
    Returns:
        Human-readable summary string
    """
    if not waypoints:
        return "KML Mission: No waypoints loaded"
    
    avg_alt = sum(wp.alt for wp in waypoints) / len(waypoints)
    return f"KML Survey [{kml_file}]: {len(waypoints)} waypoints at ~{avg_alt:.1f}m altitude"


def validate_kml_mission(waypoints, max_waypoints=500, min_altitude=0.5, max_altitude=100.0):
    """
    Validate KML-generated mission for safety.
    
    Args:
        waypoints: List of LocationGlobalRelative objects
        max_waypoints: Maximum allowed waypoints
        min_altitude: Minimum safe altitude in meters (0.5m for low-altitude scouting)
        max_altitude: Maximum safe altitude in meters
        
    Returns:
        (valid, error_message) tuple
    """
    if not waypoints:
        return False, "No waypoints in mission"
    
    if len(waypoints) > max_waypoints:
        return False, f"Too many waypoints: {len(waypoints)} > {max_waypoints}"
    
    for i, wp in enumerate(waypoints):
        if wp.alt < min_altitude:
            return False, f"Waypoint {i+1} altitude too low: {wp.alt}m < {min_altitude}m"
        if wp.alt > max_altitude:
            return False, f"Waypoint {i+1} altitude too high: {wp.alt}m > {max_altitude}m"
    
    return True, "Mission valid"
