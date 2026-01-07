"""
GPS coordinate conversion utilities.
Convert between GPS (lat/lon) and local XY coordinates.
"""

import math

# Earth radius in meters
R = 6378137


def gps_to_xy(lat, lon, ref_lat, ref_lon):
    """
    Convert GPS coordinates to local XY meters.
    
    Args:
        lat: Latitude to convert
        lon: Longitude to convert
        ref_lat: Reference latitude (origin)
        ref_lon: Reference longitude (origin)
        
    Returns:
        (x, y) tuple in meters from reference point
    """
    x = math.radians(lon - ref_lon) * R * math.cos(math.radians(ref_lat))
    y = math.radians(lat - ref_lat) * R
    return x, y


def xy_to_gps(x, y, ref_lat, ref_lon):
    """
    Convert local XY meters to GPS coordinates.
    
    Args:
        x: X coordinate in meters
        y: Y coordinate in meters
        ref_lat: Reference latitude (origin)
        ref_lon: Reference longitude (origin)
        
    Returns:
        (lat, lon) tuple
    """
    lat = ref_lat + math.degrees(y / R)
    lon = ref_lon + math.degrees(x / (R * math.cos(math.radians(ref_lat))))
    return lat, lon
