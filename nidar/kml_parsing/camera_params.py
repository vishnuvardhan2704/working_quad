"""
Camera field-of-view and pass spacing calculations.
Used to determine optimal waypoint spacing for area coverage.
"""

import math


def calculate_pass_spacing(altitude_m, fov_deg, overlap=0.25):
    """
    Calculate camera ground coverage swath and pass spacing.
    
    Args:
        altitude_m: Flight altitude in meters AGL
        fov_deg: Camera field of view in degrees
        overlap: Desired overlap fraction (0.25 = 25% overlap)
        
    Returns:
        (swath_m, spacing_m) tuple
        - swath_m: Ground coverage width at this altitude
        - spacing_m: Distance between parallel passes
    """
    # Calculate ground swath width from camera FOV
    swath = 2 * altitude_m * math.tan(math.radians(fov_deg / 2))
    
    # Calculate spacing to achieve desired overlap
    spacing = swath * (1 - overlap)
    
    return swath, spacing
