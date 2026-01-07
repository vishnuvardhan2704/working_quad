"""
NIDAR Path Planning Algorithm
Generates curved center-shrinking coverage patterns for area surveys.
"""

from shapely.geometry import Polygon
import numpy as np
from .gps_utils import gps_to_xy, xy_to_gps
from .camera_params import calculate_pass_spacing


def smooth_path(coords, iterations=2):
    """
    Apply Chaikin curve smoothing algorithm.
    
    Args:
        coords: List of (x, y) coordinate tuples
        iterations: Number of smoothing passes
        
    Returns:
        Smoothed coordinate list
    """
    for _ in range(iterations):
        new_coords = []
        for i in range(len(coords) - 1):
            p0 = np.array(coords[i])
            p1 = np.array(coords[i + 1])
            # Chaikin subdivision points
            q = 0.75 * p0 + 0.25 * p1
            r = 0.25 * p0 + 0.75 * p1
            new_coords.extend([tuple(q), tuple(r)])
        new_coords.append(new_coords[0])  # Close loop
        coords = new_coords
    return coords


def downsample(coords, step=5.0):
    """
    Downsample path to reduce waypoint density.
    
    Args:
        coords: List of (x, y) coordinate tuples
        step: Minimum distance in meters between sampled points
        
    Returns:
        Downsampled coordinate list
    """
    sampled = [coords[0]]
    acc = 0.0
    for i in range(1, len(coords)):
        p0 = np.array(coords[i-1])
        p1 = np.array(coords[i])
        acc += np.linalg.norm(p1 - p0)
        if acc >= step:
            sampled.append(tuple(p1))
            acc = 0.0
    return sampled


def generate_curved_center_coverage(boundary_gps, altitude_m, camera_fov=57, overlap=0.25):
    """
    Generate inward-spiraling coverage pattern with curved paths.
    
    This algorithm creates a lawn-mower pattern that spirals inward from
    the boundary, suitable for human detection surveys.
    
    Args:
        boundary_gps: List of (lat, lon) tuples defining survey area boundary
        altitude_m: Flight altitude in meters AGL
        camera_fov: Camera field of view in degrees (default 57°)
        overlap: Coverage overlap fraction (default 0.25 = 25%)
        
    Returns:
        List of (lat, lon, altitude) tuples for mission waypoints
    """
    # Use first boundary point as reference for local coordinate system
    ref_lat, ref_lon = boundary_gps[0]

    # Convert boundary to XY coordinates (meters)
    boundary_xy = [gps_to_xy(lat, lon, ref_lat, ref_lon)
                   for lat, lon in boundary_gps]

    # Create polygon from boundary
    polygon = Polygon(boundary_xy)

    # Calculate pass spacing based on camera FOV and overlap
    _, pass_spacing = calculate_pass_spacing(
        altitude_m=altitude_m,
        fov_deg=camera_fov,
        overlap=overlap
    )

    waypoints = []
    current = polygon

    # Minimum area threshold to stop spiraling inward
    MIN_LOOP_AREA = (pass_spacing * 3) ** 2

    # Generate inward-spiraling coverage pattern
    while not current.is_empty and current.area > MIN_LOOP_AREA:
        # Get current loop boundary
        ring = list(current.exterior.coords)

        # Smooth the path for curved turns
        smooth_ring = smooth_path(ring, iterations=1)
        
        # Downsample to reduce waypoint count
        smooth_ring = downsample(smooth_ring, step=25.0)

        # Convert XY waypoints back to GPS
        for x, y in smooth_ring:
            lat, lon = xy_to_gps(x, y, ref_lat, ref_lon)
            waypoints.append((lat, lon, altitude_m))

        # Shrink polygon inward by pass spacing
        current = current.buffer(-pass_spacing)

    # Add entry waypoint at boundary start
    entry_x, entry_y = polygon.exterior.coords[0]
    entry_lat, entry_lon = xy_to_gps(entry_x, entry_y, ref_lat, ref_lon)
    waypoints.insert(0, (entry_lat, entry_lon, altitude_m))

    return waypoints


def generate_lawnmower_coverage(boundary_gps, altitude_m, camera_fov=57, overlap=0.25, angle=0):
    """
    Generate traditional parallel lawnmower coverage pattern.
    
    Args:
        boundary_gps: List of (lat, lon) tuples defining survey area boundary
        altitude_m: Flight altitude in meters AGL
        camera_fov: Camera field of view in degrees
        overlap: Coverage overlap fraction
        angle: Path angle in degrees (0 = East-West)
        
    Returns:
        List of (lat, lon, altitude) tuples for mission waypoints
    """
    # Reference point
    ref_lat, ref_lon = boundary_gps[0]

    # Convert boundary to XY
    boundary_xy = [gps_to_xy(lat, lon, ref_lat, ref_lon)
                   for lat, lon in boundary_gps]

    polygon = Polygon(boundary_xy)

    # Calculate pass spacing
    _, pass_spacing = calculate_pass_spacing(
        altitude_m=altitude_m,
        fov_deg=camera_fov,
        overlap=overlap
    )

    # Get polygon bounds
    min_x, min_y, max_x, max_y = polygon.bounds

    waypoints = []
    y = min_y
    direction = True

    # Generate parallel passes
    while y <= max_y:
        if direction:
            start = (min_x - 10, y)
            end = (max_x + 10, y)
        else:
            start = (max_x + 10, y)
            end = (min_x - 10, y)

        # Check if line intersects polygon
        from shapely.geometry import LineString
        line = LineString([start, end])
        intersection = line.intersection(polygon)
        
        if not intersection.is_empty:
            # Get intersection endpoints
            coords = list(intersection.coords)
            if len(coords) >= 2:
                x1, y1 = coords[0]
                x2, y2 = coords[-1]
                
                lat1, lon1 = xy_to_gps(x1, y1, ref_lat, ref_lon)
                lat2, lon2 = xy_to_gps(x2, y2, ref_lat, ref_lon)

                waypoints.append((lat1, lon1, altitude_m))
                waypoints.append((lat2, lon2, altitude_m))

        direction = not direction
        y += pass_spacing

    return waypoints
