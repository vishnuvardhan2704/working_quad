"""
KML file parser for extracting survey area boundaries.
"""

from lxml import etree


def extract_boundary_from_kml(kml_file):
    """
    Extract boundary polygon from KML file.
    
    Supports Google Earth KML files with Polygon features.
    
    Args:
        kml_file: Path to KML file containing AOI polygon
        
    Returns:
        List of (lat, lon) tuples defining the boundary
        
    Raises:
        ValueError: If no polygon found in KML file
        FileNotFoundError: If KML file doesn't exist
    """
    try:
        tree = etree.parse(kml_file)
    except FileNotFoundError:
        raise FileNotFoundError(f"KML file not found: {kml_file}")
    except Exception as e:
        raise ValueError(f"Failed to parse KML file: {str(e)}")
    
    root = tree.getroot()
    
    # KML namespace
    ns = {"kml": "http://www.opengis.net/kml/2.2"}
    
    # Find polygon coordinates
    coords = root.xpath(
        ".//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates",
        namespaces=ns
    )
    
    if not coords:
        raise ValueError("No polygon found in KML file")
    
    coord_text = coords[0].text.strip()
    
    # Parse coordinates (KML format: lon,lat,alt or lon,lat)
    boundary = []
    for line in coord_text.split():
        line = line.strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) >= 2:
            lon = float(parts[0])
            lat = float(parts[1])
            boundary.append((lat, lon))
    
    if len(boundary) < 3:
        raise ValueError(f"Invalid polygon: need at least 3 points, got {len(boundary)}")
    
    return boundary


def get_kml_center(boundary):
    """
    Calculate approximate center of KML boundary.
    
    Args:
        boundary: List of (lat, lon) tuples
        
    Returns:
        (center_lat, center_lon) tuple
    """
    lats = [p[0] for p in boundary]
    lons = [p[1] for p in boundary]
    return sum(lats) / len(lats), sum(lons) / len(lons)


def get_kml_bounds(boundary):
    """
    Get bounding box of KML boundary.
    
    Args:
        boundary: List of (lat, lon) tuples
        
    Returns:
        (min_lat, min_lon, max_lat, max_lon) tuple
    """
    lats = [p[0] for p in boundary]
    lons = [p[1] for p in boundary]
    return min(lats), min(lons), max(lats), max(lons)
