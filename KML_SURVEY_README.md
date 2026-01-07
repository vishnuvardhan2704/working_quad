# KML Area Survey Mission System

## Overview

This quadcopter now supports **KML-based autonomous area surveys** for human detection and search/rescue operations. The system loads survey boundaries from Google Earth KML files and automatically generates optimal coverage flight paths using the NIDAR path planning algorithm.

## Features

- **KML Polygon Import**: Define survey areas in Google Earth, export as KML
- **NIDAR Path Planning**: Automatic curved center-shrinking coverage pattern
- **Camera-Optimized**: Adjustable camera FOV and overlap settings
- **Human Detection Ready**: Integrates with existing detection system
- **DroneKit Integration**: Seamless waypoint upload to ArduPilot
- **Safety Validation**: Automatic mission validation before execution

## Quick Start

### 1. Create Survey Area in Google Earth

1. Open Google Earth Pro or Google Earth Web
2. Use the Polygon tool to draw your survey area
3. Save/Export as KML file
4. Copy KML file to `/home/dart/quadtest/missions/`

### 2. Run Survey Mission (Two Methods)

#### Method A: Via LoRa Radio Commands (Recommended)

From ground station TX computer:
```bash
# Load and upload KML mission
KML:SURVEY:my_area.kml,15

# Then execute:
ARM
TAKEOFF:15
MODE:AUTO
```

#### Method B: Environment Variables (mission_control.py)

```bash
export MISSION_SOURCE=kml
export KML_FILE=missions/my_area.kml
export KML_ALTITUDE=15.0
export KML_PATTERN=curved
python3 nidar/mission_control.py
```

### 3. Monitor Mission

The drone will:
1. Upload waypoints from KML survey pattern
2. Take off to specified altitude
3. Execute spiral coverage pattern
4. Scan for humans using camera detection
5. Return to home when complete

## System Architecture

### Modules Created

```
nidar/
├── kml_parsing/               # KML processing and path planning
│   ├── __init__.py
│   ├── kml_parser.py         # Extract polygon from KML
│   ├── gps_utils.py          # GPS <-> XY coordinate conversion
│   ├── camera_params.py      # FOV and pass spacing calculation
│   └── path_planner.py       # NIDAR coverage algorithm
│
└── mission/
    └── kml_loader.py          # DroneKit integration

missions/                      # KML files directory
└── survey_area.kml           # Example survey area
```

### Path Planning Algorithm

The **NIDAR Curved Center Coverage** algorithm:

1. **Parse KML**: Extract polygon boundary from KML file
2. **Convert to XY**: Transform GPS to local meters
3. **Calculate Spacing**: Based on camera FOV and overlap:
   - Swath = 2 × altitude × tan(FOV/2)
   - Spacing = Swath × (1 - overlap)
4. **Generate Pattern**: Spiral inward from boundary
5. **Smooth Curves**: Apply Chaikin smoothing for curved turns
6. **Downsample**: Reduce waypoint density to 25m spacing
7. **Convert to GPS**: Transform back to lat/lon coordinates

## Configuration Options

### Camera Parameters

```python
camera_fov = 57      # Camera field of view in degrees
overlap = 0.25       # 25% coverage overlap
```

### Coverage Patterns

**Curved (Default)**: Spiral inward pattern with smooth turns
```python
pattern = "curved"
```

**Lawnmower**: Traditional parallel passes
```python
pattern = "lawnmower"
```

### Mission Parameters

```python
altitude_meters = 15.0    # Flight altitude AGL
min_altitude = 5.0        # Safety minimum
max_altitude = 100.0      # Safety maximum
max_waypoints = 500       # Mission size limit
```

## Usage Examples

### Example 1: Basic Survey

```bash
# From TX ground station
KML:SURVEY:park_search.kml,20
ARM
TAKEOFF:20
MODE:AUTO
```

### Example 2: Low Altitude Detail Survey

```bash
KML:SURVEY:building_scan.kml,10
ARM
TAKEOFF:10
MODE:AUTO
```

### Example 3: Large Area Survey

```bash
KML:SURVEY:forest_area.kml,30
ARM
TAKEOFF:30
MODE:AUTO
```

## Creating KML Files

### Google Earth Pro

1. Open Google Earth Pro
2. Click **Add Polygon** (toolbar)
3. Draw survey area by clicking corners
4. Name polygon (e.g., "Search Area Alpha")
5. Right-click polygon → **Save Place As...**
6. Save as `search_area.kml`

### Google Earth Web

1. Go to earth.google.com
2. Click menu → **Add placemark**
3. Select polygon tool
4. Draw boundary
5. Click **Export** → Download KML

### QGIS / Mission Planner

You can also create KML files using:
- **QGIS**: Vector layer → Export → KML
- **Mission Planner**: Fence → Export as KML

## KML File Format

Standard Google Earth KML with Polygon:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Survey Area</name>
    <Placemark>
      <name>Search Zone</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              lon1,lat1,0
              lon2,lat2,0
              lon3,lat3,0
              lon1,lat1,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
```

**Note**: KML coordinates are `lon,lat,alt` (longitude first!)

## Integration with Human Detection

The KML survey integrates with your existing detection system:

1. **Upload KML mission** using `KML:SURVEY` command
2. **Waypoints generated** with optimal camera coverage
3. **AUTO mode** executes survey pattern
4. **Detection runs** at each waypoint (5-second hover)
5. **Alerts logged** for human detections
6. **RTL** when survey complete

### Detection During Survey

Modify [main.py](main.py) to enable detection during AUTO mode:

```python
# In mission monitoring loop
if self.vehicle.mode.name == "AUTO":
    current_wp = self.vehicle.commands.next
    # Run detection at each waypoint
    if detection_enabled:
        detect_humans()
```

## Safety Features

### Mission Validation

Before upload, validates:
- Waypoint count < 500
- Altitude 5m - 100m
- Valid GPS coordinates

### Abort Options

```bash
ABORT          # Emergency land
RTL            # Return to home
MODE:LOITER    # Hold position
```

## Troubleshooting

### "KML file not found"

Check file is in `/home/dart/quadtest/missions/` directory:
```bash
ls -la /home/dart/quadtest/missions/
```

### "No polygon found in KML"

Ensure KML contains Polygon (not just Placemark or LineString)

### "Too many waypoints"

- Increase altitude (reduces pass count)
- Reduce survey area size
- Adjust overlap percentage

### "Mission upload failed"

- Check vehicle connection
- Verify GPS lock
- Try clearing existing mission first

## Advanced Configuration

### Environment Variables

```bash
# Mission source selection
export MISSION_SOURCE=kml              # or "static"

# KML file settings
export KML_FILE=missions/area.kml
export KML_ALTITUDE=15.0
export KML_PATTERN=curved              # or "lawnmower"

# Camera settings
export KML_CAMERA_FOV=57               # degrees
export KML_OVERLAP=0.25                # 25% overlap

# Run mission
python3 nidar/mission_control.py
```

### Python API Usage

```python
from mission.kml_loader import load_waypoints_from_kml

# Load waypoints
waypoints = load_waypoints_from_kml(
    kml_file="missions/survey.kml",
    altitude_meters=20.0,
    pattern="curved",
    camera_fov=57,
    overlap=0.25
)

# Upload to vehicle
from mission.waypoint_mission import WaypointMission
mission = WaypointMission(vehicle)
mission.upload_mission(waypoints)
```

## Performance

### Waypoint Count Estimates

| Area Size | Altitude | Pattern  | Waypoints |
|-----------|----------|----------|-----------|
| 100×100m  | 15m      | Curved   | ~40       |
| 200×200m  | 15m      | Curved   | ~120      |
| 500×500m  | 20m      | Curved   | ~250      |
| 100×100m  | 10m      | Lawnmower| ~60       |

### Mission Duration

Estimate: `waypoints × (5s hover + travel_time)`

Example: 100 waypoints @ 5m/s = ~15 minutes

## Dependencies

Required Python packages (already installed):

```bash
pip install dronekit pymavlink shapely lxml numpy
```

## Credits

- **NIDAR Path Planning**: Adapted from VTOL QuadPlane project
- **Shapely**: Polygon processing and buffering
- **lxml**: KML XML parsing
- **DroneKit**: ArduPilot MAVLink interface

## See Also

- [mission_data.py](nidar/mission/mission_data.py) - Mission configuration
- [kml_loader.py](nidar/mission/kml_loader.py) - KML integration
- [path_planner.py](nidar/kml_parsing/path_planner.py) - NIDAR algorithm
- [rx_commands.py](rx_commands.py) - Command interface

## Support

For issues or questions:
1. Check service logs: `sudo journalctl -u lora -n 100`
2. Verify KML file validity in Google Earth
3. Test with example KML first: `survey_area.kml`
