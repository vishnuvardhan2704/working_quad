# SCOUT Command Quick Reference

## Overview
The `SCOUT` command initiates an autonomous KML area survey mission for human detection. Just send "SCOUT" from TX and the drone will execute the configured survey pattern.

## Quick Start

### From TX (Ground Station):
```
SCOUT
ARM
TAKEOFF:15
MODE:AUTO
```

That's it! The drone will execute the survey mission.

## Configuration on RPI

Edit [rx_commands.py](rx_commands.py) lines 52-54:

```python
DEFAULT_KML_FILE = "survey_area.kml"  # Your KML filename
DEFAULT_SCOUT_ALTITUDE = 15.0          # Flight altitude (meters)
DEFAULT_SCOUT_PATTERN = "curved"       # "curved" or "lawnmower"
```

## KML File Setup

1. **Place your KML file** in `/home/dart/quadtest/missions/`
2. **Update DEFAULT_KML_FILE** in rx_commands.py to match your filename
3. **Test the file** first with example area

### Example KML files location:
```bash
/home/dart/quadtest/missions/
├── survey_area.kml        # Default example
├── park_search.kml        # Your custom areas
└── field_scan.kml
```

## TX Commands Reference

### Basic SCOUT (uses default config):
```
SCOUT
```
Uses: DEFAULT_KML_FILE at DEFAULT_SCOUT_ALTITUDE

### Custom KML Survey:
```
KML:SURVEY:filename,altitude
```
Examples:
```
KML:SURVEY:park.kml,20
KML:SURVEY:field.kml,12
KML:SURVEY:building.kml,10
```

### Complete Mission Sequence:
```
SCOUT                  # Upload mission
ARM                    # Arm motors
TAKEOFF:15             # Takeoff to 15m
MODE:AUTO              # Start survey
# ... mission executes ...
# RTL automatically or manually:
RTL                    # Return home
```

## Mission Workflow

1. **TX sends:** `SCOUT`
2. **RX receives:** Loads missions/survey_area.kml
3. **RX generates:** Curved coverage waypoints
4. **RX uploads:** Mission to Pixhawk
5. **RX responds:** "Mission uploaded: X waypoints"
6. **TX sends:** `ARM` → `TAKEOFF:15` → `MODE:AUTO`
7. **Drone executes:** Survey pattern
8. **Camera detects:** Humans during flight
9. **Mission completes:** Auto RTL or manual RTL

## TX Output Examples

### Successful SCOUT:
```
CMD> SCOUT
  >> [DRONE] Loading KML survey: survey_area.kml
  >> [DRONE] Generated 6 waypoints
  >> [DRONE] Mission uploaded: 6 waypoints
  >> [DRONE] Ready: ARM -> TAKEOFF -> MODE:AUTO to begin survey
```

### During Mission:
```
  📡 [TELEM][INFO] Mission WP 1/6
  📷 DETECTION: Scanning frame 125
  🚨 HUMAN DETECTED at WP3!
  📡 [TELEM][INFO] Mission WP 4/6
```

## Customization

### Change Default KML File:
```bash
ssh dart@raspberrypi
cd /home/dart/quadtest
nano rx_commands.py
# Edit line 52: DEFAULT_KML_FILE = "your_file.kml"
```

### Change Default Altitude:
```bash
# Edit line 53: DEFAULT_SCOUT_ALTITUDE = 20.0
```

### Change Coverage Pattern:
```bash
# Edit line 54: DEFAULT_SCOUT_PATTERN = "lawnmower"
```

### Restart Service After Changes:
```bash
sudo systemctl restart lora
```

## Creating KML Files

### Google Earth:
1. Open Google Earth
2. Add Polygon tool
3. Draw survey area
4. Save As → `myarea.kml`
5. Copy to `/home/dart/quadtest/missions/`

### Coordinates Format:
KML uses `longitude,latitude,altitude`:
```xml
<coordinates>
  149.1650000,-35.3632000,0
  149.1655000,-35.3632000,0
</coordinates>
```

## Troubleshooting

### "KML file not found"
```bash
# Check file exists:
ls -la /home/dart/quadtest/missions/
# Verify filename matches DEFAULT_KML_FILE
```

### "No polygon found in KML"
- Ensure KML has `<Polygon>` not just `<Placemark>`
- Test in Google Earth first

### "Too many waypoints"
- Increase altitude (reduces passes)
- Make survey area smaller
- Adjust overlap: Edit nidar/mission/kml_loader.py line 38

### Mission won't start
```
STATUS           # Check status
PREFLIGHT        # Run checks
MODE:GUIDED      # Try switching modes
MODE:AUTO        # Then start mission
```

## Safety Notes

⚠️ **Before SCOUT:**
- Verify GPS lock (fix_type ≥ 3)
- Check battery voltage
- Confirm survey area is clear
- Set appropriate altitude for obstacles

⚠️ **During Mission:**
- Monitor telemetry for alerts
- Keep `ABORT` ready
- Watch battery levels
- Maintain visual line of sight

⚠️ **Emergency Abort:**
```
ABORT            # Immediate land
RTL              # Return to home
MODE:LOITER      # Hold position
```

## Integration with Human Detection

The SCOUT mission is designed for human detection:

- **5-second hover** at each waypoint
- **Camera scans** during hover
- **YOLO detection** identifies humans
- **Alerts sent** to TX via telemetry
- **Locations logged** for rescue teams

## Advanced Usage

### Sequential Surveys:
```
# Survey area 1
KML:SURVEY:zone1.kml,15
ARM
TAKEOFF:15
MODE:AUTO
# ... wait for completion ...
RTL
LAND

# Survey area 2
KML:SURVEY:zone2.kml,15
ARM
TAKEOFF:15
MODE:AUTO
```

### Variable Altitude Survey:
```
KML:SURVEY:lowres.kml,30   # Quick overview at 30m
# ... complete ...
KML:SURVEY:detail.kml,10   # Detailed scan at 10m
```

## File Locations

**Config:** `/home/dart/quadtest/rx_commands.py` (lines 52-54)
**KML files:** `/home/dart/quadtest/missions/*.kml`
**Logs:** `sudo journalctl -u lora -f`
**Documentation:** [KML_SURVEY_README.md](KML_SURVEY_README.md)

## Support

Test KML parsing:
```bash
cd /home/dart/quadtest
/home/dart/venv-ardupilot/bin/python3 -c "
import sys
sys.path.insert(0, 'nidar')
from mission.kml_loader import load_waypoints_from_kml
wps = load_waypoints_from_kml('missions/survey_area.kml', 15.0)
print(f'Loaded {len(wps)} waypoints')
"
```

View service logs:
```bash
sudo journalctl -u lora -n 100 -f
```
