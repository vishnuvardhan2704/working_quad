# TX → SCOUT Command Changes Summary

## What Changed

### On RX Side (Raspberry Pi)

**File:** [rx_commands.py](rx_commands.py)

**Lines 52-54** - Added default configuration:
```python
DEFAULT_KML_FILE = "survey_area.kml"     # KML filename in missions/
DEFAULT_SCOUT_ALTITUDE = 15.0            # Flight altitude (meters AGL)
DEFAULT_SCOUT_PATTERN = "curved"         # Coverage pattern type
```

**Line ~230** - Added SCOUT command handler:
```python
elif cmd == "SCOUT":
    self.cmd_kml_survey(DEFAULT_KML_FILE, DEFAULT_SCOUT_ALTITUDE)
```

### On TX Side (Laptop)

**File:** [tx_commands.py](tx_commands.py)

**Lines 7-8** - Updated command list:
```python
Commands:
    SCOUT         - Start KML area survey mission (human detection)
    KML:SURVEY:file,alt - Custom KML survey (e.g. KML:SURVEY:area.kml,20)
```

## How It Works

### Simple Flow:
```
TX (Laptop)         →  SCOUT  →         RX (RPI)
                                        ↓
                                    Load missions/survey_area.kml
                                        ↓
                                    Generate waypoints (NIDAR)
                                        ↓
                                    Upload to Pixhawk
                                        ↓
TX (Laptop)         ←  "Ready"  ←      RX (RPI)
```

### Complete Mission:
```bash
# From TX ground station:
SCOUT              # 1. Upload mission
ARM                # 2. Arm motors  
TAKEOFF:15         # 3. Takeoff to 15m
MODE:AUTO          # 4. Start survey
# Mission executes automatically
RTL                # 5. Return home (or auto RTL)
```

## Configuration

### To change default KML file on RPI:

```bash
ssh dart@raspberrypi
nano /home/dart/quadtest/rx_commands.py
```

Edit line 52:
```python
DEFAULT_KML_FILE = "your_custom_area.kml"
```

Save and restart:
```bash
sudo systemctl restart lora
```

### To use custom file without changing default:

From TX:
```
KML:SURVEY:park_search.kml,20
```

## Your KML Files

**Location:** `/home/dart/quadtest/missions/`

**Example provided:** `survey_area.kml` (50m × 66m rectangle)

**Create new:** Use Google Earth → Draw Polygon → Save as KML

## Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `SCOUT` | Use default config | `SCOUT` |
| `KML:SURVEY:file,alt` | Custom survey | `KML:SURVEY:park.kml,20` |
| `ARM` | Arm motors | `ARM` |
| `TAKEOFF:X` | Takeoff to X meters | `TAKEOFF:15` |
| `MODE:AUTO` | Start mission | `MODE:AUTO` |
| `RTL` | Return to launch | `RTL` |
| `ABORT` | Emergency land | `ABORT` |
| `STATUS` | Get status | `STATUS` |

## What Happens During SCOUT

1. **RX receives SCOUT** command from TX
2. **Loads KML** from `/home/dart/quadtest/missions/survey_area.kml`
3. **Parses polygon** boundary from KML
4. **Generates coverage path** using NIDAR algorithm:
   - Curved center-shrinking spiral pattern
   - Optimized for 57° camera FOV
   - 25% overlap for complete coverage
5. **Creates waypoints** (DroneKit LocationGlobalRelative)
6. **Validates mission** (altitude, waypoint count, GPS coords)
7. **Uploads to Pixhawk** via MAVLink
8. **Sends confirmation** to TX: "Mission uploaded: X waypoints"
9. **Waits for ARM/TAKEOFF/AUTO** commands

## TX Changes Made

✅ Updated command list in help text
✅ No new code needed - SCOUT is handled on RX side
✅ Just send "SCOUT" as a simple command

## RX Changes Made

✅ Added default configuration (3 lines)
✅ Added SCOUT command handler (1 line)
✅ Existing KML:SURVEY handler already works
✅ All path planning modules created

## Files Created

- `nidar/kml_parsing/` - Path planning modules
  - `kml_parser.py` - Extract polygons from KML
  - `gps_utils.py` - GPS ↔ XY conversion
  - `camera_params.py` - FOV calculations
  - `path_planner.py` - NIDAR algorithm
  
- `nidar/mission/kml_loader.py` - DroneKit integration

- `missions/survey_area.kml` - Example survey area

- `KML_SURVEY_README.md` - Full documentation
- `SCOUT_COMMAND_GUIDE.md` - Quick reference
- `TX_SCOUT_SUMMARY.md` - This file

## Dependencies Installed

✅ `shapely` - Polygon processing (installed in venv-ardupilot)

## Testing

Tested successfully:
```bash
✓ KML parsing
✓ NIDAR path generation  
✓ Waypoint creation
✓ Mission validation
✓ Import paths correct
```

## Next Steps

1. **Copy your KML file** to RPI: `scp myarea.kml dart@raspberrypi:~/quadtest/missions/`

2. **Update default** (optional): Edit `DEFAULT_KML_FILE` in rx_commands.py

3. **Test from TX:**
   ```
   SCOUT
   STATUS
   ARM
   TAKEOFF:15
   MODE:AUTO
   ```

4. **Monitor:**
   ```bash
   # On RPI:
   sudo journalctl -u lora -f
   ```

## Example Session

```
TX> SCOUT
  >> [DRONE] Loading KML survey: survey_area.kml
  >> [DRONE] Generated 6 waypoints
  >> [DRONE] Mission uploaded: 6 waypoints
  >> [DRONE] Ready: ARM -> TAKEOFF -> MODE:AUTO to begin survey

TX> ARM
  >> [DRONE] Armed

TX> TAKEOFF:15
  >> [DRONE] Taking off to 15.0m

TX> MODE:AUTO
  >> [DRONE] Mode changed to AUTO
  📡 [TELEM][INFO] Mission started: WP 1/6
  📡 [TELEM][INFO] Scanning for humans...
  🚨 HUMAN DETECTED at waypoint 3!
  📡 [TELEM][INFO] Mission WP 6/6 complete
  📡 [TELEM][INFO] Returning to launch

TX> STATUS
  >> STATUS: AUTO,ARM,ALT=15.2m,BAT=12.4V,GPS=3
```

## That's It!

You now have a single-word `SCOUT` command that:
- Loads your predefined KML area
- Generates optimal coverage pattern
- Uploads mission to drone
- Ready to execute with ARM/TAKEOFF/AUTO

**No need to specify file or altitude each time - it's all configured on the RPI side!**
