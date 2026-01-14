# Quadcopter Autonomous Surveillance System - Complete Project Context

## Project Overview

**Purpose:** Autonomous quadcopter drone system for area surveillance with real-time human detection capabilities, controlled via long-range LoRa/3DR radio link.

**Primary Use Case:** KML-based area survey missions with automated human detection, video recording, and GPS-tagged detection logging.

**Current Status:** Core system operational, human detection integrated, failsafe system implemented. Current hardware issue: RadioLink Pix6 battery voltage reading showing 0V (hardware connection problem, not software).

---

## Hardware Configuration

### Flight Controller
- **Model:** RadioLink Pix6 (replacing Pixhawk 2.4.8)
- **Firmware:** ArduCopter 4.6.3
- **Connection:** USB serial (`/dev/ttyACM0` or `/dev/ttyACM1`)
- **Baud Rate:** 115200
- **Protocol:** MAVLink

### Battery Configuration
- **Type:** 3S LiPo (11.1V nominal)
- **Monitor Type:** `BATT_MONITOR = 3` or `4` (Analog Voltage)
- **Current Issue:** Voltage reading 0V - power module not connected or faulty
- **Failsafe Thresholds:**
  - RTH (Return to Home): 8.0V
  - Critical Emergency Land: 7.5V

### Communication
- **Radio:** LoRa/3DR radio telemetry
- **Port:** `/dev/ttyUSB-radio` (symlink) or `/dev/ttyUSB0`
- **Baud Rate:** 57600
- **Protocol:** Custom text-based command protocol over serial

### Sensors & Cameras
- **Primary Camera:** Intel RealSense (preferred for human detection)
- **Fallback Camera:** USB webcam (OpenCV VideoCapture)
- **Detection Model:** YOLOv8 ONNX (`best.onnx`)
- **Detection Classes:** Person detection (class 0)

### Compute Platform
- **Platform:** Raspberry Pi (likely Pi 4 or 5)
- **OS:** Raspberry Pi OS (Linux)
- **Python Version:** 3.11+

---

## Software Architecture

### Main Components

#### 1. **main.py** - Central Controller
**Purpose:** Main entry point for drone operations, command processing, mission execution

**Key Classes:**
- `FailsafeMonitor` - Monitors battery voltage and radio link, triggers automatic RTH/emergency land
- `MainController` - Primary controller handling all drone operations

**Core Features:**
- LoRa radio command reception and processing
- Pixhawk/Pix6 connection via DroneKit
- Mission execution (waypoint navigation)
- Human detection integration
- Video recording during missions
- Telemetry logging to ground station
- Multi-threaded operation (main loop, detection thread, failsafe monitor)

#### 2. **tx_commands.py** - Ground Station
**Purpose:** Ground station interface for sending commands to drone

**Features:**
- Interactive command-line interface
- Command history and autocomplete
- Real-time telemetry display from drone
- KML file upload capability
- Status monitoring

#### 3. **yolo_detector.py** - Human Detection
**Purpose:** Real-time human detection using YOLOv8 ONNX model

**Features:**
- ONNX Runtime inference
- Configurable confidence threshold (default 0.6)
- Bounding box drawing
- Person counting
- Frame-by-frame processing

#### 4. **nidar/** - Mission Control Modules
**Purpose:** Reusable mission control and safety modules

**Structure:**
```
nidar/
├── mission/
│   ├── waypoint_mission.py    # Waypoint navigation
│   ├── mission_data.py        # Mission data structures
│   └── kml_loader.py          # KML mission import
├── kml_parsing/
│   ├── kml_parser.py          # KML file parsing
│   ├── path_planner.py        # Route optimization
│   ├── camera_params.py       # Camera field of view calculations
│   └── gps_utils.py           # GPS/coordinate utilities
├── safety/
│   ├── preflight.py           # Pre-flight safety checks
│   └── abort.py               # Emergency abort procedures
└── utils/
    ├── logger.py              # Mission logging
    ├── file_logger.py         # Session file logging
    ├── telemetry_logger.py    # Radio telemetry system
    └── connection.py          # Vehicle connection management
```

---

## Command Protocol

### Communication Flow
```
Ground Station (tx_commands.py) → LoRa Radio → Drone → main.py → Execute → Response
```

### Command Set

#### Basic Commands
- `PING` - Test connection
- `STATUS` - Get current drone status (GPS, battery, mode, altitude)
- `PREFLIGHT` - Run comprehensive preflight checks
- `ARM` - Arm motors
- `FORCEARM` - Force arm (bypass pre-arm checks)
- `DISARM` - Disarm motors

#### Flight Commands
- `TAKEOFF:alt` - Takeoff to altitude (e.g., `TAKEOFF:5`)
- `LAND` - Land immediately at current location
- `RTL` - Return to launch point
- `ABORT` - Emergency abort (immediate RTL)
- `MODE:xxx` - Change flight mode (e.g., `MODE:GUIDED`, `MODE:LOITER`)

#### Mission Commands
- `MISSION:START` - Start autonomous waypoint mission
- `MISSION:STOP` - Stop current mission
- `LOAD:filename.kml` - Load KML mission file

#### Human Detection Commands
- `DETECT:START` - Start human detection camera
- `DETECT:STOP` - Stop human detection
- `DETECT:STATUS` - Get detection status
- `DETECT:CONF:0.7` - Set confidence threshold (0.1-1.0)

#### Scout Commands (Integrated Surveillance)
- `SCOUT` - Start KML survey mission with auto-detection (uses default KML)
- `SCOUT:KML:file.kml,alt` - Custom KML survey at specified altitude
- `SCOUT:STOP` - Stop scouting, detection, save recording

#### Telemetry Control
- `TELEM:ON` - Enable telemetry streaming
- `TELEM:OFF` - Disable telemetry streaming

#### Test Commands
- `TEST:RTL` - Manually trigger RTL failsafe (for testing)
- `TEST:CRITICAL` - Manually trigger emergency land failsafe (for testing)

---

## Failsafe System (Mandatory Requirements)

### 1. Manual Return-To-Home
- **Trigger:** `RTL` or `ABORT` command from ground station
- **Action:** Return to launch point using ArduPilot RTL mode
- **Status:** ✓ Implemented

### 2. Automatic RTH on Radio Link Loss
- **Trigger:** No commands received for 30 seconds while armed and flying (>2m altitude)
- **Action:** Automatic RTH
- **Status:** ✓ Implemented
- **Implementation:** `FailsafeMonitor._monitor_loop()` - checks `time_since_last_cmd`

### 3. Automatic RTH on Low Battery
- **Trigger:** Battery voltage drops below 8.0V
- **Action:** Automatic RTH
- **Status:** ✓ Implemented (awaiting hardware fix)
- **Current Issue:** Battery voltage reading 0V due to power module connection

### 4. Emergency Landing on Critical Battery
- **Trigger:** Battery voltage drops below 7.5V
- **Action:** Immediate emergency landing at current location
- **Status:** ✓ Implemented (awaiting hardware fix)

### Implementation Details
```python
class FailsafeMonitor:
    battery_rtl_threshold = 8.0       # RTH at 8V
    battery_critical_threshold = 7.5   # Emergency land at 7.5V
    link_timeout = 30                  # RTH after 30s no commands
    
    # Monitors in background thread every 2 seconds
    # Triggers are one-shot (won't spam multiple RTL commands)
    # Integrates with mission abort system
```

---

## Mission Flow

### Standard Scout Mission
1. Ground station sends `SCOUT` or `SCOUT:KML:file.kml,alt`
2. Drone loads KML file (default: `missions/survey_area.kml`)
3. Parses KML into waypoints at specified altitude (default: 5m AGL)
4. Runs preflight checks
5. Arms drone
6. Takeoff to survey altitude
7. **Starts human detection** automatically
8. **Starts video recording** automatically
9. Navigates through waypoints
10. When human detected:
    - Logs detection with GPS coordinates
    - Sends alert to ground station: `HUMAN DETECTED: 2 person(s), conf=0.85, loc=37.7749,-122.4194,15.5m`
11. Returns to launch point
12. Lands
13. Saves recording
14. Mission complete

### Detection Features ("Smart Detection")
- **Frame skipping:** Processes every 3rd frame for performance (configurable)
- **Recording:** Saves ALL frames at 15fps (no skipping in video)
- **Confidence filtering:** Only reports detections above threshold
- **GPS tagging:** Each detection logged with drone's GPS location
- **Real-time alerts:** Sent to ground station immediately

---

## File Structure

### Core Files
```
main.py                    # Main drone controller
tx_commands.py             # Ground station interface
yolo_detector.py          # Human detection module
camera.py                 # Camera interface
rx_commands.py            # Radio receiver utilities
best.onnx                 # YOLOv8 detection model
```

### Configuration Files
```
setup_service.sh          # Systemd service installer
view_logs.sh              # Log viewer script
view_session_logs.sh      # Session log viewer
```

### Mission Files
```
missions/
└── survey_area.kml       # Default survey mission KML
```

### Diagnostic Tools
```
pix6_battery_diag.py          # Full battery diagnostic
pix6_battery_setup.py         # Configure battery parameters
pix6_deep_diag.py             # Deep MAVLink message diagnostic
pix6_list_params.py           # List all battery parameters
voltage_test_usb.py           # Voltage reading tests
voltage_monitor_continuous.py # Real-time voltage monitor
```

### Logs
```
logs/
└── sessions/
    └── YYYY-MM-DD/
        └── session_HH-MM-SS.log
```

### Documentation
```
ARCHITECTURE_FIX_README.md     # Architecture documentation
BEFORE_AFTER_COMPARISON.md     # Code evolution
CHANGES_SUMMARY.md             # Change log
IMPLEMENTATION_SUMMARY.md      # Implementation notes
KML_PROTOCOL_REFERENCE.md      # KML usage guide
KML_SURVEY_README.md           # Survey mission guide
QUICK_REFERENCE.md             # Command quick reference
QUICKSTART_KML.md              # KML quickstart
SCOUT_COMMAND_GUIDE.md         # Scout command documentation
SCOUT_UPDATE.md                # Scout feature updates
TX_SCOUT_SUMMARY.md            # Ground station guide
GCS_COPILOT_PROMPT.md          # Development prompt
```

---

## Dependencies

### Python Packages
```
dronekit==2.9.2           # MAVLink drone control
pymavlink                 # MAVLink protocol
pyserial                  # Serial communication
opencv-python (cv2)       # Camera/video
onnxruntime              # AI inference
numpy                    # Array operations
simplekml                # KML parsing
pyrealsense2             # RealSense camera (optional)
```

### System Requirements
```bash
# Installation
pip install dronekit pymavlink pyserial opencv-python onnxruntime numpy simplekml

# Optional (for RealSense)
pip install pyrealsense2
```

---

## Current Issues & Status

### ✓ Working Features
- Radio communication (LoRa/3DR)
- Command processing
- Mission execution (waypoint navigation)
- KML file parsing and mission planning
- Human detection (YOLOv8)
- Video recording
- Telemetry logging
- Session file logging
- Failsafe monitoring (code ready)

### ⚠ Known Issues

#### 1. Battery Voltage Reading (CRITICAL)
**Problem:** RadioLink Pix6 reports 0V battery voltage
**Diagnosis:**
- `BATT_MONITOR` parameter correctly set to 3 or 4
- MAVLink SYS_STATUS message shows 0mV
- BATTERY_STATUS shows cell voltages = 65535 (invalid data marker)
- POWER_STATUS shows Vservo = 65-74mV (essentially 0V)

**Root Cause:** Hardware - power module not connected or not working
- Power module 6-pin cable not in POWER1 port, OR
- Battery not connected to power module, OR
- Power module faulty/damaged, OR
- Voltage sense wire in cable damaged

**Impact:** 
- Failsafe battery monitoring cannot function
- Low battery RTH will not trigger
- Critical battery emergency land will not trigger
- Drone could fly until battery dies completely (dangerous!)

**Solution Required:**
1. Physically connect battery to power module (XT60/XT30 connector)
2. Connect power module 6-pin cable to Pix6 POWER1 port
3. Verify with multimeter: battery should read 11.1-12.6V
4. Power cycle Pix6 after connection
5. Run `python3 voltage_monitor_continuous.py` to verify

**Software Safeguard:** Code now skips battery checks if voltage = 0 (prevents false failsafes)

---

## Key Configuration Parameters

### ArduPilot Parameters (RadioLink Pix6)
```
BATT_MONITOR = 3 or 4      # Battery monitoring enabled
BATT_ARM_VOLT = 10.5       # Minimum voltage to arm
BATT_LOW_VOLT = 10.8       # Low battery warning
BATT_CRT_VOLT = 10.2       # Critical battery voltage
BATT_CAPACITY = 2200       # Battery capacity in mAh
BATT_FS_LOW_ACT = 0        # Failsafe action on low battery
BATT_FS_CRT_ACT = 0        # Failsafe action on critical battery
```

### Application Constants (main.py)
```python
SCOUT_ALTITUDE = 5.0                    # Default survey altitude (meters AGL)
battery_rtl_threshold = 8.0             # RTH voltage threshold
battery_critical_threshold = 7.5        # Emergency land threshold
link_timeout = 30                       # Radio link timeout (seconds)
detection_confidence = 0.6              # Human detection threshold
detection_skip_frames = 3               # Frame skip for performance
```

---

## Operational Procedures

### Pre-Flight Checklist
1. Connect battery to power module
2. Verify Pix6 LEDs indicate power (should change when battery connected)
3. Connect Pix6 via USB to Raspberry Pi
4. Start main controller: `python3 main.py`
5. Wait for "Failsafe monitoring started" message
6. Check battery voltage is reading correctly (not 0V)
7. From ground station, send `PREFLIGHT` command
8. Verify all checks pass
9. Send `ARM` command
10. Mission ready

### Mission Execution
1. `SCOUT` - Start default survey, OR
2. `SCOUT:KML:area.kml,10` - Custom survey at 10m altitude
3. Monitor telemetry for human detections
4. Mission auto-completes and lands
5. Video saved in `recordings/` directory

### Emergency Procedures
1. **Manual abort:** Send `ABORT` command
2. **Radio link loss:** Automatic RTH after 30s
3. **Low battery:** Automatic RTH at 8V
4. **Critical battery:** Automatic emergency land at 7.5V

---

## Development Notes

### Recent Changes (January 2026)
1. Implemented comprehensive failsafe system per requirements
2. Added battery voltage monitoring with automatic RTH/emergency land
3. Added radio link loss detection with automatic RTH
4. Created diagnostic tools for RadioLink Pix6
5. Fixed frame rate issues in video recording
6. Added session-based file logging
7. Integrated telemetry streaming to ground station

### Code Architecture Philosophy
- **Modular design:** Separate concerns (mission, safety, detection, telemetry)
- **Thread-safe:** Uses locks for shared state (abort_flag, mission_running)
- **Fail-safe defaults:** Assumes worst-case scenarios
- **Extensive logging:** All actions logged for post-flight analysis
- **Real-time feedback:** Commands get immediate responses

### Testing Methodology
- Ground testing with USB power (current mode)
- Bench testing with battery connected
- Tethered flight testing
- Full autonomous flight testing
- Failsafe triggering tests (TEST:RTL, TEST:CRITICAL commands)

---

## Future Enhancements (Potential)

### High Priority
1. **Fix battery voltage reading** - CRITICAL for safe operation
2. GPS waypoint tracking accuracy improvements
3. Multiple KML file management system
4. Real-time video streaming to ground station

### Medium Priority
1. Object tracking (follow detected humans)
2. Multiple drone coordination
3. Obstacle avoidance integration
4. Battery capacity estimation and remaining flight time
5. Wind compensation

### Low Priority
1. Web-based ground control station
2. Mobile app interface
3. Cloud logging/analytics
4. Advanced path planning algorithms

---

## Troubleshooting Guide

### Battery Voltage Shows 0V
**Symptoms:** Voltage always 0V, failsafe monitoring shows warning
**Tools:** Run `python3 pix6_battery_diag.py --port /dev/ttyACM1`
**Solution:** Check power module connections (see Current Issues section)

### Radio Disconnects Frequently
**Symptoms:** "Radio disconnected" messages, connection drops
**Tools:** Check `logs/sessions/` for error patterns
**Solution:** 
- Check radio power supply
- Verify baud rate (57600)
- Check antenna connections
- Reduce radio data rate

### Human Detection Not Working
**Symptoms:** No detections or false detections
**Tools:** Run detection standalone with test images
**Solution:**
- Adjust confidence threshold: `DETECT:CONF:0.5`
- Check camera is working: `ls /dev/video*`
- Verify ONNX model exists: `ls -lh best.onnx`
- Check lighting conditions (YOLO needs good lighting)

### Pixhawk Won't Arm
**Symptoms:** ARM command fails, pre-arm check errors
**Tools:** Send `PREFLIGHT` command for detailed checks
**Solution:**
- Check GPS lock (needs 6+ satellites)
- Check accelerometer calibration
- Verify flight mode is GUIDED
- Check battery voltage (if readable)

### Mission Won't Start
**Symptoms:** MISSION:START fails or immediate abort
**Tools:** Check session logs for error messages
**Solution:**
- Verify KML file loaded: `LOAD:missions/survey_area.kml`
- Check waypoints are valid (not too far, reasonable altitude)
- Ensure drone is armed
- Verify GPS has good fix

---

## API Reference

### MainController Class

#### Methods
```python
def connect_pixhawk() -> bool
    # Connects to Pixhawk/Pix6, initializes safety modules

def connect_radio() -> bool
    # Connects to LoRa/3DR radio for command reception

def execute_command(cmd: str)
    # Parses and executes received command

def cmd_status()
    # Returns current status (GPS, battery, mode, etc.)

def cmd_arm()
    # Arms the drone (runs preflight checks)

def cmd_takeoff(altitude: float)
    # Takeoff to specified altitude

def cmd_land()
    # Land at current location

def cmd_rtl()
    # Return to launch

def cmd_scout_start()
    # Start scout mission (KML survey + detection)

def cmd_detection_start()
    # Start human detection only

def send_response(message: str)
    # Send message back to ground station
```

### FailsafeMonitor Class

#### Methods
```python
def start()
    # Start background monitoring thread

def stop()
    # Stop monitoring

def update_command_time()
    # Reset radio link timeout (call on each command)

def _trigger_rtl(reason: str)
    # Trigger RTH with specified reason

def _trigger_emergency_land()
    # Trigger emergency landing
```

---

## Environment Variables & Paths

```bash
# Working directory
/home/dart/quadtest/

# Python environment
python3 (system Python 3.11+)
Virtual env: nidar/venv/ (if used)

# Serial devices
/dev/ttyACM0, /dev/ttyACM1    # Pixhawk/Pix6
/dev/ttyUSB0, /dev/ttyUSB1    # Radio
/dev/ttyUSB-radio             # Symlink to radio

# Log paths
logs/sessions/YYYY-MM-DD/session_HH-MM-SS.log

# Video recordings
recordings/*.avi

# Detection model
best.onnx (YOLOv8 ONNX format)
```

---

## Contact & Support

**Project Status:** Active development
**Last Updated:** January 15, 2026
**Current Focus:** Resolving RadioLink Pix6 battery voltage reading issue

---

## Quick Start Summary

```bash
# 1. Install dependencies
pip install dronekit pymavlink pyserial opencv-python onnxruntime numpy simplekml

# 2. Connect hardware
# - Pix6 via USB to Raspberry Pi
# - LoRa radio via USB
# - Battery to power module to Pix6 POWER1 port

# 3. Configure battery monitoring (one-time)
python3 pix6_battery_setup.py --port /dev/ttyACM1

# 4. Verify battery voltage
python3 voltage_monitor_continuous.py --port /dev/ttyACM1
# Should show >11V if battery connected

# 5. Start drone controller
python3 main.py --pixhawk /dev/ttyACM1 --radio /dev/ttyUSB0

# 6. From another terminal, start ground station
python3 tx_commands.py --port /dev/ttyUSB1

# 7. Send commands
STATUS
PREFLIGHT
ARM
SCOUT

# 8. Monitor logs
tail -f logs/sessions/*/session_*.log
```

---

**END OF PROJECT CONTEXT**

This document contains the complete project context for the autonomous quadcopter surveillance system. Use this to understand system architecture, current status, and operational procedures.
