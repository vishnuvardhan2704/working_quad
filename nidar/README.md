# Autonomous Drone Navigation System
## ArduPilot + DroneKit + QGroundControl

Production-quality autonomous waypoint navigation for ArduCopter using ArduPilot SITL simulation and real Pixhawk hardware.


sudo systemctl start lora
---
sudo journalctl -u lora -n 50 --no-pager
sudo journalctl -u lora -f

sudo systemctl restart lora
sudo systemctl status lora --no-pager
sudo journalctl -u lora -f

## 🎯 Project Overview

This system provides **fully autonomous waypoint navigation** with:
- ✅ **3 waypoints** in triangle pattern (~100 feet / 30 meters apart)
- ✅ **5-second hover** at each waypoint for inspection/photography
- ✅ **Automatic takeoff, navigation, and return-to-launch**
- ✅ **Emergency abort** capability (type `abort` anytime)
- ✅ **QGroundControl** integration for real-time telemetry and map visualization
- ✅ **Comprehensive pre-flight safety checks**
- ✅ **Mission-based AUTO mode** (ArduPilot native waypoint system)

### Mission Profile
```
1. Pre-flight checks (GPS, battery, armable, home location)
2. Auto-start countdown (3 seconds)
3. Arm & takeoff to 10 meters (GUIDED mode)
4. Switch to AUTO mode
5. Navigate to Waypoint 1 → Hover 5 seconds
6. Navigate to Waypoint 2 → Hover 5 seconds  
7. Navigate to Waypoint 3 → Hover 5 seconds
8. Return to Launch (RTL mode)
9. Land and disarm automatically
```

**Type `abort` at any time to safely stop the mission and return home.**

---

## 🏗️ System Architecture

### Project Structure
```
ardupilot_anav/
├── mission_control.py          # Main orchestrator (entry point)
├── requirements.txt            # Python dependencies
├── mission/
│   ├── mission_data.py        # Waypoint definitions (EDIT COORDINATES HERE)
│   └── waypoint_mission.py    # Mission upload & AUTO mode control
├── safety/
│   ├── preflight.py           # Pre-flight validation checks
│   └── abort.py               # RTL and emergency procedures
└── utils/
    ├── connection.py          # DroneKit vehicle connection
    └── logger.py              # Color-coded console logging
```

### Component Responsibilities

#### **mission_control.py** (Main Entry Point)
- Orchestrates entire mission sequence
- Manages connection to vehicle (SITL or real Pixhawk)
- Executes pre-flight checks
- Handles arm, takeoff, and mode transitions
- Runs abort listener in separate thread
- Monitors mission progress

#### **mission/mission_data.py** (Waypoint Configuration)
- Defines mission waypoints using `LocationGlobalRelative`
- ArduPilot coordinate system: (latitude, longitude, altitude_above_home)
- **Edit this file to customize your mission waypoints**
- Default: 3 waypoints in triangle pattern for SITL

#### **mission/waypoint_mission.py** (Mission Execution)
- Uploads mission to ArduCopter using `MAV_CMD_NAV_WAYPOINT`
- Sets 5-second hover time at each waypoint (param1=5)
- Switches vehicle to AUTO mode for autonomous navigation
- Monitors waypoint progress
- Uses ArduPilot native mission protocol (not setpoint streaming)

#### **safety/preflight.py** (Pre-Flight Validation)
- GPS lock check (minimum satellites, 3D fix)
- Vehicle armable check
- Battery level verification (minimum 50%)
- Home location set confirmation (60-second timeout for SITL)
- All checks must pass before mission starts

#### **safety/abort.py** (Emergency Procedures)
- `return_to_launch()`: Switch to RTL mode, navigate home, land
- `emergency_land()`: Immediate descent at current location
- `monitor_landing()`: Track altitude until touchdown

#### **utils/connection.py** (Vehicle Connection)
- Connects to vehicle via MAVLink (UDP, TCP, serial)
- Default SITL: `udp:127.0.0.1:14550`
- Real Pixhawk: `/dev/ttyACM0` or `/dev/ttyUSB0`
- Handles connection timeouts and errors

#### **utils/logger.py** (Logging System)
- Color-coded console output (INFO, WARNING, ERROR, SUCCESS, STATE)
- Timestamped messages
- Clean, readable mission status updates

---

## 📦 Installation & Setup

### 1. Prerequisites

**System Requirements:**
- Ubuntu 20.04+ (or similar Linux)
- Python 3.8+
- Git

**Install ArduPilot SITL:**
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Add to `~/.bashrc`:
```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Reload:
```bash
source ~/.bashrc
```

**Install QGroundControl (Optional but Recommended):**
```bash
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

### 2. Project Setup

**Clone or navigate to project:**
```bash
cd ~/ardupilot_anav
```

**Create Python virtual environment:**
```bash
python3 -m venv venv
source venv/bin/activate
```

**Install Python dependencies:**
```bash
pip install -r requirements.txt
```

**Dependencies installed:**
- `dronekit` (from GitHub master - Python 3.12 compatible)
- `pymavlink` (MAVLink protocol library)
- `future` (Python 2/3 compatibility)

---

## ⚙️ Configuration Guide

### Scout Altitude (Scouting Missions)

The default altitude for all scouting missions (both waypoint-based and KML-based) is **5 meters AGL** (Above Ground Level).

#### **How to Change Scout Altitude**

**Option 1: Edit Global Constant (Permanent)**

Edit `../main.py` (line 108):
```python
SCOUT_ALTITUDE = 5.0  # Change this value to your desired altitude in meters
```

Example:
```python
SCOUT_ALTITUDE = 15.0  # Scout at 15 meters instead
```

This single constant controls all scouting altitudes throughout the system.

**Option 2: Change During Runtime (Temporary)**

Send the `ALT` command via radio/ground station:
```
ALT:20  # Set scout altitude to 20 meters
```

This change only affects the current session and will revert to the default value on next restart.

**Option 3: Override in KML Survey Commands**

When starting a KML-based survey, specify altitude:
```
SCOUT:KML:survey_area.kml,15  # Survey at 15 meters altitude
```

#### **Scout Altitude Parameters Summary**
| Parameter | File | Line | Current Value | Notes |
|-----------|------|------|---------------|-------|
| `SCOUT_ALTITUDE` | `../main.py` | 108 | 5.0 m | Global constant for all scouting |
| `self.scout_altitude` | `../main.py` | 160 | Uses SCOUT_ALTITUDE | Waypoint scouting altitude |
| `self.default_kml_altitude` | `../main.py` | 164 | Uses SCOUT_ALTITUDE | KML survey altitude |

**Recommended Ranges:**
- **2-5m**: Low-altitude survey, detailed imagery, low speed
- **5-15m**: Balanced coverage and detail, typical survey altitude
- **15-30m**: Wider coverage, faster mission, less detail

---

## 🚀 How to Run the Mission

### SITL Simulation (3 Terminals)

#### **Terminal 1: Start ArduPilot SITL with Multiple Outputs**
```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14551
```

**This is the official ArduPilot method with multiple outputs:**
- Port **14550**: QGroundControl (default)
- Port **14551**: DroneKit mission control

**Wait for GPS lock** (~30-60 seconds). Look for:
```
AP: EKF3 IMU0 origin set
AP: EKF3 IMU1 origin set
```

#### **Terminal 2: Launch QGroundControl (Optional)**
```bash
~/Downloads/QGroundControl.AppImage
```

QGroundControl will:
- Auto-connect to SITL on UDP port 14550
- Display live map with flight path
- Show telemetry: altitude, speed, battery, GPS
- Visualize waypoint markers and mission progress

#### **Terminal 3: Run Mission Control**
```bash
cd ~/ardupilot_anav
source venv/bin/activate
python3 mission_control.py --connect udp:127.0.0.1:14551
```

**Note:** DroneKit connects to port **14551** (separate from QGC on 14550)

**Mission will auto-start after 3-second countdown!**

To abort mission at any time:
```
abort
```

---

## 🗺️ Waypoint Coordinate System

### ArduPilot Coordinate Format

**Uses `LocationGlobalRelative`:**
```python
LocationGlobalRelative(latitude, longitude, altitude)
```

- **Latitude**: Decimal degrees (North positive, South negative)
- **Longitude**: Decimal degrees (East positive, West negative)  
- **Altitude**: Meters **above home location** (not above sea level)

This is ArduPilot's standard coordinate system for waypoint missions.

### Default SITL Waypoints

Located in `mission/mission_data.py`:

```python
waypoints = [
    # Waypoint 1: Start
    LocationGlobalRelative(-35.3632607, 149.1652351, 10.0),
    
    # Waypoint 2: ~30m East
    LocationGlobalRelative(-35.3632607, 149.1655351, 10.0),
    
    # Waypoint 3: ~30m South from WP2
    LocationGlobalRelative(-35.3635107, 149.1655351, 10.0),
]
```

**Coordinates are for ArduPilot SITL default location** (Canberra, Australia).

### Customizing Waypoints for Real Flights

1. Open `mission/mission_data.py`
2. Replace coordinates with your GPS locations
3. Keep altitude relative to home (e.g., 10.0 = 10 meters above takeoff point)
4. Ensure adequate spacing between waypoints (minimum 10 meters recommended)

**Example for custom location:**
```python
waypoints = [
    LocationGlobalRelative(37.7749, -122.4194, 15.0),  # San Francisco
    LocationGlobalRelative(37.7750, -122.4194, 15.0),  # 11m North
    LocationGlobalRelative(37.7750, -122.4184, 15.0),  # 76m East
]
```

**Tip:** Use Google Maps to get coordinates:
1. Right-click location → "What's here?"
2. Copy lat/lon (e.g., `37.7749, -122.4194`)
3. Paste into `LocationGlobalRelative(lat, lon, altitude)`

---

## 🎮 Mission Control Interface

### Console Output Example

```
============================================================
AUTONOMOUS DRONE NAVIGATION SYSTEM
============================================================

[12:34:56] INFO: ArduPilot + DroneKit Mission Control
[12:34:56] INFO: Connection: udp:127.0.0.1:14550
[12:34:56] INFO: Type 'abort' at any time to stop the mission

============================================================
PRE-FLIGHT CHECKS
============================================================
[12:34:58] INFO: Checking GPS lock...
[12:34:59] SUCCESS: ✓ GPS lock acquired (14 satellites, 3D fix)
[12:35:00] SUCCESS: ✓ Vehicle is armable
[12:35:00] SUCCESS: ✓ Battery level OK: 100%
[12:35:01] SUCCESS: ✓ Home location set
[12:35:01] SUCCESS: ✓ All pre-flight checks PASSED

============================================================
MISSION UPLOAD
============================================================
[12:35:02] INFO: Mission: 3 waypoints at 10.0m altitude
[12:35:02] INFO: Uploading mission with 3 waypoints...
[12:35:02] INFO:   WP1: (-35.363261, 149.165235, 10.0m) [hover 5s]
[12:35:02] INFO:   WP2: (-35.363261, 149.165535, 10.0m) [hover 5s]
[12:35:03] INFO:   WP3: (-35.363511, 149.165535, 10.0m) [hover 5s]
[12:35:03] SUCCESS: Mission uploaded: 3 waypoints

============================================================
AUTO-START SEQUENCE
============================================================
[12:35:04] INFO: Mission ready - starting in 3 seconds...
[12:35:04] INFO: Takeoff altitude: 10.0m
[12:35:04] INFO: Waypoints: 3 (5-second hover at each)
[12:35:04] INFO: Starting in 3...
[12:35:05] INFO: Starting in 2...
[12:35:06] INFO: Starting in 1...

[12:35:07] INFO: Setting GUIDED mode...
[12:35:08] SUCCESS: GUIDED mode set
[12:35:08] INFO: Arming motors...
[12:35:10] SUCCESS: Vehicle armed
[12:35:10] STATE: Taking off to 10.0m...
[12:35:12] INFO: Altitude: 2.3m / 10.0m
[12:35:14] INFO: Altitude: 5.8m / 10.0m
[12:35:16] INFO: Altitude: 9.1m / 10.0m
[12:35:17] SUCCESS: Reached target altitude: 9.6m

============================================================
AUTONOMOUS NAVIGATION
============================================================
[12:35:18] INFO: Switching to AUTO mode for waypoint navigation
[12:35:19] SUCCESS: AUTO mode activated
[12:35:22] STATE: Waypoint 1/3 | Alt: 10.1m | Speed: 3.2m/s
[12:35:35] STATE: Waypoint 2/3 | Alt: 10.0m | Speed: 3.1m/s
[12:35:48] STATE: Waypoint 3/3 | Alt: 9.9m | Speed: 3.3m/s
[12:36:01] SUCCESS: All waypoints reached!

============================================================
MISSION COMPLETE - RETURNING HOME
============================================================
[12:36:02] INFO: Switching to RTL mode
[12:36:03] SUCCESS: RTL mode activated - returning to launch
[12:36:05] INFO: Altitude: 9.8m | Distance to home: 42.3m
[12:36:10] INFO: Altitude: 8.1m | Distance to home: 18.5m
[12:36:15] INFO: Altitude: 3.2m | Distance to home: 2.1m
[12:36:18] SUCCESS: Landed - altitude: 0.1m
[12:36:19] SUCCESS: Vehicle disarmed

[12:36:20] SUCCESS: ✅ Mission completed successfully!
```

### QGroundControl Display

**Telemetry Panel:**
- GPS: Satellites, HDOP, 3D fix status
- Altitude: Relative, Absolute, Ground speed
- Battery: Voltage, current, remaining %
- Mode: GUIDED → AUTO → RTL → LAND

**Map View:**
- Home position marker (launch point)
- Waypoint markers (WP1, WP2, WP3)
- Flight path line (real-time)
- Current vehicle position
- Distance/bearing to next waypoint

---

## 🆘 Emergency Abort

### Software Abort (Mission Control)

**Type in Terminal 3:**
```
abort
```

**What happens:**
1. Abort flag set immediately
2. Mission stops at current location
3. Vehicle switches to RTL mode
4. Navigates back to launch point
5. Automatic landing
6. Safe disarm

**Abort can be triggered:**
- During countdown (before arm)
- During takeoff
- During waypoint navigation
- At any point in the mission

### Hardware Abort (RC Transmitter)

If using real Pixhawk with RC transmitter:
1. Switch to **STABILIZE** or **LOITER** mode
2. Take manual control
3. Land manually or use RTL switch

### Critical Emergency

**RC Kill Switch:** Immediately disarms motors (use only in emergency - drone will drop!)

---

## 🔧 Real Hardware Deployment

### Pixhawk Configuration

1. **Flash ArduCopter Firmware:**
   - Use Mission Planner or QGroundControl
   - Select correct board: Pixhawk 2.4.8
   - Flash latest stable ArduCopter

2. **Calibrate Sensors:**
   - Compass calibration (mandatory)
   - Accelerometer calibration
   - Radio calibration (RC transmitter)
   - ESC calibration

3. **Configure Parameters:**
   - Frame type (Quad X, Quad +, etc.)
   - Battery failsafe thresholds
   - Geofence (altitude/distance limits)
   - RTL altitude

### Connection String

**Change in `mission_control.py`:**

For USB connection:
```python
controller = MissionController('/dev/ttyACM0')
```

For telemetry radio (e.g., 3DR Radio):
```python
controller = MissionController('/dev/ttyUSB0')
```

Or run with argument:
```bash
python3 mission_control.py --connect /dev/ttyACM0
```

### Safety Checklist

- [ ] All sensors calibrated
- [ ] GPS has 3D fix (minimum 10 satellites)
- [ ] Battery fully charged
- [ ] Propellers correctly installed (check rotation)
- [ ] Flight area clear of obstacles
- [ ] RC transmitter ready for manual takeover
- [ ] Waypoints verified (realistic coordinates)
- [ ] Geofence configured
- [ ] Emergency procedures reviewed

### First Flight Procedure

1. **Ground Test:** Connect laptop, verify telemetry, test arm/disarm
2. **Manual Test Flight:** Fly in STABILIZE/LOITER mode first
3. **Guided Takeoff Test:** Use `arm_and_takeoff()` only (no waypoints)
4. **Single Waypoint Test:** Upload one waypoint, test AUTO mode
5. **Full Mission:** Run complete 3-waypoint mission

**Always have RC transmitter ready to switch to manual mode!**

---

## 📡 Real-Time Telemetry Logging

The system includes comprehensive telemetry logging that sends all important events to your ground station (QGroundControl) in real-time.

### What's Logged to Ground Station

- **Pre-arm checks**: GPS, EKF, battery, compass status
- **Failsafe events**: Battery low/critical, GPS loss, GCS timeout
- **System status**: Mode changes, arm/disarm events
- **Sensor health**: Accelerometer, gyro, magnetometer issues
- **Battery monitoring**: Voltage drops, low voltage warnings
- **GPS status**: Fix type changes, satellite count
- **Flight controller messages**: All STATUSTEXT from ArduPilot

### Viewing Logs in QGroundControl

1. Open QGroundControl
2. Connect to your vehicle
3. Click the **speech bubble icon** (Messages) in the top toolbar
4. All telemetry logs appear here in real-time

### Using Telemetry with Mission Control

Telemetry is **enabled by default**. All mission logs are sent to the ground station:

```bash
# Normal run with telemetry (default)
python3 mission_control.py --connect /dev/ttyUSB0

# Disable telemetry if needed
python3 mission_control.py --connect /dev/ttyUSB0 --no-telemetry
```

### Standalone Telemetry Monitor

For continuous monitoring without running a mission, use the dedicated telemetry monitor:

```bash
# Monitor via USB
python3 telemetry_monitor.py --connect /dev/ttyACM0

# Monitor via telemetry radio
python3 telemetry_monitor.py --connect /dev/ttyUSB0

# SITL monitoring
python3 telemetry_monitor.py --connect udp:127.0.0.1:14551

# Custom status report interval (default 10 seconds)
python3 telemetry_monitor.py --connect /dev/ttyUSB0 --interval 5
```

### Telemetry Message Severity Levels

Messages in QGC are color-coded by severity:

| Level | Color | Description |
|-------|-------|-------------|
| EMERGENCY | Red | System unusable |
| ALERT | Red | Immediate action required |
| CRITICAL | Red | Critical failure |
| ERROR | Orange | Error conditions |
| WARNING | Yellow | Warning conditions |
| NOTICE | Blue | Significant events (mode changes, arm) |
| INFO | White | Informational |
| DEBUG | Gray | Debug messages |

### Telemetry Files Structure

```
utils/
├── logger.py              # Console logging (now with telemetry forwarding)
└── telemetry_logger.py    # MAVLink telemetry system
    ├── TelemetryLogger      # Sends STATUSTEXT messages
    ├── VehicleStatusMonitor # Continuous status monitoring
    ├── PrearmCheckReporter  # Pre-arm check reporting
    └── FailsafeMonitor      # Failsafe configuration/events
```

---

## 🧪 Troubleshooting

### Common Issues

#### "No heartbeat in last 5 seconds"
**Cause:** SITL not running or GPS not locked  
**Solution:**
```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```
Wait for GPS lock (30-60 seconds)  
Check for "EKF3 IMU0 origin set" message

#### "Pre-flight checks failed - GPS"
**Cause:** GPS hasn't locked yet in SITL  
**Solution:**
- SITL GPS takes 30-60 seconds to initialize
- Wait for EKF (Extended Kalman Filter) to set origin
- Check SITL console for GPS status messages

#### "ModuleNotFoundError: No module named 'dronekit'"
**Cause:** Virtual environment not activated  
**Solution:**
```bash
cd ~/ardupilot_anav
source venv/bin/activate
python3 mission_control.py
```

#### "ModuleNotFoundError: No module named 'pexpect'"
**Cause:** Running SITL from within venv  
**Solution:**
- SITL requires system Python (not venv)
- Run `deactivate` before starting SITL
- Or use: `cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter`

#### Mission doesn't start after countdown
**Cause:** Pre-flight checks failed or abort triggered  
**Solution:**
- Check console for error messages
- Verify all pre-flight checks passed (green checkmarks)
- Ensure you didn't type "abort" during countdown

#### Vehicle doesn't navigate to waypoints
**Cause:** Mission not uploaded or AUTO mode not activated  
**Solution:**
- Check console: "Mission uploaded: 3 waypoints"
- Verify "AUTO mode activated" message
- Check QGC: waypoint markers should be visible
- Ensure waypoints are reasonable distance apart (>10m)

---

## 🎓 Code Deep Dive

### Mission Upload Process

**File:** `mission/waypoint_mission.py`

```python
def upload_mission(self, waypoints):
    cmds = self.vehicle.commands
    cmds.clear()
    
    for i, waypoint in enumerate(waypoints):
        cmd = Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Altitude relative to home
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,           # Waypoint command
            0, 0,
            5, 0, 0, 0,  # param1=5sec hold, param2-4=unused
            waypoint.lat,
            waypoint.lon,
            waypoint.alt
        )
        cmds.add(cmd)
    
    cmds.upload()
```

**MAV_CMD_NAV_WAYPOINT Parameters:**
- `param1`: Hold time (seconds) - **5 seconds for inspection/hover**
- `param2`: Acceptance radius (meters) - 0 = use default
- `param3`: Pass-through radius - 0 = stop at waypoint
- `param4`: Yaw angle - 0 = don't change heading
- `x`: Latitude (decimal degrees)
- `y`: Longitude (decimal degrees)
- `z`: Altitude (meters above home)

### Coordinate System Verification

ArduPilot uses **LocationGlobalRelative** which matches the firmware's coordinate system:
- **MAV_FRAME_GLOBAL_RELATIVE_ALT**: Latitude/longitude in WGS84, altitude relative to home
- This is the standard for ArduCopter waypoint missions
- QGroundControl uses the same coordinate system
- Compatible with Mission Planner and all ArduPilot GCS software

### AUTO Mode Navigation

**How it works:**
1. GUIDED mode for takeoff (manual altitude control)
2. Switch to AUTO mode (ArduPilot takes over)
3. ArduPilot's mission controller navigates waypoints
4. Vehicle hovers at each waypoint for 5 seconds
5. After last waypoint, mission script triggers RTL

**No setpoint streaming** - uses ArduPilot's native mission system.

### Abort Mechanism

**Threading model:**
```python
# Main thread: Mission execution
# Daemon thread: Listens for 'abort' input

abort_flag = False  # Shared state
abort_lock = threading.Lock()  # Thread safety

def abort_listener(controller):
    while not controller.abort_flag:
        user_input = input().strip().lower()
        if user_input == 'abort':
            with controller.abort_lock:
                controller.abort_flag = True
            break

# Main mission loop checks abort flag
with self.abort_lock:
    if self.abort_flag:
        self.safety.return_to_launch()
        return False
```

**Thread-safe abort** prevents race conditions.

### Pre-Flight Validation

**File:** `safety/preflight.py`

```python
def run_all_checks(self):
    checks = [
        ("GPS lock", self.check_gps),
        ("Vehicle armable", self.check_armable),
        ("Battery level", self.check_battery),
        ("Home location", self.check_home_location),
    ]
    
    for name, check_func in checks:
        if not check_func():
            return False
    return True
```

**GPS Check:**
- Minimum 6 satellites (SITL: often 14)
- 3D fix required
- EPH/EPV values within limits

**Home Location Check:**
- Waits up to 60 seconds (SITL needs time)
- Verifies lat/lon are non-zero
- Ensures EKF origin is set

---

## 📊 Performance & Specifications

### Flight Performance (SITL)
- Takeoff speed: ~3 m/s vertical
- Cruise speed: ~3-5 m/s horizontal
- Waypoint accuracy: ~1-2 meters
- Hover stability: ±0.5 meters
- GPS update rate: 10 Hz

### Mission Timing (Typical)
- Pre-flight checks: 5-10 seconds
- Takeoff to 10m: 3-5 seconds
- Per waypoint (30m travel + 5s hover): ~15 seconds
- Total waypoint time: ~45 seconds (3 waypoints)
- RTL and landing: 10-15 seconds
- **Total mission duration: ~1-2 minutes**

### Resource Usage
- Python memory: ~50-80 MB
- CPU usage: <5% (mostly idle)
- Network: Negligible (MAVLink ~50 KB/s)

---

## 📄 License & Credits

**Built with:**
- [ArduPilot](https://ardupilot.org/) - Open-source autopilot
- [DroneKit-Python](https://github.com/dronekit/dronekit-python) - Python API for drones
- [MAVLink](https://mavlink.io/) - Communication protocol
- [QGroundControl](http://qgroundcontrol.com/) - Ground control station

**Architecture:**
- Mission-based navigation (not offboard control)
- ArduPilot native flight modes (GUIDED, AUTO, RTL)
- No ROS dependencies required
- Compatible with Pixhawk 2.4.8 and newer hardware

---

## 📞 Support

**Documentation:**
- ArduPilot: https://ardupilot.org/copter/
- DroneKit: https://dronekit-python.readthedocs.io/
- MAVLink: https://mavlink.io/en/

**Community:**
- ArduPilot Forum: https://discuss.ardupilot.org/
- DroneKit Forum: https://discuss.dronekit.io/

**Safety Reminder:**  
Always follow local drone regulations, maintain line-of-sight, and have manual override capability when flying real hardware.

---

**Happy Flying! 🚁**
