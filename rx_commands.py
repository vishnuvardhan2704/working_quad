#!/usr/bin/env python3
"""
Drone Command Receiver via LoRa/3DR Radio (BASIC VERSION)

NOTE: For human detection and autonomous scouting missions, use main.py instead!
      main.py has DETECT:START, MISSION:START, and human detection features.
      This file (rx_commands.py) is for basic manual flight control only.

Listens for commands from another laptop via 3DR radio and executes them on the Pixhawk.
INTEGRATED with nidar/ modules for safety checks and proper logging.

Commands Supported:
    ARM           - Arm the drone (with preflight checks)
    DISARM        - Disarm the drone
    TAKEOFF:5     - Takeoff to 5 meters
    LAND          - Land the drone (emergency land)
    RTL           - Return to launch
    MODE:STABILIZE - Change flight mode
    MODE:LOITER   - Change to loiter mode
    MODE:GUIDED   - Change to guided mode
    GOTO:lat,lon,alt - Go to GPS location
    MOVE:n,e,d    - Move relative (north, east, down in meters)
    STOP          - Stop and hover (BRAKE mode)
    PREFLIGHT     - Run preflight checks only
    ABORT         - Emergency abort (immediate land)
    SCOUT         - Start KML area survey mission (uses default config)
    KML:SURVEY:filename,altitude - Custom KML survey with specific file
    
Hardware Setup:
    - Pixhawk connected via USB (/dev/ttyACM0)
    - 3DR Radio connected via USB (/dev/ttyUSB0) for receiving commands

Usage:
    python3 rx_commands.py
    python3 rx_commands.py --pixhawk /dev/ttyACM0 --radio /dev/ttyUSB0
    python3 rx_commands.py --skip-preflight  # Skip preflight checks (bench test)
    
    FOR SCOUTING WITH HUMAN DETECTION, USE:
    python3 main.py
"""

import sys
import os
import time
import argparse
import serial
import threading
from datetime import datetime
import math  # For coordinate calculations

# Add nidar/ to path for importing modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NIDAR_DIR = os.path.join(SCRIPT_DIR, 'nidar')
sys.path.insert(0, NIDAR_DIR)

# Default KML survey configuration for SCOUT command
DEFAULT_KML_FILE = "survey_area.kml"  # Filename in /home/dart/quadtest/missions/
DEFAULT_SCOUT_ALTITUDE = 5.0  # meters AGL
DEFAULT_SCOUT_PATTERN = "curved"  # "curved" or "lawnmower"

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
except ImportError:
    print("ERROR: DroneKit not installed. Run: pip install dronekit")
    sys.exit(1)

# Import nidar modules
try:
    from utils.logger import MissionLogger
    from utils.connection import VehicleConnection
    from safety.preflight import PreflightChecks
    from safety.abort import SafetyAbort
    NIDAR_AVAILABLE = True
    print("[OK] nidar/ modules loaded (PreflightChecks, SafetyAbort, MissionLogger)")
except ImportError as e:
    print(f"[WARN] Could not import nidar modules: {e}")
    print("[WARN] Running in standalone mode without safety features")
    NIDAR_AVAILABLE = False


class DroneCommandReceiver:
    """Receives and executes drone commands from LoRa/3DR radio."""
    
    def __init__(self, pixhawk_port, pixhawk_baud, radio_port, radio_baud, skip_preflight=False):
        self.pixhawk_port = pixhawk_port
        self.pixhawk_baud = pixhawk_baud
        self.radio_port = radio_port
        self.radio_baud = radio_baud
        self.skip_preflight = skip_preflight
        self.vehicle = None
        self.radio = None
        self.running = False
        
        # Safety modules (initialized after connection)
        self.preflight = None
        self.safety_abort = None
        
        # Telemetry streaming thread
        self.telemetry_thread = None
        self.telemetry_interval = 2.0  # Send telemetry every 2 seconds
        
        # Telemetry logging
        self.telemetry_log_file = None
        self.setup_telemetry_logging()
    
    def setup_telemetry_logging(self):
        """Setup telemetry log file."""
        try:
            # Create logs directory if it doesn't exist
            log_dir = "/home/dart/quadtest/logs"
            os.makedirs(log_dir, exist_ok=True)
            
            # Create timestamped log file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(log_dir, f"rx_telemetry_{timestamp}.log")
            self.telemetry_log_file = open(log_path, 'w')
            self.telemetry_log_file.write(f"RX Telemetry Log - Started at {datetime.now()}\n")
            self.telemetry_log_file.write("=" * 80 + "\n")
            self.telemetry_log_file.write("Timestamp | Mode | Armed | Alt | Voltage | Current | % | GPS Fix | Sats | HDOP | GyroX | GyroY | GyroZ | VibeX | VibeY | VibeZ | RSSI\n")
            self.telemetry_log_file.write("=" * 80 + "\n")
            self.telemetry_log_file.flush()
            print(f"[INFO] Logging telemetry to: {log_path}")
        except Exception as e:
            print(f"[WARN] Could not setup telemetry logging: {e}")
            self.telemetry_log_file = None
    
    def log_telemetry_to_file(self, mode, armed, alt, voltage, current, level, gps_fix, sats, hdop, gx, gy, gz, vx, vy, vz, rssi):
        """Log telemetry data to CSV-style file."""
        if self.telemetry_log_file:
            try:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                line = f"{timestamp} | {mode} | {armed} | {alt:.1f} | {voltage:.2f} | {current:.1f} | {level} | {gps_fix} | {sats} | {hdop:.1f} | {gx:.2f} | {gy:.2f} | {gz:.2f} | {vx:.2f} | {vy:.2f} | {vz:.2f} | {rssi}\n"
                self.telemetry_log_file.write(line)
                self.telemetry_log_file.flush()
            except Exception as e:
                pass  # Silently ignore logging errors
        
    def log(self, level, msg):
        """Log message using MissionLogger if available, else print."""
        if NIDAR_AVAILABLE:
            if level == "info":
                MissionLogger.info(msg)
            elif level == "success":
                MissionLogger.success(msg)
            elif level == "warning":
                MissionLogger.warning(msg)
            elif level == "error":
                MissionLogger.error(msg)
            elif level == "state":
                MissionLogger.state(msg)
        else:
            print(f"[{level.upper()}] {msg}")
        
    def connect_pixhawk(self):
        """Connect to Pixhawk."""
        self.log("info", f"Connecting to Pixhawk on {self.pixhawk_port}...")
        try:
            self.vehicle = connect(self.pixhawk_port, baud=self.pixhawk_baud, 
                                   wait_ready=False, timeout=60)
            time.sleep(2)
            self.log("success", f"Connected to Pixhawk")
            self.log("info", f"Firmware: {self.vehicle.version}")
            self.log("info", f"Mode: {self.vehicle.mode.name}")
            self.log("info", f"Armed: {self.vehicle.armed}")
            
            # Initialize safety modules
            if NIDAR_AVAILABLE:
                self.preflight = PreflightChecks(self.vehicle)
                self.safety_abort = SafetyAbort(self.vehicle)
                self.log("success", "Safety modules initialized")
            
            return True
        except Exception as e:
            self.log("error", f"Failed to connect to Pixhawk: {e}")
            return False
    
    def connect_radio(self):
        """Connect to 3DR radio for receiving commands."""
        self.log("info", f"Opening radio on {self.radio_port} @ {self.radio_baud}...")
        try:
            self.radio = serial.Serial(
                port=self.radio_port,
                baudrate=self.radio_baud,
                timeout=0.1
            )
            self.log("success", "Radio connected")
            return True
        except Exception as e:
            self.log("error", f"Failed to open radio: {e}")
            return False
    
    def send_response(self, msg):
        """Send response back via radio."""
        try:
            response = f"[DRONE] {msg}\n"
            self.radio.write(response.encode())
            self.log("info", f"[TX] {msg}")
        except Exception as e:
            self.log("error", f"Failed to send response: {e}")
    
    def send_telemetry(self, msg):
        """Send telemetry data via radio (tagged with [TELEM])."""
        try:
            response = f"[TELEM][INFO] {msg}\n"
            self.radio.write(response.encode())
        except Exception as e:
            pass  # Don't spam logs with telemetry send errors
    
    def telemetry_loop(self):
        """Background thread that continuously sends telemetry data."""
        while self.running:
            try:
                if self.vehicle and self.radio:
                    # Get all telemetry values
                    mode = self.vehicle.mode.name
                    armed = "ARM" if self.vehicle.armed else "DISARM"
                    
                    # Altitude
                    alt = self.vehicle.location.global_relative_frame.alt or 0
                    
                    # Battery
                    bat = self.vehicle.battery
                    voltage = bat.voltage if (bat and bat.voltage) else 0
                    current = bat.current if (bat and bat.current) else 0
                    level = bat.level if (bat and bat.level) else 0
                    
                    # GPS
                    gps = self.vehicle.gps_0
                    gps_fix = gps.fix_type if gps else 0
                    satellites = gps.satellites_visible if gps else 0
                    hdop = gps.eph if gps else 9999  # HDOP (horizontal dilution of precision)
                    
                    # IMU - Gyro
                    # Access raw IMU data from vehicle parameters
                    try:
                        # DroneKit doesn't directly expose gyro, so we use attitude velocities
                        gyro_x = self.vehicle.attitude.roll if hasattr(self.vehicle.attitude, 'roll') else 0
                        gyro_y = self.vehicle.attitude.pitch if hasattr(self.vehicle.attitude, 'pitch') else 0
                        gyro_z = self.vehicle.attitude.yaw if hasattr(self.vehicle.attitude, 'yaw') else 0
                    except:
                        gyro_x = gyro_y = gyro_z = 0
                    
                    # Vibration - Access via vehicle.parameters or raw_imu
                    try:
                        # Try to get vibration data from vehicle
                        vibe_x = self.vehicle.vibration.vibration_x if hasattr(self.vehicle, 'vibration') else 0
                        vibe_y = self.vehicle.vibration.vibration_y if hasattr(self.vehicle, 'vibration') else 0
                        vibe_z = self.vehicle.vibration.vibration_z if hasattr(self.vehicle, 'vibration') else 0
                    except:
                        vibe_x = vibe_y = vibe_z = 0
                    
                    # RSSI - Radio Signal Strength
                    try:
                        rssi = self.vehicle.parameters.get('RSSI', 0)
                    except:
                        rssi = 0
                    
                    # Log to file
                    self.log_telemetry_to_file(
                        mode, armed, alt, voltage, current, level,
                        gps_fix, satellites, hdop,
                        gyro_x, gyro_y, gyro_z,
                        vibe_x, vibe_y, vibe_z,
                        rssi
                    )
                    
                    # Send compact telemetry message
                    telem_msg = (
                        f"Mode:{mode} {armed} | "
                        f"Alt:{alt:.1f}m | "
                        f"Bat:{voltage:.2f}V {current:.1f}A {level}% | "
                        f"GPS:Fix{gps_fix} Sat{satellites} HDOP:{hdop:.1f} | "
                        f"Gyro:X{gyro_x:.2f} Y{gyro_y:.2f} Z{gyro_z:.2f} | "
                        f"Vibe:X{vibe_x:.2f} Y{vibe_y:.2f} Z{vibe_z:.2f} | "
                        f"RSSI:{rssi}"
                    )
                    self.send_telemetry(telem_msg)
                    
            except Exception as e:
                pass  # Don't spam errors in telemetry thread
            
            time.sleep(self.telemetry_interval)

    
    def execute_command(self, cmd):
        """Parse and execute a command."""
        cmd = cmd.strip().upper()
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log("state", f"[RX] Command: {cmd}")
        
        if not self.vehicle:
            self.send_response("ERROR: No vehicle connected")
            return
        
        try:
            # ARM command (with preflight checks)
            if cmd == "ARM":
                self.cmd_arm()
            
            # DISARM command
            elif cmd == "DISARM":
                self.cmd_disarm()
            
            # TAKEOFF command (TAKEOFF:altitude)
            elif cmd.startswith("TAKEOFF"):
                if ":" in cmd:
                    alt = float(cmd.split(":")[1])
                else:
                    alt = 3.0  # Default 3 meters
                self.cmd_takeoff(alt)
            
            # LAND command (uses SafetyAbort)
            elif cmd == "LAND":
                self.cmd_land()
            
            # RTL (Return to Launch - uses SafetyAbort)
            elif cmd == "RTL":
                self.cmd_rtl()
            
            # MODE command (MODE:STABILIZE)
            elif cmd.startswith("MODE:"):
                mode = cmd.split(":")[1]
                self.cmd_set_mode(mode)
            
            # GOTO command (GOTO:lat,lon,alt)
            elif cmd.startswith("GOTO:"):
                params = cmd.split(":")[1].split(",")
                lat = float(params[0])
                lon = float(params[1])
                alt = float(params[2]) if len(params) > 2 else 10.0
                self.cmd_goto(lat, lon, alt)
            
            # MOVE command (MOVE:north,east,down)
            elif cmd.startswith("MOVE:"):
                params = cmd.split(":")[1].split(",")
                north = float(params[0])
                east = float(params[1])
                down = float(params[2]) if len(params) > 2 else 0.0
                self.cmd_move(north, east, down)
            
            # STOP/BRAKE command
            elif cmd == "STOP" or cmd == "BRAKE":
                self.cmd_stop()
            
            # STATUS command
            elif cmd == "STATUS":
                self.cmd_status()
            
            # PREFLIGHT command - run preflight checks only
            elif cmd == "PREFLIGHT":
                self.cmd_preflight()
            
            # ABORT command - emergency abort using SafetyAbort
            elif cmd == "ABORT":
                self.cmd_abort()
            
            # SCOUT command - use default KML configuration
            elif cmd == "SCOUT":
                self.cmd_kml_survey(DEFAULT_KML_FILE, DEFAULT_SCOUT_ALTITUDE)
            
            # KML SURVEY command (KML:SURVEY:filename,altitude)
            elif cmd.startswith("KML:SURVEY:"):
                params = cmd.split(":")[2].split(",")
                kml_file = params[0]
                altitude = float(params[1]) if len(params) > 1 else 5.0
                self.cmd_kml_survey(kml_file, altitude)
            
            # PING command
            elif cmd == "PING":
                self.send_response("PONG")
            
            # Unknown command
            else:
                self.send_response(f"ERROR: Unknown command '{cmd}'")
                
        except Exception as e:
            self.send_response(f"ERROR: {str(e)}")
    
    def cmd_arm(self):
        """Arm the drone with preflight checks."""
        if self.vehicle.armed:
            self.send_response("Already armed")
            return
        
        # Run preflight checks if available and not skipped
        if NIDAR_AVAILABLE and self.preflight and not self.skip_preflight:
            self.log("info", "Running preflight checks before arming...")
            self.send_response("Running preflight checks...")
            
            # Check GPS (optional - warn but continue)
            gps = self.vehicle.gps_0
            if gps and gps.fix_type is not None and gps.fix_type < 3:
                self.log("warning", f"GPS fix type {gps.fix_type} (need 3 for 3D)")
                self.send_response(f"WARN: GPS fix={gps.fix_type} (need 3)")
            
            # Check EKF
            if not self.vehicle.ekf_ok:
                self.log("warning", "EKF not converged")
                self.send_response("WARN: EKF not converged")
            
            self.send_response("Preflight OK, arming...")
        
        # Set mode - use STABILIZE if no GPS, else GUIDED
        gps = self.vehicle.gps_0
        if gps and gps.fix_type is not None and gps.fix_type >= 3:
            self.vehicle.mode = VehicleMode("GUIDED")
        else:
            self.vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(1)
        
        self.vehicle.armed = True
        
        # Wait for arming
        timeout = 10
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                self.send_response("ARM FAILED: Timeout")
                return
            time.sleep(0.5)
        
        self.log("success", "Vehicle ARMED")
        self.send_response(f"ARMED OK (mode={self.vehicle.mode.name})")
    
    def cmd_disarm(self):
        """Disarm the drone."""
        if not self.vehicle.armed:
            self.send_response("Already disarmed")
            return
        
        self.vehicle.armed = False
        
        timeout = 5
        start = time.time()
        while self.vehicle.armed:
            if time.time() - start > timeout:
                self.send_response("DISARM FAILED: Timeout")
                return
            time.sleep(0.5)
        
        self.log("success", "Vehicle DISARMED")
        self.send_response("DISARMED OK")
    
    def cmd_takeoff(self, altitude):
        """Takeoff to specified altitude."""
        if not self.vehicle.armed:
            self.send_response("ERROR: Not armed")
            return
        
        # Must be in GUIDED mode for takeoff
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        
        self.log("state", f"Taking off to {altitude}m")
        self.send_response(f"TAKEOFF to {altitude}m...")
        self.vehicle.simple_takeoff(altitude)
        
        # Wait to reach altitude (with timeout)
        timeout = 30
        start = time.time()
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            if current_alt >= altitude * 0.95:
                self.log("success", f"Reached altitude: {current_alt:.1f}m")
                self.send_response(f"TAKEOFF OK: {current_alt:.1f}m")
                break
            if time.time() - start > timeout:
                self.send_response(f"TAKEOFF: At {current_alt:.1f}m (timeout)")
                break
            time.sleep(1)
    
    def cmd_land(self):
        """Land the drone using SafetyAbort if available."""
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.emergency_land()
            self.send_response("LANDING (SafetyAbort)...")
        else:
            self.vehicle.mode = VehicleMode("LAND")
            self.send_response("LANDING...")
    
    def cmd_rtl(self):
        """Return to launch using SafetyAbort if available."""
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.return_to_launch()
            self.send_response("RTL (SafetyAbort): Returning to launch")
        else:
            self.vehicle.mode = VehicleMode("RTL")
            self.send_response("RTL: Returning to launch")
    
    def cmd_abort(self):
        """Emergency abort - immediate land."""
        self.log("warning", "EMERGENCY ABORT TRIGGERED")
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.emergency_land()
            self.send_response("ABORT: Emergency landing!")
        else:
            self.vehicle.mode = VehicleMode("LAND")
            self.send_response("ABORT: Emergency landing!")
    
    def cmd_preflight(self):
        """Run preflight checks only."""
        if not NIDAR_AVAILABLE or not self.preflight:
            self.send_response("ERROR: Preflight module not available")
            return
        
        self.send_response("Running preflight checks...")
        
        # Battery check
        bat_ok = self.preflight.check_battery(min_voltage=10.5)
        bat_v = self.vehicle.battery.voltage if self.vehicle.battery else 0
        
        # GPS check
        gps = self.vehicle.gps_0
        gps_fix = gps.fix_type if gps else 0
        gps_sats = gps.satellites_visible if gps else 0
        
        # EKF check
        ekf_ok = self.vehicle.ekf_ok
        
        # Armable check
        armable = self.vehicle.is_armable
        
        result = f"PREFLIGHT: BAT={bat_v:.1f}V({'OK' if bat_ok else 'LOW'}), GPS={gps_fix}/sats={gps_sats}, EKF={'OK' if ekf_ok else 'NO'}, ARMABLE={'YES' if armable else 'NO'}"
        self.send_response(result)
    
    def cmd_set_mode(self, mode):
        """Change flight mode."""
        try:
            self.vehicle.mode = VehicleMode(mode)
            time.sleep(1)
            if self.vehicle.mode.name == mode:
                self.send_response(f"MODE: {mode} OK")
            else:
                self.send_response(f"MODE: Failed, current={self.vehicle.mode.name}")
        except Exception as e:
            self.send_response(f"MODE ERROR: {e}")
    
    def cmd_goto(self, lat, lon, alt):
        """Go to GPS location."""
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        
        location = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(location)
        self.send_response(f"GOTO: {lat:.6f},{lon:.6f},{alt:.1f}m")
    
    def cmd_move(self, north, east, down):
        """Move relative to current position."""
        from dronekit import LocationGlobal
        import math
        
        # Get current location
        current = self.vehicle.location.global_relative_frame
        
        # Calculate new position (approximate)
        earth_radius = 6378137.0
        d_lat = north / earth_radius
        d_lon = east / (earth_radius * math.cos(math.pi * current.lat / 180))
        
        new_lat = current.lat + (d_lat * 180 / math.pi)
        new_lon = current.lon + (d_lon * 180 / math.pi)
        new_alt = current.alt - down  # down is positive for descending
        
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        
        location = LocationGlobalRelative(new_lat, new_lon, new_alt)
        self.vehicle.simple_goto(location)
        self.send_response(f"MOVE: N={north}m E={east}m D={down}m")
    
    def cmd_stop(self):
        """Stop and hover."""
        # Try BRAKE mode first, fall back to LOITER
        try:
            self.vehicle.mode = VehicleMode("BRAKE")
        except:
            self.vehicle.mode = VehicleMode("LOITER")
        self.send_response(f"STOP: Mode={self.vehicle.mode.name}")
    
    def cmd_status(self):
        """Send current status with enhanced telemetry."""
        mode = self.vehicle.mode.name
        armed = "ARM" if self.vehicle.armed else "DISARM"
        alt = self.vehicle.location.global_relative_frame.alt or 0
        
        # Battery
        bat = self.vehicle.battery
        voltage = bat.voltage if (bat and bat.voltage) else 0
        current = bat.current if (bat and bat.current) else 0
        level = bat.level if (bat and bat.level) else 0
        
        # GPS
        gps = self.vehicle.gps_0
        gps_fix = gps.fix_type if gps else 0
        satellites = gps.satellites_visible if gps else 0
        hdop = gps.eph if gps else 9999
        
        # IMU/Gyro (using attitude as proxy)
        try:
            gyro_x = self.vehicle.attitude.roll if hasattr(self.vehicle.attitude, 'roll') else 0
            gyro_y = self.vehicle.attitude.pitch if hasattr(self.vehicle.attitude, 'pitch') else 0
            gyro_z = self.vehicle.attitude.yaw if hasattr(self.vehicle.attitude, 'yaw') else 0
        except:
            gyro_x = gyro_y = gyro_z = 0
        
        # Vibration
        try:
            vibe_x = self.vehicle.vibration.vibration_x if hasattr(self.vehicle, 'vibration') else 0
            vibe_y = self.vehicle.vibration.vibration_y if hasattr(self.vehicle, 'vibration') else 0
            vibe_z = self.vehicle.vibration.vibration_z if hasattr(self.vehicle, 'vibration') else 0
        except:
            vibe_x = vibe_y = vibe_z = 0
        
        # RSSI
        try:
            rssi = self.vehicle.parameters.get('RSSI', 0)
        except:
            rssi = 0
        
        status = (
            f"STATUS: {mode},{armed},ALT={alt:.1f}m | "
            f"BAT={voltage:.2f}V,{current:.1f}A,{level}% | "
            f"GPS=Fix{gps_fix},Sat{satellites},HDOP{hdop:.1f} | "
            f"GYRO=X{gyro_x:.2f},Y{gyro_y:.2f},Z{gyro_z:.2f} | "
            f"VIBE=X{vibe_x:.2f},Y{vibe_y:.2f},Z{vibe_z:.2f} | "
            f"RSSI={rssi}"
        )
        self.send_response(status)
    
    def cmd_kml_survey(self, kml_filename, altitude):
        """
        Execute KML-based area survey mission for human detection.
        
        Args:
            kml_filename: Name of KML file in /home/dart/quadtest/missions/
            altitude: Flight altitude in meters AGL
        """
        if not NIDAR_AVAILABLE:
            self.send_response("ERROR: KML survey requires nidar modules")
            return
        
        self.log("state", f"Starting KML survey: {kml_filename} at {altitude}m")
        self.send_response(f"Loading KML survey: {kml_filename}")
        
        try:
            # Import KML loader
            from mission.kml_loader import load_waypoints_from_kml, validate_kml_mission
            from mission.waypoint_mission import WaypointMission
            
            # Build KML file path
            kml_path = os.path.join("/home/dart/quadtest/missions", kml_filename)
            if not os.path.exists(kml_path):
                self.send_response(f"ERROR: KML file not found: {kml_path}")
                return
            
            # Load waypoints from KML
            self.log("info", f"Loading waypoints from {kml_path}...")
            waypoints = load_waypoints_from_kml(
                kml_file=kml_path,
                altitude_meters=altitude,
                pattern="curved",  # Use curved center coverage pattern
                camera_fov=57,     # Wide-angle camera FOV
                overlap=0.25       # 25% overlap
            )
            
            self.send_response(f"Generated {len(waypoints)} waypoints")
            
            # Validate mission
            valid, msg = validate_kml_mission(waypoints)
            if not valid:
                self.send_response(f"ERROR: {msg}")
                return
            
            self.log("success", f"Mission validated: {len(waypoints)} waypoints")
            
            # Upload mission to vehicle
            mission = WaypointMission(self.vehicle)
            if not mission.upload_mission(waypoints):
                self.send_response("ERROR: Mission upload failed")
                return
            
            self.send_response(f"Mission uploaded: {len(waypoints)} waypoints")
            self.log("success", "KML survey mission ready - use ARM/TAKEOFF to start")
            self.send_response("Ready: ARM -> TAKEOFF -> MODE:AUTO to begin survey")
            
        except Exception as e:
            error_msg = f"KML survey error: {str(e)}"
            self.log("error", error_msg)
            self.send_response(f"ERROR: {error_msg}")
    
    def listen_loop(self):
        """Main loop to listen for commands."""
        if NIDAR_AVAILABLE:
            MissionLogger.header("LISTENING FOR COMMANDS")
        else:
            print("\n" + "=" * 50)
            print("  LISTENING FOR COMMANDS")
            print("=" * 50)
        
        self.log("info", "Waiting for commands from remote laptop...")
        self.log("info", "Commands: ARM, DISARM, TAKEOFF:5, LAND, RTL, ABORT, SCOUT, PREFLIGHT, STATUS")
        
        
        buffer = ""
        
        while self.running:
            try:
                # Read from radio
                if self.radio.in_waiting > 0:
                    data = self.radio.read(self.radio.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete commands (newline terminated)
                    while '\n' in buffer:
                        cmd, buffer = buffer.split('\n', 1)
                        cmd = cmd.strip()
                        if cmd:
                            self.execute_command(cmd)
                
                time.sleep(0.05)  # Small delay to prevent CPU hogging
                
            except KeyboardInterrupt:
                self.log("info", "Stopping...")
                break
            except Exception as e:
                self.log("error", f"Listen error: {e}")
                time.sleep(1)
    
    def run(self):
        """Start the command receiver."""
        # Connect to Pixhawk
        if not self.connect_pixhawk():
            return False
        
        # Connect to radio
        if not self.connect_radio():
            return False
        
        self.running = True
        
        # Start telemetry streaming thread
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        self.log("success", "Telemetry streaming started (2s interval)")
        
        # Show integration status
        if NIDAR_AVAILABLE:
            self.log("success", "Integrated with nidar/ modules:")
            self.log("info", "  - PreflightChecks: Battery, GPS, EKF checks")
            self.log("info", "  - SafetyAbort: Emergency land, RTL")
            self.log("info", "  - MissionLogger: Colored logging")
        else:
            self.log("warning", "Running standalone (no nidar/ integration)")
        
        if self.skip_preflight:
            self.log("warning", "PREFLIGHT CHECKS DISABLED (--skip-preflight)")
        
        try:
            self.listen_loop()
        finally:
            self.running = False
            if self.vehicle:
                self.vehicle.close()
            if self.radio:
                self.radio.close()
            if self.telemetry_log_file:
                self.telemetry_log_file.write(f"\n\nLog ended at {datetime.now()}\n")
                self.telemetry_log_file.close()
            self.log("info", "Connections closed")
        
        return True


def main():
    parser = argparse.ArgumentParser(description='Drone Command Receiver via LoRa/3DR Radio')
    parser.add_argument('--pixhawk', default='/dev/ttyACM0',
                       help='Pixhawk port (default: /dev/ttyACM0)')
    parser.add_argument('--pixhawk-baud', type=int, default=115200,
                       help='Pixhawk baud rate (default: 115200)')
    parser.add_argument('--radio', default='/dev/ttyUSB0',
                       help='3DR Radio port (default: /dev/ttyUSB0)')
    parser.add_argument('--radio-baud', type=int, default=57600,
                       help='3DR Radio baud rate (default: 57600)')
    parser.add_argument('--skip-preflight', action='store_true',
                       help='Skip preflight checks (for bench testing)')
    args = parser.parse_args()
    
    if NIDAR_AVAILABLE:
        MissionLogger.header("DRONE COMMAND RECEIVER")
        MissionLogger.info("Integrated with nidar/ safety modules")
    else:
        print("=" * 50)
        print("  DRONE COMMAND RECEIVER")
        print("  (Standalone mode - no nidar/ integration)")
        print("=" * 50)
    
    receiver = DroneCommandReceiver(
        pixhawk_port=args.pixhawk,
        pixhawk_baud=args.pixhawk_baud,
        radio_port=args.radio,
        radio_baud=args.radio_baud,
        skip_preflight=args.skip_preflight
    )
    
    success = receiver.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
