#!/usr/bin/env python3
"""
Main Autonomous Drone Controller with LoRa Command Interface and Human Detection

This is the main entry point that:
1. Listens for commands from ground station via LoRa/3DR radio
2. Executes preflight checks
3. Runs waypoint missions
4. Supports KML file import for mission planning
5. Performs real-time human detection using YOLOv8 ONNX model

Commands received from ground station (tx_commands.py):
    PING              - Test connection
    STATUS            - Get drone status
    PREFLIGHT         - Run preflight checks
    ARM               - Arm the drone
    FORCEARM          - Force arm (bypass pre-arm checks)
    DISARM            - Disarm the drone
    TAKEOFF:5         - Takeoff to 5 meters
    LAND              - Land immediately
    RTL               - Return to launch
    ABORT             - Emergency abort
    MISSION:START     - Start the autonomous mission
    MISSION:STOP      - Stop current mission
    MODE:xxx          - Change flight mode
    LOAD:filename     - Load KML mission file
    
    Human Detection  Commands:
    DETECT:START      - Start human detection camera
    DETECT:STOP       - Stop human detection
    DETECT:STATUS     - Get current detection status
    DETECT:CONF:0.7   - Set confidence threshold (0.1-1.0)
    
    Scout Commands (auto-starts detection + recording):
    SCOUT             - Start KML area survey mission (loads default KML file)
    SCOUT:KML:file,alt - Custom KML survey (e.g. SCOUT:KML:park.kml,20)
    SCOUT:STOP        - Stop scouting, detection, and save recording

Usage:
    python3 main.py
    python3 main.py --pixhawk /dev/ttyACM0 --radio /dev/ttyUSB0
"""

import sys
import os
import time
import argparse
import serial
import threading
from datetime import datetime
from collections import deque
import numpy as np

# Add current directory to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NIDAR_DIR = os.path.join(SCRIPT_DIR, 'nidar')
sys.path.insert(0, NIDAR_DIR)

# Try to import human detection modules
try:
    import cv2
    from yolo_detector import YOLODetector
    DETECTION_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] Human detection not available: {e}")
    DETECTION_AVAILABLE = False

# Try to import pyrealsense2 for color camera
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

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
    from mission.waypoint_mission import WaypointMission
    from mission.mission_data import MissionData
    from utils.telemetry_logger import (
        RadioTelemetryLogger,
        RadioStatusMonitor,
        create_radio_telemetry_system
    )
    NIDAR_AVAILABLE = True
    TELEMETRY_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] Could not import nidar modules: {e}")
    NIDAR_AVAILABLE = False
    TELEMETRY_AVAILABLE = False

# Import session file logger for persistent log storage
try:
    from utils.file_logger import SessionFileLogger, init_session_logging, get_session_logger
    FILE_LOGGING_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] File logging not available: {e}")
    FILE_LOGGING_AVAILABLE = False

# Global constants - change these values to modify drone behavior
SCOUT_ALTITUDE = 5.0  # Default altitude for all scouting missions (meters AGL)


class MainController:
    """
    Main autonomous drone controller.
    Listens for LoRa commands and executes missions.
    """
    
    def __init__(self, pixhawk_port, pixhawk_baud, radio_port, radio_baud, require_radio=True):
        self.pixhawk_port = pixhawk_port
        self.pixhawk_baud = pixhawk_baud
        self.radio_port = radio_port
        self.radio_baud = radio_baud
        self.require_radio = require_radio
        
        self.vehicle = None
        self.radio = None
        self.running = False
        
        # Mission state
        self.mission_running = False
        self.mission_thread = None
        self.abort_flag = False
        self.abort_lock = threading.Lock()
        
        # Safety modules
        self.preflight = None
        self.safety_abort = None
        self.mission = None
        
        # Human detection state
        self.detector = None
        self.camera = None
        self.rs_pipeline = None  # RealSense pipeline
        self.use_realsense = False
        self.detection_running = False
        self.detection_thread = None
        self.latest_detections = []  # List of (boxes, scores, timestamp)
        self.detection_lock = threading.Lock()
        self.frame_queue = deque(maxlen=1)
        self.detection_count = 0
        self.last_detection_time = 0
        
        # Detection settings
        self.detection_confidence = 0.6
        self.detection_skip_frames = 3
        self.model_path = os.path.join(SCRIPT_DIR, 'best.onnx')
        
        # Dynamic waypoints (sent from ground station)
        self.waypoints = []  # List of (lat, lon, alt) tuples
        self.scout_altitude = SCOUT_ALTITUDE  # Default scout altitude
        
        # Default KML survey configuration for SCOUT command
        self.default_kml_file = "survey_area.kml"  # Filename in /home/dart/quadtest/missions/
        self.default_kml_altitude = SCOUT_ALTITUDE  # meters AGL
        self.default_kml_pattern = "curved"  # "curved" or "lawnmower"
        
        # Radio telemetry system (sends logs to tx_commands.py)
        self.telem_logger = None
        self.status_monitor = None
        self.telemetry_enabled = True
        
        # Video recording
        self.video_writer = None
        self.recording = False
        self.recording_path = None
        self.recording_start_time = 0
    
    def log(self, level, msg):
        """Log with MissionLogger if available."""
        timestamp = datetime.now().strftime("%H:%M:%S")
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
            elif level == "header":
                MissionLogger.header(msg)
        else:
            print(f"[{timestamp}] [{level.upper()}] {msg}")
    
    def connect_pixhawk(self):
        """Connect to Pixhawk."""
        self.log("info", f"Connecting to Pixhawk on {self.pixhawk_port}...")
        try:
            self.vehicle = connect(self.pixhawk_port, baud=self.pixhawk_baud,
                                   wait_ready=False, timeout=60)
            time.sleep(2)
            self.log("success", "Connected to Pixhawk")
            self.log("info", f"Firmware: {self.vehicle.version}")
            self.log("info", f"Mode: {self.vehicle.mode.name}")
            self.log("info", f"Armed: {self.vehicle.armed}")
            
            # Initialize safety modules
            if NIDAR_AVAILABLE:
                self.preflight = PreflightChecks(self.vehicle)
                self.safety_abort = SafetyAbort(self.vehicle)
                self.mission = WaypointMission(self.vehicle)
                self.log("success", "Mission modules initialized")
            
            # Note: Telemetry is set up AFTER radio connects (in run())
            
            return True
        except Exception as e:
            self.log("error", f"Failed to connect to Pixhawk: {e}")
            return False
    
    def _setup_telemetry(self):
        """Initialize radio telemetry logging to ground station (tx_commands.py)."""
        if not self.radio:
            self.log("warning", "Radio not connected - cannot setup telemetry")
            self.telemetry_enabled = False
            return
            
        self.log("info", "Setting up radio telemetry...")
        try:
            # Create radio telemetry components
            self.telem_logger, self.status_monitor = create_radio_telemetry_system(
                self.vehicle, 
                self.radio
            )
            
            # Start telemetry
            self.telem_logger.start()
            
            # Connect MissionLogger to telemetry (console logs also go to radio)
            MissionLogger.set_telemetry_logger(self.telem_logger)
            
            # Connect SessionFileLogger to telemetry (stderr logs also go to radio)
            if FILE_LOGGING_AVAILABLE:
                SessionFileLogger.set_radio_telemetry(self.telem_logger)
            
            # Start continuous status monitor (sends updates every 5 seconds)
            self.status_monitor.start(interval=5)
            
            # Send initial status
            self.telem_logger.info("=== TELEMETRY ACTIVE ===")
            self.status_monitor.send_status()
            
            self.log("success", "Radio telemetry active")
            
        except Exception as e:
            self.log("warning", f"Telemetry init failed: {e}")
            self.telemetry_enabled = False
    
    def _stop_telemetry(self):
        """Stop telemetry system."""
        if self.status_monitor:
            self.status_monitor.stop()
        if self.telem_logger:
            self.telem_logger.info("Telemetry stopping...")
            self.telem_logger.stop()
        MissionLogger.set_telemetry_logger(None)
        
    def connect_radio(self):
        """Connect to LoRa/3DR radio."""
        self.log("info", f"Opening radio on {self.radio_port} @ {self.radio_baud}...")
        try:
            self.radio = serial.Serial(
                port=self.radio_port,
                baudrate=self.radio_baud,
                timeout=0.1
            )
            self.log("success", "Radio connected - listening for commands")
            return True
        except Exception as e:
            self.log("error", f"Failed to open radio: {e}")
            return False
    
    def send_response(self, msg):
        """Send response to ground station with timestamp."""
        try:
            # Add timestamp to message for sync verification
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
            response = f"[{timestamp}] [DRONE] {msg}\n"
            self.radio.write(response.encode())
            self.radio.flush()  # Ensure data is sent immediately
            self.log("info", f"[TX] {msg}")
        except Exception as e:
            self.log("error", f"Failed to send: {e}")
    
    def execute_command(self, cmd):
        """Parse and execute received command."""
        cmd = cmd.strip().upper()
        self.log("state", f"[RX] Command: {cmd}")
        
        if not self.vehicle:
            self.send_response("ERROR: No vehicle connected")
            return
        
        try:
            # Connection test
            if cmd == "PING":
                self.send_response("PONG")
            
            # Status
            elif cmd == "STATUS":
                self.cmd_status()
            
            # Preflight checks
            elif cmd == "PREFLIGHT":
                self.cmd_preflight()
            
            # Arm
            elif cmd == "ARM":
                self.cmd_arm()
            
            # Force Arm (bypass pre-arm checks)
            elif cmd == "FORCEARM":
                self.cmd_force_arm()
            
            # Disarm
            elif cmd == "DISARM":
                self.cmd_disarm()
            
            # Takeoff
            elif cmd.startswith("TAKEOFF"):
                alt = float(cmd.split(":")[1]) if ":" in cmd else 5.0
                self.cmd_takeoff(alt)
            
            # Land
            elif cmd == "LAND":
                self.cmd_land()
            
            # RTL
            elif cmd == "RTL":
                self.cmd_rtl()
            
            # Abort
            elif cmd == "ABORT":
                self.cmd_abort()
            
            # Start mission
            elif cmd == "MISSION:START" or cmd == "START":
                self.cmd_mission_start()
            
            # Stop mission
            elif cmd == "MISSION:STOP" or cmd == "STOP":
                self.cmd_mission_stop()
            
            # Mode change
            elif cmd.startswith("MODE:"):
                mode = cmd.split(":")[1]
                self.cmd_set_mode(mode)
            
            # Load KML file
            elif cmd.startswith("LOAD:"):
                filename = cmd.split(":")[1]
                self.cmd_load_kml(filename)
            
            # GOTO
            elif cmd.startswith("GOTO:"):
                params = cmd.split(":")[1].split(",")
                lat, lon = float(params[0]), float(params[1])
                alt = float(params[2]) if len(params) > 2 else 10.0
                self.cmd_goto(lat, lon, alt)
            
            # Human Detection Commands
            elif cmd == "DETECT:START" or cmd == "DETECTION:START":
                self.cmd_detection_start()
            
            elif cmd == "DETECT:STOP" or cmd == "DETECTION:STOP":
                self.cmd_detection_stop()
            
            elif cmd == "DETECT:STATUS" or cmd == "DETECTION:STATUS":
                self.cmd_detection_status()
            
            elif cmd.startswith("DETECT:CONF:"):
                conf = float(cmd.split(":")[2])
                self.cmd_detection_set_confidence(conf)
            
            # Waypoint Commands (for scouting)
            elif cmd.startswith("WP:"):
                # WP:lat,lon,alt - Add waypoint
                params = cmd.split(":")[1].split(",")
                lat, lon = float(params[0]), float(params[1])
                alt = float(params[2]) if len(params) > 2 else self.scout_altitude
                self.cmd_add_waypoint(lat, lon, alt)
            
            elif cmd == "WP:CLEAR" or cmd == "CLEARWP":
                self.cmd_clear_waypoints()
            
            elif cmd == "WP:LIST" or cmd == "LISTWP":
                self.cmd_list_waypoints()
            
            elif cmd.startswith("ALT:"):
                # ALT:15 - Set scout altitude
                alt = float(cmd.split(":")[1])
                self.scout_altitude = alt
                self.send_response(f"Scout altitude set to {alt}m")
            
            elif cmd == "SCOUT":
                # SCOUT - Load default KML and start survey
                self.cmd_scout_kml(self.default_kml_file, self.default_kml_altitude)
            
            elif cmd.startswith("SCOUT:KML:"):
                # SCOUT:KML:filename,altitude
                params = cmd.split(":")[2].split(",")
                kml_file = params[0]
                altitude = float(params[1]) if len(params) > 1 else self.default_kml_altitude
                self.cmd_scout_kml(kml_file, altitude)
            
            elif cmd == "SCOUT:START":
                # Legacy SCOUT:START - waypoint mode
                self.cmd_scout_start()
            
            elif cmd == "SCOUT:STOP":
                self.cmd_scout_stop()
            
            # Unknown
            else:
                self.send_response(f"ERROR: Unknown command '{cmd}'")
                
        except Exception as e:
            self.send_response(f"ERROR: {str(e)}")
    
    # ==================== COMMAND HANDLERS ====================
    
    def cmd_status(self):
        """Send current drone status."""
        mode = self.vehicle.mode.name
        armed = "ARMED" if self.vehicle.armed else "DISARMED"
        alt = self.vehicle.location.global_relative_frame.alt or 0
        bat = self.vehicle.battery.voltage if self.vehicle.battery else 0
        gps = self.vehicle.gps_0
        gps_fix = gps.fix_type if gps else 0
        gps_sats = gps.satellites_visible if gps else 0
        mission_status = "RUNNING" if self.mission_running else "IDLE"
        
        status = (f"STATUS: {mode},{armed},ALT={alt:.1f}m,BAT={bat:.1f}V,"
                  f"GPS={gps_fix}/sats={gps_sats},MISSION={mission_status}")
        self.send_response(status)
    
    def cmd_preflight(self):
        """Run preflight checks."""
        if not NIDAR_AVAILABLE or not self.preflight:
            self.send_response("ERROR: Preflight module not available")
            return
        
        self.send_response("Running preflight checks...")
        
        # Battery
        bat_ok = self.preflight.check_battery(min_voltage=10.5)
        bat_v = self.vehicle.battery.voltage if self.vehicle.battery else 0
        
        # GPS
        gps = self.vehicle.gps_0
        gps_fix = gps.fix_type if gps else 0
        gps_sats = gps.satellites_visible if gps else 0
        gps_ok = gps_fix >= 3
        
        # EKF
        ekf_ok = self.vehicle.ekf_ok
        
        # Armable
        armable = self.vehicle.is_armable
        
        all_ok = bat_ok and gps_ok and ekf_ok
        
        result = (f"PREFLIGHT: BAT={bat_v:.1f}V({'OK' if bat_ok else 'LOW'}), "
                  f"GPS={gps_fix}/sats={gps_sats}({'OK' if gps_ok else 'NO'}), "
                  f"EKF={'OK' if ekf_ok else 'NO'}, ARMABLE={'YES' if armable else 'NO'}, "
                  f"RESULT={'PASS' if all_ok else 'FAIL'}")
        self.send_response(result)
    
    def cmd_arm(self):
        """Arm the drone with preflight checks."""
        if self.vehicle.armed:
            self.send_response("Already armed")
            return
        
        # Set mode based on GPS
        gps = self.vehicle.gps_0
        if gps and gps.fix_type is not None and gps.fix_type >= 3:
            self.vehicle.mode = VehicleMode("GUIDED")
        else:
            self.vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(1)
        
        self.vehicle.armed = True
        
        timeout = 10
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                self.send_response("ARM FAILED: Timeout")
                return
            time.sleep(0.5)
        
        self.log("success", "Vehicle ARMED")
        self.send_response(f"ARMED OK (mode={self.vehicle.mode.name})")
    
    def cmd_force_arm(self):
        """Force arm the drone, bypassing ALL pre-arm checks."""
        if self.vehicle.armed:
            self.send_response("Already armed")
            return
        
        self.log("warning", "FORCE ARM - bypassing pre-arm checks!")
        
        # Set mode to STABILIZE (most permissive)
        self.vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(1)
        
        # Force arm by setting the parameter to skip safety checks
        # This uses MAVLink COMMAND_LONG with force flag
        try:
            from pymavlink import mavutil
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,       # confirmation
                1,       # param1 (1=arm, 0=disarm)
                21196,   # param2 (force arm magic number - bypasses checks)
                0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
        except Exception as e:
            self.log("error", f"Force arm MAVLink failed: {e}")
            # Fallback to standard arming
            self.vehicle.armed = True
        
        timeout = 10
        start = time.time()
        while not self.vehicle.armed:
            if time.time() - start > timeout:
                self.send_response("FORCE ARM FAILED: Timeout")
                return
            time.sleep(0.5)
        
        self.log("success", "Vehicle FORCE ARMED (checks bypassed)")
        self.send_response(f"FORCE ARMED OK (mode={self.vehicle.mode.name})")
    
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
        """Takeoff to altitude."""
        if not self.vehicle.armed:
            self.send_response("ERROR: Not armed")
            return
        
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
        
        self.log("state", f"Taking off to {altitude}m")
        self.send_response(f"TAKEOFF to {altitude}m...")
        self.vehicle.simple_takeoff(altitude)
        
        timeout = 30
        start = time.time()
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt or 0
            if current_alt >= altitude * 0.95:
                self.send_response(f"TAKEOFF OK: {current_alt:.1f}m")
                break
            if time.time() - start > timeout:
                self.send_response(f"TAKEOFF: At {current_alt:.1f}m (timeout)")
                break
            time.sleep(1)
    
    def cmd_land(self):
        """Land immediately."""
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.emergency_land()
        else:
            self.vehicle.mode = VehicleMode("LAND")
        self.send_response("LANDING...")
    
    def cmd_rtl(self):
        """Return to launch."""
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.return_to_launch()
        else:
            self.vehicle.mode = VehicleMode("RTL")
        self.send_response("RTL: Returning to launch")
    
    def cmd_abort(self):
        """Emergency abort."""
        self.log("warning", "EMERGENCY ABORT!")
        
        # Stop any running mission
        with self.abort_lock:
            self.abort_flag = True
        
        # Stop detection if running
        if self.detection_running:
            self.detection_running = False
            if self.detection_thread:
                self.detection_thread.join(timeout=1.0)
            if self.camera:
                self.camera.release()
                self.camera = None
        
        if NIDAR_AVAILABLE and self.safety_abort:
            self.safety_abort.emergency_land()
        else:
            self.vehicle.mode = VehicleMode("LAND")
        
        self.send_response("ABORT: Emergency landing!")
    
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
    
    def cmd_load_kml(self, filename):
        """Load mission from KML file."""
        # TODO: Implement KML parsing
        self.send_response(f"LOAD KML: {filename} (not implemented yet)")
    
    # ==================== WAYPOINT HANDLERS ====================
    
    def cmd_add_waypoint(self, lat, lon, alt):
        """Add a waypoint for scouting mission."""
        self.waypoints.append((lat, lon, alt))
        wp_num = len(self.waypoints)
        self.send_response(f"WP {wp_num} added: {lat:.6f},{lon:.6f},{alt:.1f}m")
    
    def cmd_clear_waypoints(self):
        """Clear all waypoints."""
        count = len(self.waypoints)
        self.waypoints = []
        self.send_response(f"Cleared {count} waypoints")
    
    def cmd_list_waypoints(self):
        """List all waypoints."""
        if not self.waypoints:
            self.send_response("No waypoints set")
            return
        
        self.send_response(f"Waypoints ({len(self.waypoints)} total):")
        for i, (lat, lon, alt) in enumerate(self.waypoints, 1):
            self.send_response(f"  WP{i}: {lat:.6f},{lon:.6f},{alt:.1f}m")
    
    def cmd_scout_start(self):
        """Start scouting mission - auto-starts detection and recording.
        
        If waypoints are set, will fly to each waypoint while detecting.
        If no waypoints, will hover in place and detect (stationary scout).
        """
        if self.mission_running:
            self.send_response("Mission already running")
            return
        
        # Check if detection modules available
        if not DETECTION_AVAILABLE:
            self.send_response("ERROR: Detection modules not available (cv2/onnxruntime)")
            return
        
        # Auto-start detection with recording
        if not self.detection_running:
            self.log("info", "SCOUT: Auto-starting detection with recording...")
            self._start_detection_with_recording()
            
            if not self.detection_running:
                self.send_response("ERROR: Failed to start detection")
                return
        elif not self.recording:
            # Detection running but not recording - start recording
            self._start_recording()
        
        # Reset abort flag
        with self.abort_lock:
            self.abort_flag = False
        
        # Determine scout mode
        if self.waypoints:
            # Waypoint-based scouting
            # Check if armed and in air
            if not self.vehicle.armed:
                self.send_response("ERROR: Drone not armed. Send ARM first, or use SCOUT without waypoints for ground test")
                return
            
            current_alt = self.vehicle.location.global_relative_frame.alt or 0
            if current_alt < 2.0:
                self.send_response("ERROR: Drone not airborne. Send TAKEOFF:10 first")
                return
            
            self.send_response(f"SCOUT: Starting with {len(self.waypoints)} waypoints, detection ON, recording ON")
            
            # Start scout mission in separate thread
            self.mission_thread = threading.Thread(target=self._run_scout_mission, daemon=True)
            self.mission_thread.start()
        else:
            # Stationary scout - just detection and recording
            self.mission_running = True
            self.send_response("SCOUT: Detection ON, recording ON (stationary mode - no waypoints)")
            self.send_response("Tip: Add waypoints with WP:lat,lon,alt or send SCOUT:STOP when done")
    
    def cmd_scout_kml(self, kml_filename, altitude):
        """
        Execute KML-based area survey mission with human detection.
        Loads KML file, generates coverage waypoints, uploads mission.
        
        Args:
            kml_filename: Name of KML file in /home/dart/quadtest/missions/
            altitude: Flight altitude in meters AGL
        """
        self.log("state", f"SCOUT: Loading KML survey {kml_filename} @ {altitude}m")
        self.send_response(f"Loading KML survey: {kml_filename}")
        
        try:
            # Import KML loader
            sys.path.insert(0, NIDAR_DIR)
            from mission.kml_loader import load_waypoints_from_kml, validate_kml_mission
            from mission.waypoint_mission import WaypointMission
            
            # Build KML file path
            kml_path = os.path.join(SCRIPT_DIR, "missions", kml_filename)
            if not os.path.exists(kml_path):
                self.send_response(f"ERROR: KML file not found: {kml_path}")
                return
            
            # Load waypoints from KML
            self.log("info", f"Generating waypoints from {kml_path}...")
            waypoints = load_waypoints_from_kml(
                kml_file=kml_path,
                altitude_meters=altitude,
                pattern=self.default_kml_pattern,
                camera_fov=57,
                overlap=0.25
            )
            
            self.send_response(f"Generated {len(waypoints)} waypoints")
            self.log("info", f"KML survey: {len(waypoints)} waypoints generated")
            
            # Validate mission
            valid, msg = validate_kml_mission(waypoints)
            if not valid:
                self.send_response(f"ERROR: {msg}")
                return
            
            # Upload mission to vehicle
            mission = WaypointMission(self.vehicle)
            if not mission.upload_mission(waypoints):
                self.send_response("ERROR: Mission upload failed")
                return
            
            self.send_response(f"Mission uploaded: {len(waypoints)} waypoints")
            self.log("success", "KML survey mission uploaded to Pixhawk")
            
            # Auto-start detection with recording
            if not self.detection_running:
                self.log("info", "SCOUT: Auto-starting detection with camera recording...")
                self._start_detection_with_recording()
                
                if not self.detection_running:
                    self.send_response("ERROR: Failed to start detection/camera")
                    return
            elif not self.recording:
                # Detection running but not recording - start recording
                self._start_recording()
            
            # Reset abort flag
            with self.abort_lock:
                self.abort_flag = False
            
            # Check if armed
            if not self.vehicle.armed:
                self.send_response("SCOUT: Mission ready. Send ARM and TAKEOFF to begin.")
                self.send_response(f"Camera: {'ON' if self.detection_running else 'FAILED'}")
                return
            
            # Check if in air
            current_alt = self.vehicle.location.global_relative_frame.alt or 0
            if current_alt < 2.0:
                self.send_response(f"SCOUT: Mission ready. Send TAKEOFF:{altitude} to begin.")
                self.send_response(f"Camera: {'ON' if self.detection_running else 'FAILED'}")
                return
            
            # Armed and airborne - auto-start mission
            self.send_response(f"SCOUT: Starting AUTO mode with {len(waypoints)} waypoints")
            self.send_response(f"Camera: {'ON' if self.detection_running else 'FAILED'}")
            
            # Start scout mission in separate thread
            self.mission_running = True
            self.mission_thread = threading.Thread(target=self._run_scout_mission_auto, daemon=True)
            self.mission_thread.start()
            
        except Exception as e:
            error_msg = f"KML survey error: {str(e)}"
            self.log("error", error_msg)
            self.send_response(f"ERROR: {error_msg}")
    
    def cmd_scout_stop(self):
        """Stop scouting - stops mission, detection, and recording."""
        # Stop mission if running
        if self.mission_running:
            with self.abort_lock:
                self.abort_flag = True
            self.mission_running = False
        
        # Stop recording
        if self.recording:
            self._stop_recording()
        
        # Stop detection
        if self.detection_running:
            self.cmd_detection_stop()
        
        self.send_response(f"SCOUT: Stopped (total detections: {self.detection_count})")
    
    def _start_detection_with_recording(self):
        """Start detection with video recording enabled."""
        if not DETECTION_AVAILABLE:
            self.send_response("ERROR: Detection modules not available (cv2/onnxruntime)")
            return
        
        if self.detection_running:
            return
        
        # Check if model exists
        if not os.path.exists(self.model_path):
            self.send_response(f"ERROR: Model not found at {self.model_path}")
            return
        
        self.send_response("Initializing detection with recording...")
        
        try:
            # Try RealSense SDK first for proper color output
            self.use_realsense = False
            if REALSENSE_AVAILABLE:
                try:
                    self.log("info", "Attempting to initialize RealSense camera...")
                    self.rs_pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    self.rs_pipeline.start(config)
                    self.use_realsense = True
                    self.log("success", "✓ RealSense COLOR camera initialized (640x480 @ 30fps)")
                    self.send_response("CAMERA: RealSense ONLINE")
                except Exception as e:
                    self.log("warning", f"RealSense init failed: {e}, trying OpenCV...")
                    self.send_response(f"Camera: RealSense failed, trying USB camera...")
                    self.rs_pipeline = None
            
            # Fallback to OpenCV if RealSense not available
            if not self.use_realsense:
                self.log("info", "Attempting to initialize USB camera (OpenCV)...")
                attempted_devices = []
                self.camera = cv2.VideoCapture(2)
                attempted_devices.append(2)
                
                if not self.camera.isOpened():
                    self.log("warning", "/dev/video2 failed, trying alternatives...")
                    for idx in [4, 0, 1]:
                        self.camera = cv2.VideoCapture(idx)
                        attempted_devices.append(idx)
                        if self.camera.isOpened():
                            self.log("success", f"✓ USB Camera initialized on /dev/video{idx}")
                            self.send_response(f"CAMERA: USB /dev/video{idx} ONLINE")
                            break
                
                if not self.camera.isOpened():
                    devices_tried = ", ".join([f"/dev/video{i}" for i in attempted_devices])
                    self.log("error", f"✗ Camera initialization FAILED - tried: {devices_tried}")
                    self.send_response(f"ERROR: Camera not found (tried {devices_tried})")
                    return
                else:
                    if 2 in attempted_devices and attempted_devices[0] == 2 and self.camera.isOpened():
                        self.log("success", "✓ USB Camera initialized on /dev/video2")
                        self.send_response("CAMERA: USB /dev/video2 ONLINE")
                
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # Initialize detector
            self.detector = YOLODetector(
                model_path=self.model_path,
                confidence_threshold=self.detection_confidence,
                iou_threshold=0.45,
                person_class_only=True
            )
            
            # Reset detection state
            self.detection_count = 0
            self.last_detection_time = 0
            with self.detection_lock:
                self.latest_detections = []
            
            # Start recording
            self._start_recording()
            
            # Start detection thread
            self.detection_running = True
            self.detection_thread = threading.Thread(target=self._detection_loop_with_recording, daemon=True)
            self.detection_thread.start()
            
            cam_type = "RealSense" if self.use_realsense else "USB OpenCV"
            self.log("success", f"✓ Detection system ACTIVE with {cam_type} camera")
            self.send_response(f"DETECTION: ACTIVE ({cam_type} recording)")
            
        except Exception as e:
            self.log("error", f"Detection start failed: {e}")
            self.send_response(f"ERROR: Detection start failed - {e}")
            self.detection_running = False
            self._stop_recording()
            if self.rs_pipeline:
                try:
                    self.rs_pipeline.stop()
                except:
                    pass
                self.rs_pipeline = None
            if self.camera:
                self.camera.release()
                self.camera = None
    
    def _start_recording(self):
        """Start video recording."""
        try:
            # Create recordings directory
            recordings_dir = os.path.join(SCRIPT_DIR, 'recordings')
            os.makedirs(recordings_dir, exist_ok=True)
            
            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.recording_path = os.path.join(recordings_dir, f"scout_{timestamp}.mp4")
            
            # Initialize video writer (640x480 @ 30fps)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.recording_path,
                fourcc,
                30.0,
                (640, 480)
            )
            
            # CRITICAL: Check if VideoWriter actually opened successfully
            if not self.video_writer.isOpened():
                self.log("error", f"VideoWriter failed to open: {self.recording_path}")
                self.log("error", "Trying alternative codec (XVID)...")
                
                # Try XVID codec as fallback
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.recording_path = os.path.join(recordings_dir, f"scout_{timestamp}.avi")
                self.video_writer = cv2.VideoWriter(
                    self.recording_path,
                    fourcc,
                    30.0,
                    (640, 480)
                )
                
                if not self.video_writer.isOpened():
                    self.log("error", "VideoWriter failed with both mp4v and XVID codecs")
                    self.send_response("ERROR: Video recording failed - codec issue")
                    self.recording = False
                    self.video_writer = None
                    return
                else:
                    self.log("warning", f"Using XVID codec (AVI format): {self.recording_path}")
            
            self.recording = True
            self.recording_start_time = time.time()
            self.log("success", f"Recording started: {self.recording_path}")
            self.send_response(f"REC: Started - {os.path.basename(self.recording_path)}")
            
        except Exception as e:
            self.log("error", f"Failed to start recording: {e}")
            self.send_response(f"ERROR: Recording failed - {e}")
            self.recording = False
            self.video_writer = None
    
    def _stop_recording(self):
        """Stop video recording and save file."""
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        if self.recording:
            duration = time.time() - self.recording_start_time
            self.log("success", f"Recording saved: {self.recording_path} ({duration:.1f}s)")
            self.send_response(f"VIDEO: Saved {self.recording_path} ({duration:.1f}s)")
        
        self.recording = False
        self.recording_path = None
    
    def _run_scout_mission(self):
        """Execute scouting mission - fly waypoints with human detection."""
        self.mission_running = True
        
        try:
            # Ensure in GUIDED mode
            if self.vehicle.mode.name != "GUIDED":
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
            
            # Fly to each waypoint
            for i, (lat, lon, alt) in enumerate(self.waypoints, 1):
                # Check abort
                with self.abort_lock:
                    if self.abort_flag:
                        self.send_response("Scout stopped")
                        self._stop_recording()
                        self.mission_running = False
                        return
                
                self.send_response(f"Flying to WP{i}/{len(self.waypoints)}: {lat:.6f},{lon:.6f}")
                
                location = LocationGlobalRelative(lat, lon, alt)
                self.vehicle.simple_goto(location)
                
                # Wait until close to waypoint
                while True:
                    with self.abort_lock:
                        if self.abort_flag:
                            self.send_response("Scout stopped")
                            self._stop_recording()
                            self.mission_running = False
                            return
                    
                    current = self.vehicle.location.global_relative_frame
                    dist = self._get_distance_metres(current, location)
                    
                    if dist < 3.0:  # Within 3 meters
                        self.send_response(f"Reached WP{i}")
                        break
                    
                    time.sleep(1)
                
                # Brief hover at waypoint to scan
                time.sleep(2)
            
            # Done with waypoints - stop recording but keep detection
            self._stop_recording()
            self.send_response(f"SCOUT DONE! All {len(self.waypoints)} waypoints visited. Detected {self.detection_count} humans.")
            self.send_response("Send RTL to return home, or add more waypoints")
            
        except Exception as e:
            self.log("error", f"Scout error: {e}")
            self.send_response(f"SCOUT ERROR: {e}")
            self._stop_recording()
        
        finally:
            self.mission_running = False
    
    def _run_scout_mission_auto(self):
        """Execute AUTO mode KML survey mission with human detection."""
        self.mission_running = True
        
        try:
            # Switch to AUTO mode
            self.log("info", "Switching to AUTO mode for mission...")
            self.vehicle.mode = VehicleMode("AUTO")
            time.sleep(2)
            
            if self.vehicle.mode.name != "AUTO":
                self.send_response("ERROR: Failed to switch to AUTO mode")
                self.mission_running = False
                return
            
            self.send_response(f"AUTO mode active - mission started")
            self.log("success", "AUTO mode activated - ArduPilot executing mission")
            
            # Monitor mission progress
            last_wp = 0
            while self.vehicle.armed:
                # Check abort
                with self.abort_lock:
                    if self.abort_flag:
                        self.send_response("SCOUT: Aborting mission")
                        self.vehicle.mode = VehicleMode("RTL")
                        break
                
                # Get current waypoint
                current_wp = self.vehicle.commands.next
                if current_wp != last_wp and current_wp > 0:
                    self.send_response(f"Waypoint {current_wp}/{self.vehicle.commands.count}")
                    self.log("info", f"Progress: WP {current_wp}/{self.vehicle.commands.count}")
                    last_wp = current_wp
                
                # Check if mission complete
                if current_wp == 0 and last_wp > 0:
                    self.send_response("Mission waypoints complete!")
                    break
                
                time.sleep(2)
            
            # Stop recording
            self._stop_recording()
            self.send_response(f"SCOUT COMPLETE! Total detections: {self.detection_count}")
            self.log("success", f"KML survey complete - {self.detection_count} human detections")
            
        except Exception as e:
            self.log("error", f"AUTO mission error: {e}")
            self.send_response(f"ERROR: {e}")
            self._stop_recording()
        
        finally:
            self.mission_running = False
    
    def _get_distance_metres(self, loc1, loc2):
        """Get distance between two locations in meters."""
        import math
        dlat = loc2.lat - loc1.lat
        dlon = loc2.lon - loc1.lon
        return math.sqrt((dlat*111320)**2 + (dlon*111320*math.cos(math.radians(loc1.lat)))**2)
    
    # ==================== HUMAN DETECTION HANDLERS ====================
    
    def cmd_detection_start(self):
        """Start human detection."""
        if not DETECTION_AVAILABLE:
            self.send_response("ERROR: Detection modules not available (cv2/onnxruntime)")
            return
        
        if self.detection_running:
            self.send_response("Detection already running")
            return
        
        # Check if model exists
        if not os.path.exists(self.model_path):
            self.send_response(f"ERROR: Model not found at {self.model_path}")
            return
        
        self.send_response("Initializing human detection...")
        
        try:
            # Try RealSense SDK first for proper color output
            self.use_realsense = False
            if REALSENSE_AVAILABLE:
                try:
                    self.rs_pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                    self.rs_pipeline.start(config)
                    self.use_realsense = True
                    self.log("success", "RealSense COLOR stream initialized (640x480)")
                except Exception as e:
                    self.log("warning", f"RealSense init failed: {e}, falling back to OpenCV")
                    self.rs_pipeline = None
            
            # Fallback to OpenCV if RealSense not available
            if not self.use_realsense:
                self.camera = cv2.VideoCapture(2)
                if not self.camera.isOpened():
                    for idx in [4, 0, 1]:
                        self.camera = cv2.VideoCapture(idx)
                        if self.camera.isOpened():
                            self.log("info", f"Using camera /dev/video{idx}")
                            break
                
                if not self.camera.isOpened():
                    self.send_response("ERROR: Could not open camera")
                    return
                
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # Initialize detector
            self.detector = YOLODetector(
                model_path=self.model_path,
                confidence_threshold=self.detection_confidence,
                iou_threshold=0.45,
                person_class_only=True
            )
            
            # Reset detection state
            self.detection_count = 0
            self.last_detection_time = 0
            with self.detection_lock:
                self.latest_detections = []
            
            # Start detection thread
            self.detection_running = True
            self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
            self.detection_thread.start()
            
            cam_type = "RealSense" if self.use_realsense else "OpenCV"
            self.log("success", f"Human detection started ({cam_type})")
            self.send_response(f"DETECTION: Started ({cam_type}, 640x480, skip=3)")
            
        except Exception as e:
            self.log("error", f"Detection start failed: {e}")
            self.send_response(f"ERROR: Detection start failed - {e}")
            self.detection_running = False
            if self.rs_pipeline:
                try:
                    self.rs_pipeline.stop()
                except:
                    pass
                self.rs_pipeline = None
            if self.camera:
                self.camera.release()
                self.camera = None
    
    def cmd_detection_stop(self):
        """Stop human detection."""
        if not self.detection_running:
            self.send_response("Detection not running")
            return
        
        self.detection_running = False
        
        # Wait for thread to stop
        if self.detection_thread:
            self.detection_thread.join(timeout=2.0)
            self.detection_thread = None
        
        # Release camera/RealSense
        if self.rs_pipeline:
            try:
                self.rs_pipeline.stop()
            except:
                pass
            self.rs_pipeline = None
        if self.camera:
            self.camera.release()
            self.camera = None
        
        self.use_realsense = False
        self.detector = None
        
        self.log("info", "Human detection stopped")
        self.send_response(f"DETECTION: Stopped (total detections: {self.detection_count})")
    
    def cmd_detection_status(self):
        """Get detection status."""
        if not self.detection_running:
            self.send_response("DETECTION: Not running")
            return
        
        with self.detection_lock:
            current_count = len(self.latest_detections)
            if current_count > 0:
                boxes, scores, ts = self.latest_detections[-1]
                humans = len(boxes)
                max_conf = max(scores) if scores else 0
            else:
                humans = 0
                max_conf = 0
        
        age = time.time() - self.last_detection_time if self.last_detection_time > 0 else 0
        rec_status = "YES" if self.recording else "NO"
        rec_duration = f"{time.time() - self.recording_start_time:.1f}s" if self.recording else "N/A"
        
        self.send_response(f"DETECTION: Running, humans={humans}, "
                          f"conf_max={max_conf:.2f}, total={self.detection_count}, "
                          f"age={age:.1f}s, recording={rec_status} ({rec_duration})")
    
    def cmd_detection_set_confidence(self, conf):
        """Set detection confidence threshold."""
        if conf < 0.1 or conf > 1.0:
            self.send_response("ERROR: Confidence must be 0.1-1.0")
            return
        
        self.detection_confidence = conf
        
        if self.detector:
            self.detector.update_threshold(confidence=conf)
        
        self.send_response(f"DETECTION: Confidence set to {conf}")
    
    def _detection_loop(self):
        """Background thread for running human detection."""
        frame_count = 0
        
        self.log("info", "Detection thread started")
        
        while self.detection_running:
            try:
                # Capture frame from RealSense or OpenCV
                if self.use_realsense and self.rs_pipeline:
                    frames = self.rs_pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    frame = np.asanyarray(color_frame.get_data())
                    ret = True
                else:
                    ret, frame = self.camera.read()
                
                if not ret:
                    time.sleep(0.01)
                    continue
                
                frame_count += 1
                
                # Skip frames for performance
                if frame_count % self.detection_skip_frames != 0:
                    continue
                
                # Resize for faster processing
                frame = cv2.resize(frame, (320, 240))
                
                # Convert BGR to RGB for detection
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Run detection
                boxes, scores = self.detector.detect(frame_rgb)
                
                # Update shared state
                with self.detection_lock:
                    self.latest_detections.append((boxes, scores, time.time()))
                    # Keep only last 10 detections
                    if len(self.latest_detections) > 10:
                        self.latest_detections.pop(0)
                
                # If humans detected, send notification
                if len(boxes) > 0:
                    self.detection_count += len(boxes)
                    self.last_detection_time = time.time()
                    
                    # Get drone location for logging
                    if self.vehicle:
                        loc = self.vehicle.location.global_relative_frame
                        lat = loc.lat if loc.lat else 0
                        lon = loc.lon if loc.lon else 0
                        alt = loc.alt if loc.alt else 0
                        
                        self.send_response(f"HUMAN DETECTED: {len(boxes)} person(s), "
                                          f"conf={max(scores):.2f}, "
                                          f"loc={lat:.6f},{lon:.6f},{alt:.1f}m")
                    else:
                        self.send_response(f"HUMAN DETECTED: {len(boxes)} person(s), "
                                          f"conf={max(scores):.2f}")
                
                time.sleep(0.001)  # Small delay to prevent CPU overload
                
            except Exception as e:
                self.log("error", f"Detection error: {e}")
                time.sleep(0.1)
        
        self.log("info", "Detection thread stopped")
    
    def _detection_loop_with_recording(self):
        """Background thread for running human detection with video recording."""
        frame_count = 0
        
        self.log("info", "Detection thread with recording started")
        
        while self.detection_running:
            try:
                # Capture frame from RealSense or OpenCV
                if self.use_realsense and self.rs_pipeline:
                    frames = self.rs_pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    frame = np.asanyarray(color_frame.get_data())
                    ret = True
                else:
                    ret, frame = self.camera.read()
                
                if not ret:
                    time.sleep(0.01)
                    continue
                
                # Always record raw frame (before resize) if recording is active
                if self.recording and self.video_writer:
                    # Ensure frame is 640x480 for recording
                    if frame.shape[:2] != (480, 640):
                        record_frame = cv2.resize(frame, (640, 480))
                    else:
                        record_frame = frame.copy()
                    self.video_writer.write(record_frame)
                
                frame_count += 1
                
                # Skip frames for performance (detection only, not recording)
                if frame_count % self.detection_skip_frames != 0:
                    continue
                
                # Resize for faster processing
                frame_resized = cv2.resize(frame, (320, 240))
                
                # Convert BGR to RGB for detection
                frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
                
                # Run detection
                boxes, scores = self.detector.detect(frame_rgb)
                
                # Update shared state
                with self.detection_lock:
                    self.latest_detections.append((boxes, scores, time.time()))
                    # Keep only last 10 detections
                    if len(self.latest_detections) > 10:
                        self.latest_detections.pop(0)
                
                # If humans detected, send notification
                if len(boxes) > 0:
                    self.detection_count += len(boxes)
                    self.last_detection_time = time.time()
                    
                    # Get drone location for logging
                    if self.vehicle:
                        loc = self.vehicle.location.global_relative_frame
                        lat = loc.lat if loc.lat else 0
                        lon = loc.lon if loc.lon else 0
                        alt = loc.alt if loc.alt else 0
                        
                        self.send_response(f"HUMAN DETECTED: {len(boxes)} person(s), "
                                          f"conf={max(scores):.2f}, "
                                          f"loc={lat:.6f},{lon:.6f},{alt:.1f}m")
                    else:
                        self.send_response(f"HUMAN DETECTED: {len(boxes)} person(s), "
                                          f"conf={max(scores):.2f}")
                
                time.sleep(0.001)  # Small delay to prevent CPU overload
                
            except Exception as e:
                self.log("error", f"Detection error: {e}")
                time.sleep(0.1)
        
        # Stop recording when detection stops
        if self.recording:
            self._stop_recording()
        
        self.log("info", "Detection thread with recording stopped")
    
    def get_latest_detection(self):
        """Get the most recent detection result (for external use)."""
        with self.detection_lock:
            if len(self.latest_detections) > 0:
                return self.latest_detections[-1]
            return ([], [], 0)
    
    # ==================== MISSION HANDLERS ====================
    
    def cmd_mission_start(self):
        """Start the autonomous mission."""
        if self.mission_running:
            self.send_response("Mission already running")
            return
        
        if not NIDAR_AVAILABLE:
            self.send_response("ERROR: Mission modules not available")
            return
        
        self.send_response("Starting autonomous mission...")
        
        # Auto-start human detection for scouting
        if DETECTION_AVAILABLE and not self.detection_running:
            self.log("info", "Auto-starting human detection for scouting mission")
            self.cmd_detection_start()
        
        # Reset abort flag
        with self.abort_lock:
            self.abort_flag = False
        
        # Start mission in separate thread
        self.mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self.mission_thread.start()
    
    def cmd_mission_stop(self):
        """Stop current mission."""
        if not self.mission_running:
            self.send_response("No mission running")
            return
        
        with self.abort_lock:
            self.abort_flag = True
        
        self.send_response("Mission stop requested - initiating RTL")
    
    def _run_mission(self):
        """Execute the autonomous mission (runs in separate thread)."""
        self.mission_running = True
        
        try:
            # Step 1: Preflight checks
            self.log("header", "AUTONOMOUS MISSION")
            self.send_response("Running preflight checks...")
            
            if not self.preflight.run_all_checks():
                self.send_response("MISSION FAILED: Preflight checks failed")
                self.mission_running = False
                return
            
            # Check abort
            with self.abort_lock:
                if self.abort_flag:
                    self.send_response("Mission aborted")
                    self.mission_running = False
                    return
            
            # Step 2: Upload mission
            self.send_response("Uploading mission waypoints...")
            waypoints = MissionData.get_waypoints()
            
            if not self.mission.upload_mission(waypoints):
                self.send_response("MISSION FAILED: Upload failed")
                self.mission_running = False
                return
            
            # Step 3: Arm and takeoff
            self.send_response("Arming and taking off...")
            
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            self.vehicle.armed = True
            
            timeout = 10
            start = time.time()
            while not self.vehicle.armed:
                if time.time() - start > timeout:
                    self.send_response("MISSION FAILED: Arm timeout")
                    self.mission_running = False
                    return
                time.sleep(0.5)
            
            # Takeoff
            takeoff_alt = MissionData.TAKEOFF_ALTITUDE
            self.vehicle.simple_takeoff(takeoff_alt)
            self.send_response(f"Taking off to {takeoff_alt}m...")
            
            while True:
                alt = self.vehicle.location.global_relative_frame.alt or 0
                if alt >= takeoff_alt * 0.95:
                    break
                with self.abort_lock:
                    if self.abort_flag:
                        self.safety_abort.return_to_launch()
                        self.send_response("Mission aborted during takeoff")
                        self.mission_running = False
                        return
                time.sleep(1)
            
            # Step 4: Start AUTO mode
            self.send_response("Starting waypoint navigation...")
            if not self.mission.start_mission():
                self.send_response("MISSION FAILED: AUTO mode failed")
                self.safety_abort.return_to_launch()
                self.mission_running = False
                return
            
            # Step 5: Monitor mission
            last_wp = 0
            while True:
                with self.abort_lock:
                    if self.abort_flag:
                        self.safety_abort.return_to_launch()
                        self.send_response("Mission aborted - RTL")
                        self.mission_running = False
                        return
                
                status = self.mission.get_mission_status()
                current_wp = status['current_waypoint']
                
                if current_wp != last_wp:
                    self.send_response(f"Waypoint {current_wp}/{status['total_waypoints']}")
                    last_wp = current_wp
                
                if self.mission.monitor_mission():
                    break
                
                time.sleep(1)
            
            # Step 6: RTL
            self.send_response("Mission complete - returning home")
            self.safety_abort.return_to_launch()
            
            # Wait for landing
            while not self.safety_abort.monitor_landing():
                time.sleep(2)
            
            self.send_response("MISSION COMPLETE!")
            self.log("success", "Mission completed successfully")
            
        except Exception as e:
            self.log("error", f"Mission error: {e}")
            self.send_response(f"MISSION ERROR: {e}")
            if self.safety_abort:
                self.safety_abort.emergency_land()
        
        finally:
            self.mission_running = False
            # Auto-stop detection when mission ends
            if self.detection_running:
                self.log("info", "Auto-stopping detection (mission ended)")
                self.cmd_detection_stop()
    
    # ==================== MAIN LOOP ====================
    
    def reconnect_radio(self):
        """Attempt to reconnect to the radio after disconnection."""
        max_retries = 10
        retry_delay = 3  # seconds
        
        self.log("warning", "Radio disconnected - attempting reconnection...")
        
        # Close existing connection if any
        if self.radio:
            try:
                self.radio.close()
            except:
                pass
            self.radio = None
        
        for attempt in range(1, max_retries + 1):
            self.log("info", f"Reconnection attempt {attempt}/{max_retries}...")
            
            # Check if symlink exists
            if not os.path.exists(self.radio_port):
                self.log("warning", f"Radio port {self.radio_port} not found, waiting...")
                time.sleep(retry_delay)
                continue
            
            try:
                self.radio = serial.Serial(
                    port=self.radio_port,
                    baudrate=self.radio_baud,
                    timeout=0.1
                )
                self.log("success", "Radio reconnected successfully!")
                
                # Update telemetry logger with new radio reference
                if self.telem_logger:
                    self.telem_logger.set_radio(self.radio)
                    self.telem_logger.enable(True)
                
                return True
            except Exception as e:
                self.log("error", f"Reconnection failed: {e}")
                time.sleep(retry_delay)
        
        self.log("error", f"Failed to reconnect after {max_retries} attempts")
        return False
    
    def listen_loop(self):
        """Main loop - listen for commands."""
        self.log("header", "LISTENING FOR COMMANDS")
        self.log("info", "Waiting for commands from ground station...")
        self.log("info", "Scout: SCOUT (auto-starts detection+recording), SCOUT:STOP")
        self.log("info", "Waypoints: WP:lat,lon,alt, WP:CLEAR, WP:LIST")
        self.log("info", "Flight: ARM, FORCEARM, TAKEOFF:x, LAND, RTL, ABORT")
        self.log("info", "Detection: DETECT:START, DETECT:STOP, DETECT:STATUS")
        
        # Send ready message
        detection_status = "available" if DETECTION_AVAILABLE else "not available"
        self.send_response(f"DRONE READY - Detection {detection_status}")
        
        buffer = ""
        consecutive_errors = 0
        max_consecutive_errors = 5  # Trigger reconnect after 5 consecutive errors
        
        while self.running:
            try:
                if self.radio.in_waiting > 0:
                    data = self.radio.read(self.radio.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        cmd, buffer = buffer.split('\n', 1)
                        cmd = cmd.strip()
                        if cmd:
                            self.execute_command(cmd)
                
                consecutive_errors = 0  # Reset on success
                time.sleep(0.05)
                
            except KeyboardInterrupt:
                self.log("info", "Stopping...")
                break
            except OSError as e:
                # I/O errors (errno 5) indicate radio disconnection
                consecutive_errors += 1
                self.log("error", f"Radio I/O error ({consecutive_errors}/{max_consecutive_errors}): {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    self.log("warning", "Too many consecutive errors - radio likely disconnected")
                    if self.reconnect_radio():
                        consecutive_errors = 0
                        buffer = ""  # Clear buffer after reconnect
                        self.send_response("DRONE RECONNECTED")
                    else:
                        self.log("error", "Radio reconnection failed - continuing without radio")
                        # Keep trying periodically
                        time.sleep(10)
                        if self.reconnect_radio():
                            consecutive_errors = 0
                            buffer = ""
                else:
                    time.sleep(1)
            except Exception as e:
                self.log("error", f"Error: {e}")
                time.sleep(1)
    
    def run(self):
        """Start the main controller."""
        self.log("header", "AUTONOMOUS DRONE CONTROLLER")
        self.log("info", "LoRa Command Interface + Mission Control + Human Detection")
        
        if DETECTION_AVAILABLE:
            self.log("info", f"Detection model: {self.model_path}")
        else:
            self.log("warning", "Human detection not available")
        
        if not self.connect_pixhawk():
            return False
        
        # Try to connect radio (optional if --no-radio)
        radio_connected = self.connect_radio()
        if not radio_connected:
            if self.require_radio:
                return False
            else:
                self.log("warning", "Running WITHOUT radio (--no-radio mode)")
                self.log("info", "Pixhawk connected - use for testing only")
        
        # Setup telemetry AFTER radio is connected (sends logs via radio)
        if radio_connected and TELEMETRY_AVAILABLE and self.telemetry_enabled:
            self._setup_telemetry()
        
        self.running = True
        
        try:
            if radio_connected:
                self.listen_loop()
            else:
                # No radio - just keep running for testing
                self.log("info", "No radio - entering idle mode (Ctrl+C to exit)")
                self.log("info", "Connect via MAVProxy/QGC to control vehicle")
                while self.running:
                    time.sleep(1)
        finally:
            self.running = False
            
            # Stop detection if running
            if self.detection_running:
                self.detection_running = False
                if self.detection_thread:
                    self.detection_thread.join(timeout=2.0)
                if self.camera:
                    self.camera.release()
            
            # Stop telemetry
            if self.telemetry_enabled:
                self._stop_telemetry()
            
            if self.vehicle:
                self.vehicle.close()
            if self.radio:
                self.radio.close()
            self.log("info", "Shutdown complete")
        
        return True


def main():
    # Initialize session file logging first (captures all output)
    session_logger = None
    if FILE_LOGGING_AVAILABLE:
        session_logger = init_session_logging()
        print(f"[INFO] Session logs being saved to: {session_logger.session_file}")
    
    parser = argparse.ArgumentParser(description='Autonomous Drone Controller with LoRa Interface')
    parser.add_argument('--pixhawk', default='/dev/ttyACM0',
                       help='Pixhawk port (default: /dev/ttyACM0)')
    parser.add_argument('--pixhawk-baud', type=int, default=115200,
                       help='Pixhawk baud rate (default: 115200)')
    parser.add_argument('--radio', default='/dev/ttyUSB-radio',
                       help='LoRa/3DR radio port (default: /dev/ttyUSB-radio)')
    parser.add_argument('--radio-baud', type=int, default=57600,
                       help='Radio baud rate (default: 57600)')
    parser.add_argument('--no-radio', action='store_true',
                       help='Run without radio (Pixhawk only, for testing)')
    args = parser.parse_args()
    
    controller = MainController(
        pixhawk_port=args.pixhawk,
        pixhawk_baud=args.pixhawk_baud,
        radio_port=args.radio,
        radio_baud=args.radio_baud,
        require_radio=not args.no_radio
    )
    
    try:
        success = controller.run()
    finally:
        # Close session logger to write footer
        if session_logger:
            session_logger.close()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
