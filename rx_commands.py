#!/usr/bin/env python3
"""
Drone Command Receiver via LoRa/3DR Radio

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
    
Hardware Setup:
    - Pixhawk connected via USB (/dev/ttyACM0)
    - 3DR Radio connected via USB (/dev/ttyUSB0) for receiving commands

Usage:
    python3 rx_commands.py
    python3 rx_commands.py --pixhawk /dev/ttyACM0 --radio /dev/ttyUSB0
    python3 rx_commands.py --skip-preflight  # Skip preflight checks (bench test)
"""

import sys
import os
import time
import argparse
import serial
import threading
from datetime import datetime

# Add nidar/ to path for importing modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NIDAR_DIR = os.path.join(SCRIPT_DIR, 'nidar')
sys.path.insert(0, NIDAR_DIR)

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
            
            # Check battery
            if not self.preflight.check_battery(min_voltage=10.5):
                self.send_response("ARM FAILED: Battery too low")
                return
            
            # Check GPS (optional - warn but continue)
            gps = self.vehicle.gps_0
            if gps and gps.fix_type < 3:
                self.log("warning", f"GPS fix type {gps.fix_type} (need 3 for 3D)")
                self.send_response(f"WARN: GPS fix={gps.fix_type} (need 3)")
            
            # Check EKF
            if not self.vehicle.ekf_ok:
                self.log("warning", "EKF not converged")
                self.send_response("WARN: EKF not converged")
            
            self.send_response("Preflight OK, arming...")
        
        # Set mode - use STABILIZE if no GPS, else GUIDED
        gps = self.vehicle.gps_0
        if gps and gps.fix_type >= 3:
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
        bat_v = self.vehicle.battery.voltage or 0
        
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
        """Send current status."""
        mode = self.vehicle.mode.name
        armed = "ARM" if self.vehicle.armed else "DISARM"
        alt = self.vehicle.location.global_relative_frame.alt or 0
        bat = self.vehicle.battery.voltage or 0
        gps = self.vehicle.gps_0.fix_type if self.vehicle.gps_0 else 0
        
        status = f"STATUS: {mode},{armed},ALT={alt:.1f}m,BAT={bat:.1f}V,GPS={gps}"
        self.send_response(status)
    
    def listen_loop(self):
        """Main loop to listen for commands."""
        if NIDAR_AVAILABLE:
            MissionLogger.header("LISTENING FOR COMMANDS")
        else:
            print("\n" + "=" * 50)
            print("  LISTENING FOR COMMANDS")
            print("=" * 50)
        
        self.log("info", "Waiting for commands from remote laptop...")
        self.log("info", "Commands: ARM, DISARM, TAKEOFF:5, LAND, RTL, ABORT, PREFLIGHT, STATUS")
        
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
