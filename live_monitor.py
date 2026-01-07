#!/usr/bin/env python3
"""
Real-Time Telemetry Monitor
============================
Live display of flight telemetry, errors, mode changes, and system status.

Features:
- Real-time GPS (lat, lon, altitude)
- Flight mode tracking with change history
- Battery monitoring with alerts
- Attitude (roll, pitch, yaw)
- Velocity and ground speed
- Error/warning log with timestamps
- EKF and system health status

Usage:
    python3 live_monitor.py
    python3 live_monitor.py --port /dev/ttyACM0 --baud 115200
    python3 live_monitor.py --port /dev/ttyUSB0 --baud 57600  # For 3DR radio
"""

import sys
import os
import time
import math
import argparse
import threading
from datetime import datetime
from collections import deque

# ANSI Colors
class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\033[95m"
    WHITE = "\033[97m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_YELLOW = "\033[43m"


class TelemetryMonitor:
    """Real-time telemetry monitor with error logging."""
    
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.vehicle = None
        self.running = False
        
        # Telemetry state
        self.last_mode = None
        self.last_armed = None
        self.update_count = 0
        self.start_time = None
        
        # Event logs (errors, warnings, mode changes)
        self.event_log = deque(maxlen=15)  # Keep last 15 events
        self.mode_history = deque(maxlen=10)
        
        # Thresholds for alerts
        self.battery_warn = 11.0  # Voltage warning
        self.battery_critical = 10.5  # Critical voltage
        self.gps_min_sats = 6  # Minimum satellites for good fix
        
        # Listeners
        self._listeners_added = False
    
    def log_event(self, level, message):
        """Add event to the log."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        color = Colors.WHITE
        if level == "ERROR":
            color = Colors.RED
        elif level == "WARN":
            color = Colors.YELLOW
        elif level == "MODE":
            color = Colors.CYAN
        elif level == "ARM":
            color = Colors.GREEN
        elif level == "DISARM":
            color = Colors.MAGENTA
        elif level == "INFO":
            color = Colors.BLUE
        
        self.event_log.append({
            'time': timestamp,
            'level': level,
            'message': message,
            'color': color
        })
    
    def clear_screen(self):
        """Clear terminal screen."""
        print("\033[2J\033[H", end="")
    
    def move_cursor(self, row, col):
        """Move cursor to position."""
        print(f"\033[{row};{col}H", end="")
    
    def connect(self):
        """Connect to the vehicle."""
        print(f"\n{Colors.CYAN}{'='*70}")
        print(f"   🚁 REAL-TIME TELEMETRY MONITOR")
        print(f"{'='*70}{Colors.RESET}")
        print(f"\nConnecting to {self.port} @ {self.baud} baud...")
        
        try:
            from dronekit import connect, VehicleMode
            self.vehicle = connect(self.port, baud=self.baud, wait_ready=False, timeout=30)
            time.sleep(2)  # Let data flow
            print(f"{Colors.GREEN}✓ Connected!{Colors.RESET}")
            
            # Add listeners for mode and arm state changes
            self._add_listeners()
            
            return True
        except ImportError:
            print(f"{Colors.RED}ERROR: DroneKit not installed. Run: pip install dronekit{Colors.RESET}")
            return False
        except Exception as e:
            print(f"{Colors.RED}✗ Connection failed: {e}{Colors.RESET}")
            return False
    
    def _add_listeners(self):
        """Add vehicle attribute listeners."""
        if self._listeners_added:
            return
        
        @self.vehicle.on_attribute('mode')
        def mode_callback(self_vehicle, attr_name, value):
            mode_name = value.name
            if self.last_mode and self.last_mode != mode_name:
                self.log_event("MODE", f"Mode changed: {self.last_mode} → {mode_name}")
                self.mode_history.append({
                    'time': datetime.now().strftime("%H:%M:%S"),
                    'from': self.last_mode,
                    'to': mode_name
                })
            self.last_mode = mode_name
        
        @self.vehicle.on_attribute('armed')
        def armed_callback(self_vehicle, attr_name, value):
            if self.last_armed is not None and self.last_armed != value:
                if value:
                    self.log_event("ARM", "🔴 Vehicle ARMED")
                else:
                    self.log_event("DISARM", "🟢 Vehicle DISARMED")
            self.last_armed = value
        
        @self.vehicle.on_attribute('system_status')
        def status_callback(self_vehicle, attr_name, value):
            status = str(value)
            if 'CRITICAL' in status.upper():
                self.log_event("ERROR", f"System status: {status}")
            elif 'EMERGENCY' in status.upper():
                self.log_event("ERROR", f"EMERGENCY: {status}")
        
        self._listeners_added = True
        self.last_mode = self.vehicle.mode.name
        self.last_armed = self.vehicle.armed
    
    def format_gps_fix(self, fix_type):
        """Convert GPS fix type to readable string with color."""
        fix_info = {
            0: ("No GPS", Colors.RED),
            1: ("No Fix", Colors.RED),
            2: ("2D Fix", Colors.YELLOW),
            3: ("3D Fix", Colors.GREEN),
            4: ("DGPS", Colors.GREEN),
            5: ("RTK Float", Colors.CYAN),
            6: ("RTK Fixed", Colors.CYAN)
        }
        return fix_info.get(fix_type, (f"Unknown({fix_type})", Colors.YELLOW))
    
    def get_battery_color(self, voltage):
        """Get color based on battery voltage."""
        if voltage is None or voltage == 0:
            return Colors.WHITE
        if voltage < self.battery_critical:
            return Colors.RED
        elif voltage < self.battery_warn:
            return Colors.YELLOW
        return Colors.GREEN
    
    def check_alerts(self):
        """Check for alert conditions and log them."""
        # Battery check
        battery = self.vehicle.battery
        if battery and battery.voltage:
            voltage = battery.voltage
            if voltage < self.battery_critical:
                self.log_event("ERROR", f"CRITICAL: Battery {voltage:.1f}V - LAND NOW!")
            elif voltage < self.battery_warn:
                self.log_event("WARN", f"Battery low: {voltage:.1f}V")
        
        # GPS check
        gps = self.vehicle.gps_0
        if gps and gps.fix_type < 3:
            self.log_event("WARN", f"Poor GPS: Fix={gps.fix_type}, Sats={gps.satellites_visible}")
        
        # EKF check
        if hasattr(self.vehicle, 'ekf_ok') and not self.vehicle.ekf_ok:
            self.log_event("WARN", "EKF not healthy")
    
    def draw_progress_bar(self, value, max_val, width=20, fill_char="█", empty_char="░"):
        """Draw a progress bar."""
        if max_val == 0:
            return empty_char * width
        percent = min(100, max(0, (value / max_val) * 100))
        filled = int(percent / 100 * width)
        return fill_char * filled + empty_char * (width - filled)
    
    def display(self):
        """Display the telemetry dashboard."""
        self.clear_screen()
        
        now = datetime.now().strftime("%H:%M:%S")
        elapsed = time.time() - self.start_time
        
        # Header
        print(f"{Colors.BOLD}{Colors.CYAN}╔{'═'*68}╗{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}║{'🚁 REAL-TIME TELEMETRY MONITOR':^68}║{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}╚{'═'*68}╝{Colors.RESET}")
        print(f"  {Colors.WHITE}[{now}]  Update #{self.update_count}  |  Elapsed: {elapsed:.0f}s  |  Press Ctrl+C to stop{Colors.RESET}")
        print()
        
        # === FLIGHT STATUS ===
        mode = self.vehicle.mode.name
        armed = self.vehicle.armed
        armed_str = f"{Colors.BG_RED}{Colors.WHITE} ARMED {Colors.RESET}" if armed else f"{Colors.BG_GREEN}{Colors.WHITE} DISARMED {Colors.RESET}"
        ekf_ok = self.vehicle.ekf_ok if hasattr(self.vehicle, 'ekf_ok') else None
        ekf_str = f"{Colors.GREEN}✓ OK{Colors.RESET}" if ekf_ok else f"{Colors.RED}✗ BAD{Colors.RESET}" if ekf_ok is not None else "N/A"
        
        print(f"{Colors.BOLD}┌─── ⚙️  FLIGHT STATUS {'─'*46}{Colors.RESET}")
        print(f"│  Mode: {Colors.CYAN}{Colors.BOLD}{mode:15}{Colors.RESET}  Armed: {armed_str}  EKF: {ekf_str}")
        print()
        
        # === GPS ===
        gps = self.vehicle.gps_0
        loc = self.vehicle.location.global_frame
        loc_rel = self.vehicle.location.global_relative_frame
        
        fix_str, fix_color = self.format_gps_fix(gps.fix_type if gps else 0)
        sats = gps.satellites_visible if gps else 0
        sats_color = Colors.GREEN if sats >= self.gps_min_sats else Colors.YELLOW if sats >= 4 else Colors.RED
        
        print(f"{Colors.BOLD}┌─── 📡 GPS {'─'*57}{Colors.RESET}")
        print(f"│  Fix: {fix_color}{fix_str:12}{Colors.RESET}  Satellites: {sats_color}{sats:2}{Colors.RESET}")
        
        if loc and loc.lat:
            print(f"│  {Colors.WHITE}Latitude : {Colors.YELLOW}{loc.lat:15.8f}°{Colors.RESET}")
            print(f"│  {Colors.WHITE}Longitude: {Colors.YELLOW}{loc.lon:15.8f}°{Colors.RESET}")
        else:
            print(f"│  {Colors.RED}Waiting for GPS fix...{Colors.RESET}")
        
        abs_alt = loc.alt if loc and loc.alt else 0
        rel_alt = loc_rel.alt if loc_rel and loc_rel.alt else 0
        print(f"│  {Colors.WHITE}Altitude : {Colors.CYAN}{abs_alt:10.2f} m{Colors.RESET} (MSL)  |  {Colors.CYAN}{rel_alt:10.2f} m{Colors.RESET} (Relative)")
        print()
        
        # === ATTITUDE ===
        att = self.vehicle.attitude
        print(f"{Colors.BOLD}┌─── 🎯 ATTITUDE {'─'*52}{Colors.RESET}")
        if att:
            roll = math.degrees(att.roll) if att.roll else 0
            pitch = math.degrees(att.pitch) if att.pitch else 0
            yaw = math.degrees(att.yaw) if att.yaw else 0
            
            # Color based on tilt angle
            roll_color = Colors.RED if abs(roll) > 30 else Colors.YELLOW if abs(roll) > 15 else Colors.GREEN
            pitch_color = Colors.RED if abs(pitch) > 30 else Colors.YELLOW if abs(pitch) > 15 else Colors.GREEN
            
            print(f"│  Roll:  {roll_color}{roll:+8.2f}°{Colors.RESET}   Pitch: {pitch_color}{pitch:+8.2f}°{Colors.RESET}   Yaw: {Colors.CYAN}{yaw:+8.2f}°{Colors.RESET} (Heading)")
        print()
        
        # === VELOCITY ===
        vel = self.vehicle.velocity
        print(f"{Colors.BOLD}┌─── 💨 VELOCITY {'─'*52}{Colors.RESET}")
        if vel:
            vx, vy, vz = vel[0] or 0, vel[1] or 0, vel[2] or 0
            ground_speed = math.sqrt(vx**2 + vy**2)
            total_speed = math.sqrt(vx**2 + vy**2 + vz**2)
            
            print(f"│  North: {Colors.WHITE}{vx:+7.2f}{Colors.RESET} m/s  East: {Colors.WHITE}{vy:+7.2f}{Colors.RESET} m/s  Down: {Colors.WHITE}{vz:+7.2f}{Colors.RESET} m/s")
            print(f"│  Ground Speed: {Colors.CYAN}{ground_speed:6.2f}{Colors.RESET} m/s  |  3D Speed: {Colors.CYAN}{total_speed:6.2f}{Colors.RESET} m/s")
        print()
        
        # === BATTERY ===
        batt = self.vehicle.battery
        voltage = batt.voltage if batt and batt.voltage else 0
        current = batt.current if batt and batt.current else 0
        level = batt.level if batt and batt.level else 0
        
        v_color = self.get_battery_color(voltage)
        v_percent = min(100, max(0, (voltage - 10.0) / (12.6 - 10.0) * 100)) if voltage > 0 else 0
        v_bar = self.draw_progress_bar(v_percent, 100, 25)
        
        print(f"{Colors.BOLD}┌─── 🔋 BATTERY {'─'*53}{Colors.RESET}")
        print(f"│  Voltage: {v_color}{voltage:6.2f} V{Colors.RESET}  [{v_bar}]")
        print(f"│  Current: {Colors.WHITE}{current:6.2f} A{Colors.RESET}   Level: {Colors.WHITE}{level:3}%{Colors.RESET}")
        print()
        
        # === EVENT LOG ===
        print(f"{Colors.BOLD}┌─── 📋 EVENT LOG (Errors, Warnings, Mode Changes) {'─'*18}{Colors.RESET}")
        if self.event_log:
            for event in list(self.event_log)[-8:]:  # Show last 8 events
                print(f"│  {Colors.WHITE}[{event['time']}]{Colors.RESET} {event['color']}{event['level']:8}{Colors.RESET} {event['message']}")
        else:
            print(f"│  {Colors.WHITE}No events yet...{Colors.RESET}")
        print()
        
        # === MODE HISTORY ===
        if self.mode_history:
            print(f"{Colors.BOLD}┌─── 🔄 MODE HISTORY {'─'*48}{Colors.RESET}")
            for change in list(self.mode_history)[-5:]:
                print(f"│  [{change['time']}] {Colors.YELLOW}{change['from']}{Colors.RESET} → {Colors.CYAN}{change['to']}{Colors.RESET}")
            print()
        
        print(f"{'─'*70}")
        print(f"  {Colors.WHITE}Press Ctrl+C to stop{Colors.RESET}")
    
    def run(self):
        """Main monitoring loop."""
        if not self.connect():
            return
        
        self.running = True
        self.start_time = time.time()
        self.log_event("INFO", "Monitor started")
        self.log_event("MODE", f"Initial mode: {self.vehicle.mode.name}")
        
        # Initial alert check with delay
        time.sleep(1)
        
        try:
            while self.running:
                self.update_count += 1
                
                # Check for alerts every 10 updates
                if self.update_count % 10 == 0:
                    self.check_alerts()
                
                self.display()
                time.sleep(0.5)  # 2 Hz update rate
                
        except KeyboardInterrupt:
            print(f"\n\n{Colors.YELLOW}Stopped by user{Colors.RESET}")
        finally:
            self.running = False
            if self.vehicle:
                self.vehicle.close()
            print(f"{Colors.GREEN}✓ Disconnected{Colors.RESET}")


def main():
    parser = argparse.ArgumentParser(
        description='Real-Time Telemetry Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 live_monitor.py                          # Default USB connection
  python3 live_monitor.py --port /dev/ttyACM0      # Pixhawk via USB
  python3 live_monitor.py --port /dev/ttyUSB0 --baud 57600  # 3DR radio
        """
    )
    parser.add_argument('--port', default='/dev/ttyACM0', 
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, 
                        help='Baud rate (default: 115200, use 57600 for 3DR radio)')
    args = parser.parse_args()
    
    monitor = TelemetryMonitor(args.port, args.baud)
    monitor.run()


if __name__ == "__main__":
    main()
