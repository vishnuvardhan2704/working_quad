#!/usr/bin/env python3
"""
Drone Command Sender via LoRa/3DR Radio

Send commands to drone from laptop via 3DR radio.
This script runs on the REMOTE laptop (ground station).

Commands:
    ARM           - Arm the drone
    DISARM        - Disarm the drone
    TAKEOFF:5     - Takeoff to 5 meters
    LAND          - Land the drone
    RTL           - Return to launch
    SCOUT         - Start detection + recording (auto)
    SCOUT:STOP    - Stop detection, recording, mission
    MODE:STABILIZE - Change to stabilize mode
    MODE:LOITER   - Change to loiter mode
    MODE:GUIDED   - Change to guided mode
    GOTO:lat,lon,alt - Go to GPS location
    STATUS        - Get drone status
    PING          - Test connection
    
Usage:
    python3 tx_commands.py
    python3 tx_commands.py --port COM3      # Windows
    python3 tx_commands.py --port /dev/ttyUSB0  # Linux
"""

import sys
import time
import argparse
import serial
import threading


class DroneCommandSender:
    """Sends drone commands via LoRa/3DR radio."""
    
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.serial = None
        self.running = False
        self.receive_thread = None
        
    def connect(self):
        """Connect to 3DR radio."""
        print(f"[INFO] Connecting to radio on {self.port} @ {self.baud}...")
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1
            )
            print(f"[OK] Connected to radio")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect: {e}")
            return False
    
    def receive_responses(self):
        """Background thread to receive responses from drone."""
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        # Format based on message type
                        if "HUMAN DETECTED" in data:
                            print(f"\n  🚨 {data}")
                        elif "DETECTION:" in data:
                            print(f"\n  📷 {data}")
                        elif "MISSION" in data:
                            print(f"\n  🛫 {data}")
                        # Telemetry messages from drone
                        elif "[TELEM]" in data:
                            # Color-code telemetry by severity
                            if "[ERROR]" in data or "[CRIT]" in data or "[EMERG]" in data:
                                print(f"\n  ❌ \033[91m{data}\033[0m")  # Red
                            elif "[WARN]" in data:
                                print(f"\n  ⚠️  \033[93m{data}\033[0m")  # Yellow
                            elif "[NOTICE]" in data:
                                print(f"\n  📢 \033[94m{data}\033[0m")  # Blue
                            else:
                                print(f"\n  📡 \033[96m{data}\033[0m")  # Cyan
                        else:
                            print(f"\n  >> {data}")
                        print("CMD> ", end='', flush=True)
            except:
                pass
            time.sleep(0.05)
    
    def send_command(self, cmd):
        """Send a command to the drone."""
        try:
            cmd = cmd.strip()
            if not cmd:
                return
            
            # Send command with newline
            self.serial.write(f"{cmd}\n".encode())
            print(f"[TX] Sent: {cmd}")
            
        except Exception as e:
            print(f"[ERROR] Failed to send: {e}")
    
    def run_interactive(self):
        """Run interactive command mode."""
        print("\n" + "=" * 50)
        print("  DRONE SCOUT - Ground Station")
        print("=" * 50)
        print("\nQuick start (SCOUT auto-starts detection + recording):")
        print("  1. SCOUT              <- Just this for ground test!")
        print("  2. SCOUT:STOP         <- Stop and save video")
        print("\nFull flight sequence:")
        print("  1. ARM")
        print("  2. TAKEOFF:10")
        print("  3. SCOUT              <- Auto-starts detection + recording")
        print("  4. RTL")
        print("  5. SCOUT:STOP")
        print("\nWith waypoints:")
        print("  WP:lat,lon,alt -> SCOUT")
        print("\nTelemetry: Real-time logs appear with 📡 prefix")
        print("Type HELP for all commands")
        print("-" * 50)
        
        self.running = True
        
        # Start receive thread
        self.receive_thread = threading.Thread(target=self.receive_responses, daemon=True)
        self.receive_thread.start()
        
        try:
            while self.running:
                try:
                    cmd = input("CMD> ").strip().upper()
                    
                    if cmd == "QUIT" or cmd == "EXIT" or cmd == "Q":
                        print("Exiting...")
                        break
                    
                    if cmd == "HELP" or cmd == "?":
                        self.show_help()
                        continue
                    
                    if cmd:
                        self.send_command(cmd)
                        time.sleep(0.1)  # Small delay for response
                        
                except EOFError:
                    break
                    
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        
        self.running = False
        if self.serial:
            self.serial.close()
        print("[INFO] Disconnected")
    
    def show_help(self):
        """Show help message."""
        print("""
Commands:
  === QUICK START (Ground Test) ===
  SCOUT            - Start detection + recording (auto!)
  SCOUT:STOP       - Stop everything and save video
  
  === FULL FLIGHT SEQUENCE ===
  1. ARM              - Arm the drone
  2. TAKEOFF:10       - Takeoff to 10 meters
  3. SCOUT            - Auto-starts detection + recording
  4. RTL              - Return home after scouting
  5. SCOUT:STOP       - Stop and save recording
  
  === WITH WAYPOINTS (optional) ===
  WP:lat,lon,alt   - Add waypoint BEFORE SCOUT
  WP:CLEAR         - Clear all waypoints
  WP:LIST          - Show all waypoints
  ALT:15           - Set default altitude (meters)
  Then: SCOUT      - Flies waypoints with detection
  
  === FLIGHT COMMANDS ===
  ARM              - Arm motors
  DISARM           - Disarm motors
  TAKEOFF:10       - Takeoff to 10 meters
  LAND             - Land immediately
  RTL              - Return to launch
  GOTO:lat,lon,alt - Go to single location
  ABORT            - Emergency stop
  
  === DETECTION (manual control) ===
  DETECT:START     - Start camera + detection (no recording)
  DETECT:STOP      - Stop detection
  DETECT:STATUS    - Get detection count + recording status
  DETECT:CONF:0.7  - Set confidence (0.1-1.0)
  
  === OTHER ===
  STATUS           - Get drone status
  PING             - Test connection
  MODE:GUIDED      - Set guided mode
  QUIT             - Exit program

  === TELEMETRY ===
  Real-time logs from drone appear automatically:
  📡 [TELEM][INFO]   - Informational (cyan)
  📢 [TELEM][NOTICE] - Mode changes, arm/disarm (blue)
  ⚠️  [TELEM][WARN]   - Warnings (yellow)
  ❌ [TELEM][ERROR]  - Errors (red)

  === EXAMPLE: GROUND TEST ===
  CMD> SCOUT
  ... Detection running, video recording...
  ... 🚨 HUMAN DETECTED alerts appear ...
  CMD> SCOUT:STOP
  ... Video saved to recordings/scout_YYYYMMDD_HHMMSS.mp4

  === EXAMPLE: FULL FLIGHT ===
  CMD> ARM
  CMD> TAKEOFF:10
  CMD> WP:12.9716,77.5946,10
  CMD> WP:12.9720,77.5950,10
  CMD> SCOUT
  ... Flies to waypoints, detects humans, records video...
  CMD> RTL
  CMD> SCOUT:STOP
""")


def main():
    parser = argparse.ArgumentParser(description='Drone Command Sender via LoRa/3DR Radio')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0, Windows: COM3)')
    parser.add_argument('--baud', type=int, default=57600,
                       help='Baud rate (default: 57600)')
    args = parser.parse_args()
    
    sender = DroneCommandSender(port=args.port, baud=args.baud)
    
    if not sender.connect():
        sys.exit(1)
    
    sender.run_interactive()


if __name__ == "__main__":
    main()
