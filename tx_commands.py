#!/usr/bin/env python3
"""
Drone Command Sender via LoRa/3DR Radio

Send commands to drone from laptop via 3DR radio.
This script runs on the REMOTE laptop (your friend's computer).

Commands:
    ARM           - Arm the drone
    DISARM        - Disarm the drone
    TAKEOFF:5     - Takeoff to 5 meters
    LAND          - Land the drone
    RTL           - Return to launch
    MODE:STABILIZE - Change to stabilize mode
    MODE:LOITER   - Change to loiter mode
    MODE:GUIDED   - Change to guided mode
    GOTO:lat,lon,alt - Go to GPS location
    MOVE:10,0,0   - Move 10m north
    STOP          - Stop and hover
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
        print("  DRONE COMMAND SENDER")
        print("=" * 50)
        print("\nAvailable commands:")
        print("  ARM, DISARM, TAKEOFF:5, LAND, RTL")
        print("  MODE:STABILIZE, MODE:LOITER, MODE:GUIDED")
        print("  GOTO:lat,lon,alt, MOVE:n,e,d, STOP")
        print("  STATUS, PING, QUIT")
        print("\n" + "-" * 50)
        
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
  ARM            - Arm the drone motors
  DISARM         - Disarm the drone motors
  TAKEOFF:5      - Takeoff to 5 meters altitude
  LAND           - Land the drone
  RTL            - Return to launch point
  
  MODE:STABILIZE - Manual stabilize mode
  MODE:LOITER    - GPS hold/loiter mode
  MODE:GUIDED    - Guided/autonomous mode
  MODE:ALT_HOLD  - Altitude hold mode
  
  GOTO:lat,lon,alt - Fly to GPS coordinates
  MOVE:10,0,0      - Move 10m north (N,E,Down)
  STOP             - Stop and hover
  
  STATUS         - Get drone status
  PING           - Test connection
  
  QUIT           - Exit program
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
