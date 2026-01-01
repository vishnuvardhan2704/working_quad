#!/usr/bin/env python3
"""
Live Telemetry Monitor for Pixhawk
===================================
Continuously displays GPS, IMU, Barometer, and Power readings.
Press 'q' to quit.

Usage:
    python telemetry_monitor.py
    python telemetry_monitor.py --port /dev/ttyACM0 --baud 115200
"""

import sys
import time
import threading
import math

# Global flag to stop monitoring
stop_flag = False

def clear_screen():
    """Clear terminal screen."""
    print("\033[2J\033[H", end="")

def key_listener():
    """Listen for 'q' key to quit."""
    global stop_flag
    try:
        import termios
        import tty
        import select
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not stop_flag:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch.lower() == 'q':
                        stop_flag = True
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    except Exception:
        # Fallback for non-tty environments
        pass

def format_gps_fix(fix_type):
    """Convert GPS fix type to readable string."""
    fix_names = {
        0: "No GPS",
        1: "No Fix",
        2: "2D Fix",
        3: "3D Fix",
        4: "DGPS",
        5: "RTK Float",
        6: "RTK Fixed"
    }
    return fix_names.get(fix_type, f"Unknown({fix_type})")

def main():
    global stop_flag
    
    import argparse
    parser = argparse.ArgumentParser(description='Live Telemetry Monitor')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("   PIXHAWK LIVE TELEMETRY MONITOR")
    print("="*60)
    print(f"\nConnecting to {args.port} @ {args.baud} baud...")
    
    try:
        from dronekit import connect
        vehicle = connect(args.port, baud=args.baud, wait_ready=True, timeout=30)
        print("✓ Connected!")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    # Start key listener thread
    key_thread = threading.Thread(target=key_listener, daemon=True)
    key_thread.start()
    
    print("\nStarting telemetry stream...")
    print("Press 'q' to quit\n")
    time.sleep(1)
    
    update_count = 0
    start_time = time.time()
    
    try:
        while not stop_flag:
            update_count += 1
            elapsed = time.time() - start_time
            
            # Collect all telemetry data
            # GPS
            gps = vehicle.gps_0
            location = vehicle.location.global_frame
            location_rel = vehicle.location.global_relative_frame
            
            # Attitude (IMU)
            attitude = vehicle.attitude
            
            # Velocity
            velocity = vehicle.velocity
            
            # Battery/Power
            battery = vehicle.battery
            
            # Status
            mode = vehicle.mode.name
            armed = vehicle.armed
            
            # System status
            ekf_ok = vehicle.ekf_ok if hasattr(vehicle, 'ekf_ok') else 'N/A'
            
            # Build display
            clear_screen()
            
            print("╔════════════════════════════════════════════════════════════╗")
            print("║           PIXHAWK LIVE TELEMETRY MONITOR                   ║")
            print("║                   Press 'q' to quit                        ║")
            print("╚════════════════════════════════════════════════════════════╝")
            print(f"  Update #{update_count}  |  Elapsed: {elapsed:.1f}s  |  {time.strftime('%H:%M:%S')}")
            print()
            
            # GPS Section
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  📡 GPS                                                     │")
            print("├─────────────────────────────────────────────────────────────┤")
            if gps:
                print(f"│  Fix Type    : {format_gps_fix(gps.fix_type):15} Satellites: {gps.satellites_visible or 0:3}     │")
            else:
                print("│  Fix Type    : No GPS                                       │")
            
            if location:
                lat = location.lat if location.lat else 0
                lon = location.lon if location.lon else 0
                alt = location.alt if location.alt else 0
                print(f"│  Latitude    : {lat:15.7f}°                              │")
                print(f"│  Longitude   : {lon:15.7f}°                              │")
                print(f"│  Altitude    : {alt:10.2f} m (absolute)                    │")
            if location_rel:
                alt_rel = location_rel.alt if location_rel.alt else 0
                print(f"│  Rel Altitude: {alt_rel:10.2f} m (above home)                  │")
            print("└─────────────────────────────────────────────────────────────┘")
            print()
            
            # IMU Section
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  🎯 IMU / ATTITUDE                                          │")
            print("├─────────────────────────────────────────────────────────────┤")
            if attitude:
                roll = math.degrees(attitude.roll) if attitude.roll else 0
                pitch = math.degrees(attitude.pitch) if attitude.pitch else 0
                yaw = math.degrees(attitude.yaw) if attitude.yaw else 0
                print(f"│  Roll        : {roll:+8.2f}°                                   │")
                print(f"│  Pitch       : {pitch:+8.2f}°                                   │")
                print(f"│  Yaw         : {yaw:+8.2f}°  (Heading)                          │")
            else:
                print("│  No attitude data                                           │")
            print("└─────────────────────────────────────────────────────────────┘")
            print()
            
            # Velocity Section
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  💨 VELOCITY                                                │")
            print("├─────────────────────────────────────────────────────────────┤")
            if velocity:
                vx = velocity[0] if velocity[0] else 0
                vy = velocity[1] if velocity[1] else 0
                vz = velocity[2] if velocity[2] else 0
                ground_speed = math.sqrt(vx**2 + vy**2)
                print(f"│  North (Vx)  : {vx:+8.2f} m/s                                 │")
                print(f"│  East  (Vy)  : {vy:+8.2f} m/s                                 │")
                print(f"│  Down  (Vz)  : {vz:+8.2f} m/s                                 │")
                print(f"│  Ground Speed: {ground_speed:8.2f} m/s                                 │")
            else:
                print("│  No velocity data                                           │")
            print("└─────────────────────────────────────────────────────────────┘")
            print()
            
            # Power Section
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  🔋 POWER / BATTERY                                         │")
            print("├─────────────────────────────────────────────────────────────┤")
            if battery:
                voltage = battery.voltage if battery.voltage else 0
                current = battery.current if battery.current else 0
                level = battery.level if battery.level else 0
                
                # Voltage bar
                if voltage > 0:
                    v_percent = min(100, max(0, (voltage - 10.5) / (12.6 - 10.5) * 100))
                    bar_len = int(v_percent / 5)
                    v_bar = "█" * bar_len + "░" * (20 - bar_len)
                else:
                    v_bar = "░" * 20
                    v_percent = 0
                
                print(f"│  Voltage     : {voltage:8.2f} V   [{v_bar}]      │")
                print(f"│  Current     : {current:8.2f} A                                 │")
                print(f"│  Level       : {level:8}%                                  │")
            else:
                print("│  No battery data                                            │")
            print("└─────────────────────────────────────────────────────────────┘")
            print()
            
            # Status Section
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  ⚙️  SYSTEM STATUS                                           │")
            print("├─────────────────────────────────────────────────────────────┤")
            armed_str = "🔴 ARMED" if armed else "🟢 DISARMED"
            print(f"│  Mode        : {mode:15}                              │")
            print(f"│  Armed       : {armed_str:20}                         │")
            print(f"│  EKF OK      : {str(ekf_ok):15}                              │")
            print("└─────────────────────────────────────────────────────────────┘")
            print()
            
            # Barometer (altitude is from barometer)
            print("┌─────────────────────────────────────────────────────────────┐")
            print("│  🌡️  BAROMETER                                               │")
            print("├─────────────────────────────────────────────────────────────┤")
            if location_rel:
                baro_alt = location_rel.alt if location_rel.alt else 0
                print(f"│  Baro Alt    : {baro_alt:10.2f} m (relative)                    │")
            if location:
                abs_alt = location.alt if location.alt else 0
                print(f"│  Absolute Alt: {abs_alt:10.2f} m (MSL)                          │")
            print("└─────────────────────────────────────────────────────────────┘")
            
            print("\n" + "─"*62)
            print("  Press 'q' to quit")
            
            # Update rate ~2Hz
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by Ctrl+C")
    finally:
        stop_flag = True
        print("\nClosing connection...")
        vehicle.close()
        print("✓ Disconnected. Goodbye!")

if __name__ == "__main__":
    main()
