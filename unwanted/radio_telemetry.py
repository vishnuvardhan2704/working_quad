#!/usr/bin/env python3
"""
Live Telemetry Monitor via 3DR Radio

Displays real-time telemetry from Pixhawk over 3DR telemetry radio.

Usage:
    python3 radio_telemetry.py                    # Default: /dev/ttyUSB0 @ 57600
    python3 radio_telemetry.py --port /dev/ttyUSB0
"""

import argparse
import math
import sys
import time


def clear_screen():
    """Clear terminal screen."""
    print("\033[2J\033[H", end="")


def main():
    parser = argparse.ArgumentParser(description='Live telemetry via 3DR radio')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Radio serial port')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate (default: 57600)')
    args = parser.parse_args()

    print("=" * 65)
    print("   3DR RADIO TELEMETRY - Press Ctrl+C to stop")
    print("=" * 65)
    print(f"Connecting to {args.port} @ {args.baud}...")

    try:
        from dronekit import connect
    except ImportError:
        print("ERROR: DroneKit not installed")
        sys.exit(1)

    try:
        # Connect without wait_ready for faster connection over radio
        vehicle = connect(args.port, baud=args.baud, wait_ready=False, timeout=30)
        print("✓ Connected via 3DR radio!")
        time.sleep(2)  # Let data flow
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        sys.exit(1)

    update_count = 0
    try:
        while True:
            update_count += 1
            clear_screen()

            print("╔" + "═" * 63 + "╗")
            print("║" + "   🛰️  3DR RADIO TELEMETRY MONITOR  🛰️".center(63) + "║")
            print("║" + f"        Press Ctrl+C to stop".center(63) + "║")
            print("╚" + "═" * 63 + "╝")
            print(f"  [{time.strftime('%H:%M:%S')}]  Update #{update_count}")
            print()

            # GPS
            print("┌─── 📡 GPS " + "─" * 52)
            gps = vehicle.gps_0
            fix_names = {0: 'None', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix ✓', 4: 'DGPS', 5: 'RTK'}
            fix_str = fix_names.get(gps.fix_type, f'Type {gps.fix_type}')
            print(f"│ Fix: {fix_str:<14} Satellites: {gps.satellites_visible}")
            
            loc = vehicle.location.global_frame
            if loc and loc.lat != 0:
                print(f"│ Lat: {loc.lat:>14.8f}°")
                print(f"│ Lon: {loc.lon:>14.8f}°")
            else:
                print(f"│ Lat: {'--':>14}")
                print(f"│ Lon: {'--':>14}")
            
            loc_rel = vehicle.location.global_relative_frame
            abs_alt = loc.alt if loc and loc.alt else 0
            rel_alt = loc_rel.alt if loc_rel and loc_rel.alt else 0
            print(f"│ Alt: {abs_alt:>10.2f} m (MSL)")
            print(f"│ Rel: {rel_alt:>10.2f} m (above home)")
            print()

            # IMU
            print("┌─── 🎯 IMU / ATTITUDE " + "─" * 41)
            att = vehicle.attitude
            if att:
                print(f"│ Roll:  {math.degrees(att.roll):>+8.2f}°")
                print(f"│ Pitch: {math.degrees(att.pitch):>+8.2f}°")
                print(f"│ Yaw:   {math.degrees(att.yaw):>+8.2f}° (heading)")
            print()

            # Velocity
            print("┌─── 💨 VELOCITY " + "─" * 47)
            vel = vehicle.velocity
            if vel:
                print(f"│ Vx (N): {vel[0]:>+7.2f} m/s")
                print(f"│ Vy (E): {vel[1]:>+7.2f} m/s")
                print(f"│ Vz (D): {vel[2]:>+7.2f} m/s")
                ground_speed = math.sqrt(vel[0]**2 + vel[1]**2)
                ground_speed = math.sqrt(vel[0]**2 + vel[1]**2)
                print(f"│ Ground: {ground_speed:>7.2f} m/s")
            print()

            # Battery
            print("┌─── 🔋 BATTERY " + "─" * 48)
            batt = vehicle.battery
            voltage = batt.voltage if batt.voltage else 0
            current = batt.current if batt.current else 0
            level = batt.level if batt.level else 0
            print(f"│ Voltage: {voltage:>6.2f} V")
            print(f"│ Current: {current:>6.2f} A")
            print(f"│ Level:   {level:>6}%")
            print()

            # Status
            print("┌─── ⚙️  STATUS " + "─" * 48)
            print(f"│ Mode:    {vehicle.mode.name}")
            armed_str = "🔴 ARMED" if vehicle.armed else "🟢 DISARMED"
            print(f"│ State:   {armed_str}")
            print(f"│ EKF OK:  {vehicle.ekf_ok}")
            print(f"│ Armable: {vehicle.is_armable}")
            print()

            # Radio quality indicator
            print("┌─── 📻 RADIO " + "─" * 50)
            print(f"│ Link:    Active (3DR @ {args.baud} baud)")
            print()

            print("─" * 65)
            print("  Press Ctrl+C to stop")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
    finally:
        vehicle.close()
        print("✓ Disconnected")


if __name__ == "__main__":
    main()
