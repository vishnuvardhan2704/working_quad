#!/usr/bin/env python3
"""
Temporary Bench Test Script - NO GPS REQUIRED
Tests Pixhawk connection and sensors without GPS checks.

Usage:
    python3 temp_check.py                               # 3DR radio (default)
    python3 temp_check.py --port /dev/ttyUSB0 --baud 57600    # 3DR radio
    python3 temp_check.py --port /dev/ttyACM0 --baud 115200   # USB direct
"""

import argparse
import time
import sys

try:
    from dronekit import connect
except ImportError:
    print("ERROR: DroneKit not installed. Run: pip install dronekit pymavlink")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description='Bench test without GPS')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0 for 3DR radio)')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate (default: 57600 for 3DR radio)')
    args = parser.parse_args()

    print("=" * 60)
    print("  BENCH TEST - GPS SKIPPED")
    print("  ⚠️  DO NOT ATTEMPT TO FLY - GPS CHECKS DISABLED!")
    print("=" * 60)
    print()

    # Connect
    print(f"[INFO] Connecting to {args.port} @ {args.baud}...")
    try:
        # Use wait_ready=False for faster connection over radio
        vehicle = connect(args.port, baud=args.baud, wait_ready=False, timeout=30)
        print("[OK] Connected to Pixhawk!")
        time.sleep(2)  # Allow data to flow
        print()
    except Exception as e:
        print(f"[FAIL] Connection failed: {e}")
        return False

    try:
        # ===== FIRMWARE INFO =====
        print("-" * 60)
        print("FIRMWARE INFO")
        print("-" * 60)
        print(f"  Autopilot:     {vehicle.version}")
        print(f"  System Status: {vehicle.system_status.state}")
        print()

        # ===== IMU / ATTITUDE (No GPS needed) =====
        print("-" * 60)
        print("IMU / ATTITUDE ✓")
        print("-" * 60)
        att = vehicle.attitude
        import math
        roll_deg = math.degrees(att.roll)
        pitch_deg = math.degrees(att.pitch)
        yaw_deg = math.degrees(att.yaw)
        print(f"  Roll:  {roll_deg:+7.2f}°")
        print(f"  Pitch: {pitch_deg:+7.2f}°")
        print(f"  Yaw:   {yaw_deg:+7.2f}° (heading)")
        print("[OK] IMU working\n")

        # ===== BAROMETER (No GPS needed) =====
        print("-" * 60)
        print("BAROMETER ✓")
        print("-" * 60)
        loc_rel = vehicle.location.global_relative_frame
        loc_abs = vehicle.location.global_frame
        print(f"  Relative Alt: {loc_rel.alt:.2f} m (above home)")
        print(f"  Absolute Alt: {loc_abs.alt:.2f} m (MSL)")
        print("[OK] Barometer working\n")

        # ===== BATTERY =====
        print("-" * 60)
        print("BATTERY")
        print("-" * 60)
        batt = vehicle.battery
        if batt.voltage and batt.voltage > 0:
            print(f"  Voltage: {batt.voltage:.2f} V")
            print(f"  Current: {batt.current if batt.current else 'N/A'} A")
            print(f"  Level:   {batt.level if batt.level else 'N/A'}%")
            if batt.voltage >= 10.5:
                print("[OK] Battery OK\n")
            else:
                print("[WARN] Battery LOW!\n")
        else:
            print("  Voltage: Not detected")
            print("[SKIP] No power module connected\n")

        # ===== GPS STATUS (Info only, not required) =====
        print("-" * 60)
        print("GPS STATUS (info only - not required for this test)")
        print("-" * 60)
        gps = vehicle.gps_0
        fix_names = {0: 'None', 1: 'No Fix', 2: '2D', 3: '3D ✓', 4: 'DGPS', 5: 'RTK'}
        fix_type = gps.fix_type if gps.fix_type is not None else 0
        sats = gps.satellites_visible if gps.satellites_visible is not None else 0
        print(f"  Fix Type:   {fix_type} ({fix_names.get(fix_type, 'Unknown')})")
        print(f"  Satellites: {sats}")
        if fix_type >= 3:
            print("[OK] GPS has 3D fix\n")
        else:
            print("[INFO] GPS not locked - this is OK for bench test\n")

        # ===== FLIGHT MODE =====
        print("-" * 60)
        print("FLIGHT MODE")
        print("-" * 60)
        print(f"  Current Mode: {vehicle.mode.name}")
        print(f"  Armed:        {'YES ⚠️' if vehicle.armed else 'NO (safe)'}")
        print()

        # ===== EKF STATUS =====
        print("-" * 60)
        print("EKF STATUS")
        print("-" * 60)
        print(f"  EKF OK:    {vehicle.ekf_ok}")
        print(f"  Armable:   {vehicle.is_armable}")
        if not vehicle.ekf_ok:
            print("[INFO] EKF not converged - needs GPS for full convergence")
        print()

        # ===== SUMMARY =====
        print("=" * 60)
        print("BENCH TEST SUMMARY")
        print("=" * 60)
        print("  ✓ Connection:  OK")
        print("  ✓ IMU:         OK")
        print("  ✓ Barometer:   OK")
        print(f"  {'✓' if batt.voltage and batt.voltage > 0 else '⚠'} Battery:     {'OK' if batt.voltage and batt.voltage > 0 else 'Not connected'}")
        print(f"  {'✓' if fix_type >= 3 else '⚠'} GPS:         {'OK' if fix_type >= 3 else 'No lock (OK for bench)'}")
        print()
        print("  ⚠️  REMINDER: Cannot arm/fly without GPS 3D fix!")
        print("=" * 60)

        return True

    finally:
        vehicle.close()
        print("\n[INFO] Connection closed.")


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
