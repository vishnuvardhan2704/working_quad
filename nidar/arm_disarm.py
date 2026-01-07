#!/usr/bin/env python3
"""
Simple Arm/Disarm Test Script with Pre-flight Checks

Tests arming and disarming the vehicle via USB or 3DR radio.
WARNING: Ensure propellers are REMOVED before testing!

Usage:
    python3 arm_disarm.py              # Preflight, arm, wait 10s, disarm
    python3 arm_disarm.py --arm-only   # Just arm (stay armed)
    python3 arm_disarm.py --disarm     # Just disarm
    python3 arm_disarm.py --skip-gps   # Skip GPS checks (bench test only)
"""

import argparse
import time
import sys
import glob

try:
    from dronekit import connect, VehicleMode
except ImportError:
    print("ERROR: DroneKit not installed. Run: pip install dronekit")
    sys.exit(1)


def find_pixhawk_port():
    """Auto-detect Pixhawk port."""
    # Check common Pixhawk USB ports
    ports_to_check = [
        '/dev/ttyACM0',   # Pixhawk USB (most common)
        '/dev/ttyACM1',
        '/dev/ttyUSB0',   # USB-to-serial / 3DR radio
        '/dev/ttyUSB1',
        '/dev/serial0',   # Raspberry Pi GPIO UART
    ]
    
    for port in ports_to_check:
        try:
            import os
            if os.path.exists(port):
                return port
        except:
            pass
    
    # Try glob patterns
    acm_ports = glob.glob('/dev/ttyACM*')
    if acm_ports:
        return acm_ports[0]
    
    usb_ports = glob.glob('/dev/ttyUSB*')
    if usb_ports:
        return usb_ports[0]
    
    return None


def run_preflight_checks(vehicle, skip_gps=False):
    """
    Run pre-flight safety checks.
    
    Args:
        vehicle: DroneKit vehicle object
        skip_gps: Skip GPS checks for bench testing
        
    Returns:
        True if all checks pass
    """
    print("\n" + "=" * 50)
    print("  PRE-FLIGHT CHECKS")
    print("=" * 50)
    
    all_passed = True
    
    # 1. GPS Check
    print("\n[CHECK 2] GPS...")
    if skip_gps:
        print("  ⚠ SKIPPED (--skip-gps mode)")
    else:
        gps = vehicle.gps_0
        fix_type = gps.fix_type if gps else 0
        sats = gps.satellites_visible if gps else 0
        
        if fix_type < 3:
            print(f"  ⚠ WARN: GPS fix type {fix_type} (need 3 for 3D fix)")
            print(f"         Satellites: {sats}")
            print("         Waiting for GPS lock...")
            
            # Wait up to 60 seconds for GPS
            for i in range(12):
                time.sleep(5)
                gps = vehicle.gps_0
                fix_type = gps.fix_type if gps else 0
                sats = gps.satellites_visible if gps else 0
                print(f"         GPS: fix={fix_type}, sats={sats}")
                if fix_type >= 3:
                    break
            
            if fix_type < 3:
                print(f"  ✗ FAIL: No GPS 3D fix after waiting")
                all_passed = False
            else:
                print(f"  ✓ OK: GPS 3D fix acquired (sats: {sats})")
        else:
            print(f"  ✓ OK: GPS fix type {fix_type}, satellites {sats}")
    
    # 3. EKF Check
    print("\n[CHECK 3] EKF Status...")
    if vehicle.ekf_ok:
        print("  ✓ OK: EKF converged")
    else:
        print("  ⚠ WARN: EKF not converged, waiting...")
        for i in range(10):
            time.sleep(2)
            if vehicle.ekf_ok:
                print("  ✓ OK: EKF converged")
                break
        if not vehicle.ekf_ok:
            print("  ⚠ WARN: EKF still not converged (may affect arming)")
    
    # 4. Armable Check
    print("\n[CHECK 4] Armable Status...")
    if vehicle.is_armable:
        print("  ✓ OK: Vehicle is armable")
    else:
        print("  ⚠ WARN: Vehicle not armable, waiting...")
        for i in range(10):
            time.sleep(2)
            if vehicle.is_armable:
                print("  ✓ OK: Vehicle is now armable")
                break
        if not vehicle.is_armable:
            print("  ⚠ WARN: Vehicle still not armable")
            print("         Will attempt to arm anyway...")
    
    print("\n" + "-" * 50)
    if all_passed:
        print("  ✓ ALL PRE-FLIGHT CHECKS PASSED")
    else:
        print("  ⚠ SOME CHECKS FAILED - Proceeding with caution")
    print("-" * 50)
    
    return all_passed


def arm_vehicle(vehicle, timeout=15, skip_gps=False):
    """
    Arm the vehicle.
    
    Args:
        vehicle: DroneKit vehicle object
        timeout: Maximum time to wait for arming
        skip_gps: If True, use STABILIZE mode (no GPS required)
        
    Returns:
        True if armed successfully
    """
    # Choose mode based on GPS availability
    if skip_gps:
        target_mode = "STABILIZE"  # No GPS required
        print(f"\n[ACTION] Setting mode to {target_mode} (no GPS required)...")
    else:
        target_mode = "GUIDED"
        print(f"\n[ACTION] Setting mode to {target_mode}...")
    
    vehicle.mode = VehicleMode(target_mode)
    
    # Wait for mode change
    start = time.time()
    while vehicle.mode.name != target_mode:
        if time.time() - start > 10:
            print("[WARN] Mode change timeout, trying to arm anyway...")
            break
        time.sleep(0.5)
    print(f"[OK] Mode: {vehicle.mode.name}")
    
    print("[ACTION] Arming motors...")
    vehicle.armed = True
    
    # Wait for arming
    start = time.time()
    while not vehicle.armed:
        if time.time() - start > timeout:
            print("[FAIL] Arming timeout!")
            print("[INFO] Common reasons:")
            print("       - Pre-arm checks not passed")
            print("       - GPS not locked")
            print("       - Safety switch not pressed")
            return False
        print(f"  Waiting for arm... ({int(time.time() - start)}s)", end='\r')
        time.sleep(0.5)
    
    print("\n[OK] ✓ ARMED!")
    print("[WARN] ⚠️  MOTORS ARE LIVE!")
    return True


def disarm_vehicle(vehicle, timeout=10):
    """
    Disarm the vehicle.
    
    Args:
        vehicle: DroneKit vehicle object
        timeout: Maximum time to wait for disarming
        
    Returns:
        True if disarmed successfully
    """
    print("[ACTION] Disarming...")
    vehicle.armed = False
    
    start = time.time()
    while vehicle.armed:
        if time.time() - start > timeout:
            print("[WARN] Disarm timeout - forcing...")
            vehicle.armed = False
            time.sleep(1)
            break
        time.sleep(0.5)
    
    print("[OK] ✓ DISARMED")
    return True


def main():
    parser = argparse.ArgumentParser(description='Arm/Disarm Test with Preflight Checks')
    parser.add_argument('--port', default=None, help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--arm-only', action='store_true', help='Only arm, do not disarm')
    parser.add_argument('--disarm', action='store_true', help='Only disarm')
    parser.add_argument('--skip-gps', action='store_true', help='Skip GPS checks (bench test only)')
    parser.add_argument('--wait-time', type=int, default=10, help='Seconds to wait after arming (default: 10)')
    args = parser.parse_args()

    print("=" * 50)
    print("  ARM/DISARM TEST WITH PRE-FLIGHT CHECKS")
    print("  ⚠️  REMOVE PROPELLERS BEFORE TESTING!")
    print("=" * 50)
    print()

    # Auto-detect port if not specified
    port = args.port
    if port is None:
        print("[INFO] Auto-detecting Pixhawk port...")
        port = find_pixhawk_port()
        if port is None:
            print("[ERROR] No Pixhawk found! Check USB connection.")
            print("[INFO] Available ports:")
            for p in glob.glob('/dev/tty*'):
                if 'ACM' in p or 'USB' in p:
                    print(f"       {p}")
            return False
        print(f"[INFO] Found Pixhawk on: {port}")

    # Connect
    print(f"\n[INFO] Connecting to {port} @ {args.baud}...")
    vehicle = None
    
    try:
        vehicle = connect(port, baud=args.baud, wait_ready=False, timeout=60)
        print("[OK] Connected!")
        
        # Wait for vehicle data
        print("[INFO] Waiting for vehicle data...")
        time.sleep(3)
        
        # Show current status
        print(f"\n[STATUS] Firmware: {vehicle.version}")
        print(f"[STATUS] Mode: {vehicle.mode.name}")
        print(f"[STATUS] Armed: {vehicle.armed}")
        
        gps = vehicle.gps_0
        if gps:
            print(f"[STATUS] GPS: Fix={gps.fix_type}, Sats={gps.satellites_visible}")
        
        battery = vehicle.battery
        if battery and battery.voltage:
            print(f"[STATUS] Battery: {battery.voltage:.2f}V")

        # Disarm only mode
        if args.disarm:
            if vehicle.armed:
                disarm_vehicle(vehicle)
            else:
                print("[INFO] Already disarmed")
            return True

        # Run pre-flight checks
        preflight_passed = run_preflight_checks(vehicle, skip_gps=args.skip_gps)
        
        if not preflight_passed:
            response = input("\n[?] Pre-flight checks failed. Continue anyway? [y/N]: ")
            if response.lower() != 'y':
                print("[ABORT] Arming cancelled")
                return False

        # Arm the vehicle
        if not arm_vehicle(vehicle, skip_gps=args.skip_gps):
            return False
        
        if args.arm_only:
            print("\n[INFO] Staying armed (--arm-only mode)")
            print("[INFO] Run 'python3 arm_disarm.py --disarm' to disarm")
            return True

        # Wait
        wait_time = args.wait_time
        print(f"\n[INFO] Waiting {wait_time} seconds...")
        for i in range(wait_time, 0, -1):
            print(f"  Disarming in {i}...", end='\r')
            time.sleep(1)
        print()
        
        # Disarm
        disarm_vehicle(vehicle)
        
        print("\n[SUCCESS] Arm/Disarm test complete!")
        return True

    except KeyboardInterrupt:
        print("\n\n[ABORT] Interrupted!")
        if vehicle and vehicle.armed:
            print("[ACTION] Emergency disarm...")
            vehicle.armed = False
            time.sleep(1)
        return False
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        print("[DEBUG] Troubleshooting tips:")
        print("        - Check Pixhawk is powered and connected")
        print("        - Try: ls /dev/ttyACM* /dev/ttyUSB*")
        print("        - Ensure user is in dialout group: sudo usermod -a -G dialout $USER")
        return False
    
    finally:
        if vehicle:
            vehicle.close()
            print("[INFO] Connection closed")


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
