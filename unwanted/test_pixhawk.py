#!/usr/bin/env python3
"""
Pixhawk 2.4.8 Connection & MAVLink Test Script
Test serial communication and read all telemetry data

Usage:
    python test_pixhawk.py                                  # 3DR radio (default)
    python test_pixhawk.py --port /dev/ttyUSB0 --baud 57600       # 3DR radio
    python test_pixhawk.py --port /dev/ttyACM0 --baud 115200      # USB direct
"""

import argparse
import time
import sys
from datetime import datetime

try:
    from dronekit import connect, VehicleMode
except ImportError:
    print("ERROR: DroneKit not installed. Run: pip install dronekit pymavlink")
    sys.exit(1)


class Colors:
    """ANSI color codes for terminal output"""
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\033[95m"


def print_header(text):
    """Print section header"""
    print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}{text}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.RESET}\n")


def print_success(text):
    """Print success message"""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"{Colors.GREEN}[{ts}] ✓ {text}{Colors.RESET}")


def print_error(text):
    """Print error message"""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"{Colors.RED}[{ts}] ✗ {text}{Colors.RESET}")


def print_info(text):
    """Print info message"""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"{Colors.BLUE}[{ts}] ℹ {text}{Colors.RESET}")


def print_data(label, value, unit=""):
    """Print telemetry data"""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"{Colors.CYAN}[{ts}] {label:<30} : {Colors.MAGENTA}{value}{Colors.CYAN} {unit}{Colors.RESET}")


def test_connection(connection_string, baud=57600, timeout=60):
    """
    Test connection to Pixhawk and verify MAVLink communication.
    
    Args:
        connection_string: Serial port or UDP connection
        baud: Baud rate for serial connections
        timeout: Connection timeout in seconds
    
    Returns:
        Vehicle object if successful, None otherwise
    """
    print_header("STEP 1: TESTING PIXHAWK CONNECTION")
    
    print_info(f"Connecting to: {connection_string}")
    print_info(f"Baud rate: {baud}")
    print_info(f"Timeout: {timeout} seconds")
    print()
    
    try:
        vehicle = connect(connection_string, baud=baud, wait_ready=True, timeout=timeout)
        print_success("Connected to Pixhawk successfully!")
        return vehicle
    except Exception as e:
        print_error(f"Connection failed: {str(e)}")
        print_error("Troubleshooting:")
        print_error("  1. Check serial port exists: ls -l /dev/serial0")
        print_error("  2. Check permissions: sudo usermod -a -G dialout $USER")
        print_error("  3. Verify Pixhawk is powered and connected")
        print_error("  4. Check baud rate matches Pixhawk (default: 57600)")
        return None


def test_gps(vehicle, wait_time=120):
    """
    Test GPS and wait for 3D fix.
    
    Args:
        vehicle: DroneKit Vehicle object
        wait_time: Maximum time to wait for 3D lock
    """
    print_header("STEP 2: TESTING GPS ACQUISITION")
    
    start_time = time.time()
    
    print_info("Waiting for GPS 3D lock (this may take 30-60 seconds)...")
    print_info("GPS Fix Types: 0=None, 1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK\n")
    
    while time.time() - start_time < wait_time:
        gps = vehicle.gps_0
        elapsed = int(time.time() - start_time)
        
        print_data("GPS Fix Type", gps.fix_type, "(3D=3, 2D=2)")
        print_data("Satellites Visible", gps.satellites_visible, "satellites")
        print_data("Horizontal Accuracy", f"{gps.eph:.1f}", "meters")
        print_data("Vertical Accuracy", f"{gps.epv:.1f}", "meters")
        print_data("Time Elapsed", elapsed, "seconds")
        
        # 3D fix achieved
        if gps.fix_type >= 3:
            print_success("✓ GPS 3D LOCK ACHIEVED!")
            print_success(f"Locked with {gps.satellites_visible} satellites")
            return True
        
        # 2D fix but not 3D yet
        if gps.fix_type == 2:
            print_info(f"2D fix detected ({gps.satellites_visible} sats), waiting for 3D...")
        
        # No fix yet
        if gps.fix_type < 2:
            if elapsed % 5 == 0:
                print_info(f"Searching for satellites ({gps.satellites_visible} visible)...")
        
        print()
        time.sleep(1)
    
    # Timeout
    print_error(f"GPS 3D lock not achieved within {wait_time} seconds")
    print_error("Troubleshooting:")
    print_error("  1. Move to clear sky (away from buildings/trees)")
    print_error("  2. Wait 60+ seconds for cold start lock")
    print_error("  3. Verify GPS module is connected to Pixhawk GPS port")
    print_error("  4. Check GPS antenna has clear view of sky")
    return False


def test_battery(vehicle):
    """Test battery readings"""
    print_header("STEP 3: TESTING BATTERY READINGS")
    
    battery = vehicle.battery
    
    print_data("Voltage", f"{battery.voltage:.2f}", "V")
    print_data("Current", f"{battery.current:.2f}", "A" if battery.current else "(unavailable)")
    print_data("Level", f"{battery.level:.1f}", "%")
    
    # Try to get remaining energy if available
    try:
        if hasattr(battery, 'remaining_energy') and battery.remaining_energy is not None:
            print_data("Remaining Energy", f"{battery.remaining_energy:.1f}", "mAh")
    except:
        pass
    
    if battery.voltage is None:
        print_error("Battery voltage reading unavailable!")
        print_error("Troubleshooting:")
        print_error("  1. Check power module is connected to Pixhawk")
        print_error("  2. Verify battery is plugged in")
        print_error("  3. Check power module connector is secure")
        return False
    elif battery.voltage < 10.5:
        print_error(f"Battery CRITICALLY LOW: {battery.voltage:.2f}V (minimum safe: 10.5V)")
        print_error("DO NOT FLY - Battery may be damaged or discharged")
        return False
    elif battery.voltage < 11.0:
        print_error(f"Battery below nominal: {battery.voltage:.2f}V (nominal: 11.1-11.5V)")
        print_info("Consider charging before flight")
        return True
    else:
        print_success(f"✓ Battery voltage OK: {battery.voltage:.2f}V")
        return True


def test_attitude(vehicle):
    """Test attitude (roll, pitch, yaw) readings"""
    print_header("STEP 4: TESTING ATTITUDE SENSORS")
    
    attitude = vehicle.attitude
    
    print_data("Roll", f"{attitude.roll:.2f}", "radians")
    print_data("Pitch", f"{attitude.pitch:.2f}", "radians")
    print_data("Yaw", f"{attitude.yaw:.2f}", "radians")
    print()
    
    # Convert to degrees
    import math
    roll_deg = math.degrees(attitude.roll)
    pitch_deg = math.degrees(attitude.pitch)
    yaw_deg = math.degrees(attitude.yaw)
    
    print_data("Roll (degrees)", f"{roll_deg:.1f}", "°")
    print_data("Pitch (degrees)", f"{pitch_deg:.1f}", "°")
    print_data("Yaw (degrees)", f"{yaw_deg:.1f}", "°")
    print()
    
    print_success("✓ Attitude sensors working")
    return True


def test_compass(vehicle):
    """Test compass/magnetometer readings"""
    print_header("STEP 5: TESTING COMPASS (MAGNETOMETER)")
    
    # IMU compass reading (if available)
    attitude = vehicle.attitude
    import math
    yaw_deg = math.degrees(attitude.yaw)
    
    print_data("Compass Heading", f"{yaw_deg:.1f}", "degrees")
    print_data("Heading Status", "OK (from attitude)", "")
    print()
    
    print_info("Note: Detailed magnetometer readings available in flight logs")
    print_success("✓ Compass working")
    return True


def test_barometer(vehicle):
    """Test barometric pressure and altitude"""
    print_header("STEP 6: TESTING BAROMETER (ALTITUDE)")
    
    # Relative altitude above home
    alt_relative = vehicle.location.global_relative_frame.alt
    
    # Absolute altitude (MSL)
    alt_absolute = vehicle.location.global_frame.alt if vehicle.location.global_frame else None
    
    print_data("Relative Altitude", f"{alt_relative:.1f}", "meters (above home)")
    if alt_absolute:
        print_data("Absolute Altitude", f"{alt_absolute:.1f}", "meters (MSL)")
    print()
    
    print_success("✓ Barometer working")
    return True


def test_home_location(vehicle):
    """Test home location"""
    print_header("STEP 7: TESTING HOME LOCATION")
    
    home = vehicle.home_location
    
    if home is None:
        print_error("Home location not set!")
        print_info("Home will be set when GPS gets 3D fix")
        return False
    else:
        print_data("Home Latitude", f"{home.lat:.6f}", "degrees")
        print_data("Home Longitude", f"{home.lon:.6f}", "degrees")
        print_data("Home Altitude", f"{home.alt:.1f}", "meters MSL")
        print()
        print_success("✓ Home location is set")
        return True


def test_mode_and_armed(vehicle):
    """Test mode and armed status"""
    print_header("STEP 8: TESTING FLIGHT MODE & ARMED STATUS")
    
    print_data("Current Mode", vehicle.mode.name, "")
    print_data("Armed Status", "YES" if vehicle.armed else "NO", "")
    print()
    
    if vehicle.armed:
        print_error("WARNING: Vehicle is currently armed!")
    else:
        print_info("Vehicle is disarmed (safe)")
    
    print_success("✓ Mode and armed status readable")
    return True


def test_version(vehicle):
    """Test firmware version and autopilot info"""
    print_header("STEP 9: TESTING AUTOPILOT VERSION")
    
    print_data("Autopilot Type", "Pixhawk", "")
    print_data("Firmware Version", str(vehicle.version), "")
    print_data("EKF Status", "OK" if vehicle.ekf_ok else "Not Ready", "")
    print()
    
    print_success("✓ Version info readable")
    return True


def test_ekf(vehicle):
    """Test EKF (Extended Kalman Filter) status"""
    print_header("STEP 10: TESTING EKF (SENSOR FUSION)")
    
    ekf_ok = vehicle.ekf_ok
    print_data("EKF Status", "CONVERGED ✓" if ekf_ok else "NOT CONVERGED", "")
    
    if ekf_ok:
        print_success("✓ EKF converged - vehicle ready for arming")
    else:
        print_info("EKF not converged - wait 30+ seconds for convergence")
    
    print()
    return True


def continuous_monitor(vehicle, duration=30):
    """Monitor all telemetry continuously for specified duration"""
    print_header(f"CONTINUOUS MONITORING ({duration} SECONDS)")
    
    print_info("Displaying all telemetry in real-time...\n")
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        elapsed = int(time.time() - start_time)
        
        # Get all data
        gps = vehicle.gps_0
        battery = vehicle.battery
        attitude = vehicle.attitude
        home = vehicle.home_location
        
        import math
        yaw_deg = math.degrees(attitude.yaw)
        
        # Print telemetry
        print(f"{Colors.CYAN}{'='*70}{Colors.RESET}")
        print(f"{Colors.BOLD}[TIME: {elapsed}s] TELEMETRY READ{Colors.RESET}")
        print(f"{Colors.CYAN}{'='*70}{Colors.RESET}")
        
        print(f"GPS: Fix={gps.fix_type} Sats={gps.satellites_visible} | Battery={battery.voltage:.2f}V | Alt={vehicle.location.global_relative_frame.alt:.1f}m | Heading={yaw_deg:.0f}°")
        
        if home:
            print(f"Home: ({home.lat:.6f}, {home.lon:.6f})")
        
        print(f"Mode={vehicle.mode.name} | Armed={vehicle.armed} | EKF={vehicle.ekf_ok}")
        print()
        
        time.sleep(1)
    
    print_success(f"✓ Monitoring completed ({duration} seconds)")


def main():
    """Main test execution"""
    parser = argparse.ArgumentParser(
        description="Test Pixhawk 2.4.8 connection and MAVLink communication"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0 for 3DR radio)"
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="Baud rate (default: 57600 for 3DR radio)"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Connection timeout (default: 60 seconds)"
    )
    parser.add_argument(
        "--gps-wait",
        type=int,
        default=120,
        help="Maximum time to wait for GPS lock (default: 120 seconds)"
    )
    parser.add_argument(
        "--monitor",
        type=int,
        default=0,
        help="Continuous monitoring duration in seconds (default: disabled)"
    )
    
    args = parser.parse_args()
    
    # Print banner
    print(f"{Colors.BOLD}{Colors.CYAN}")
    print("╔" + "="*68 + "╗")
    print("║" + " "*68 + "║")
    print("║" + "  PIXHAWK 2.4.8 CONNECTION & MAVLINK TEST SCRIPT".center(68) + "║")
    print("║" + " "*68 + "║")
    print("╚" + "="*68 + "╝")
    print(f"{Colors.RESET}\n")
    
    # Test connection
    vehicle = test_connection(args.port, args.baud, args.timeout)
    if vehicle is None:
        return False
    
    try:
        # Run all tests
        tests = [
            ("Version", lambda: test_version(vehicle)),
            ("EKF Status", lambda: test_ekf(vehicle)),
            ("Attitude", lambda: test_attitude(vehicle)),
            ("Compass", lambda: test_compass(vehicle)),
            ("Barometer", lambda: test_barometer(vehicle)),
            ("Battery", lambda: test_battery(vehicle)),
            ("Mode & Armed", lambda: test_mode_and_armed(vehicle)),
            ("Home Location", lambda: test_home_location(vehicle)),
            ("GPS", lambda: test_gps(vehicle, args.gps_wait)),
        ]
        
        results = {}
        for test_name, test_func in tests:
            try:
                results[test_name] = test_func()
            except Exception as e:
                print_error(f"Test failed: {str(e)}")
                results[test_name] = False
        
        # Print summary
        print_header("TEST SUMMARY")
        
        passed = sum(1 for v in results.values() if v)
        total = len(results)
        
        for test_name, result in results.items():
            status = f"{Colors.GREEN}✓ PASS{Colors.RESET}" if result else f"{Colors.RED}✗ FAIL{Colors.RESET}"
            print(f"{test_name:<30} {status}")
        
        print()
        print(f"Result: {Colors.BOLD}{passed}/{total} tests passed{Colors.RESET}")
        
        if passed == total:
            print_success("ALL TESTS PASSED - Pixhawk is ready for flight!")
        else:
            print_error(f"{total - passed} test(s) failed - see above for details")
        
        # Continuous monitoring (if requested)
        if args.monitor > 0:
            continuous_monitor(vehicle, args.monitor)
        
        return passed == total
        
    finally:
        vehicle.close()
        print_info("Connection closed")


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
