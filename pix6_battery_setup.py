#!/usr/bin/env python3
"""
RadioLink Pix6 Battery Configuration Script

This script configures the battery monitor parameters for RadioLink Pix6.
Run this ONCE to enable battery monitoring.

Usage:
    python3 pix6_battery_setup.py --port /dev/ttyACM1
"""

import argparse
import time
import sys

try:
    from dronekit import connect
except ImportError:
    print("ERROR: dronekit not installed")
    sys.exit(1)


def setup_battery(port='/dev/ttyACM1', baud=115200):
    """Configure battery monitor for RadioLink Pix6."""
    
    print("=" * 60)
    print("RADIOLINK PIX6 BATTERY SETUP")
    print("=" * 60)
    print(f"\nConnecting to {port}...")
    
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected! Firmware: {vehicle.version}")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False
    
    # RadioLink Pix6 battery parameters
    # These are typical values - may need calibration
    params_to_set = {
        'BATT_MONITOR': 4,       # Analog Voltage and Current
        'BATT_VOLT_PIN': 14,     # Voltage pin for Pix6
        'BATT_CURR_PIN': 15,     # Current pin for Pix6
        'BATT_VOLT_MULT': 10.1,  # Voltage multiplier (calibrate later)
        'BATT_AMP_PERVLT': 17.0, # Amps per volt for RadioLink PM
        'BATT_ARM_VOLT': 10.5,   # Min voltage to arm (3S LiPo)
        'BATT_LOW_VOLT': 10.8,   # Low battery warning
        'BATT_CRT_VOLT': 10.2,   # Critical battery
        'BATT_CAPACITY': 2200,   # Battery capacity mAh (adjust to your battery)
    }
    
    print("\nSetting battery parameters...")
    print("-" * 60)
    
    for param, value in params_to_set.items():
        try:
            old_value = vehicle.parameters.get(param, 'N/A')
            vehicle.parameters[param] = value
            time.sleep(0.5)  # Give it time to set
            new_value = vehicle.parameters.get(param, 'N/A')
            print(f"  {param:18} : {old_value} → {new_value}")
        except Exception as e:
            print(f"  {param:18} : ERROR - {e}")
    
    print("-" * 60)
    print("\n⚠ IMPORTANT: Parameters set. You may need to:")
    print("  1. Reboot the flight controller")
    print("  2. Calibrate BATT_VOLT_MULT with a multimeter")
    print("")
    print("To calibrate voltage multiplier:")
    print("  measured_voltage / displayed_voltage * current_mult = new_mult")
    print("")
    
    # Wait for parameters to be written
    print("Waiting for parameter write...")
    time.sleep(2)
    
    # Check if it worked
    print("\nVerifying settings...")
    batt_monitor = vehicle.parameters.get('BATT_MONITOR', 0)
    if batt_monitor == 4:
        print("✓ BATT_MONITOR is now enabled (4)")
    else:
        print(f"⚠ BATT_MONITOR is {batt_monitor} (expected 4)")
    
    # Read battery now
    print("\nReading battery status...")
    time.sleep(1)
    battery = vehicle.battery
    if battery:
        print(f"  Voltage: {battery.voltage}V")
        print(f"  Current: {battery.current}A")
        print(f"  Level:   {battery.level}%")
    
    if battery and battery.voltage and battery.voltage > 0:
        print("\n✓ SUCCESS! Battery is now being read.")
    else:
        print("\n⚠ Battery still showing 0V.")
        print("  Try one of these:")
        print("  1. Reboot the Pix6 and run this script again")
        print("  2. Try different voltage pins:")
        print("     - BATT_VOLT_PIN = 2  (older firmware)")
        print("     - BATT_VOLT_PIN = 14 (newer firmware)")
        print("  3. Check power module is connected to POWER1 port")
    
    vehicle.close()
    print("\nSetup complete. Reconnect to verify.")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Setup RadioLink Pix6 Battery Monitor')
    parser.add_argument('--port', default='/dev/ttyACM1', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    setup_battery(args.port, args.baud)
