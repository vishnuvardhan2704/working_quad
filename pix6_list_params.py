#!/usr/bin/env python3
"""
List all battery-related parameters on the flight controller.
Helps identify correct parameter names for RadioLink Pix6.
"""

import argparse
import time
import sys

try:
    from dronekit import connect
except ImportError:
    print("ERROR: dronekit not installed")
    sys.exit(1)


def list_battery_params(port='/dev/ttyACM1', baud=115200):
    print(f"Connecting to {port}...")
    
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected! Firmware: {vehicle.version}\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    print("Downloading all parameters (this takes a moment)...")
    
    # Wait for parameters to download
    time.sleep(3)
    
    print("\n" + "=" * 60)
    print("ALL BATTERY-RELATED PARAMETERS")
    print("=" * 60)
    
    batt_params = []
    for param in vehicle.parameters:
        if 'BATT' in param.upper():
            value = vehicle.parameters[param]
            batt_params.append((param, value))
    
    batt_params.sort()
    
    for param, value in batt_params:
        print(f"  {param:25} = {value}")
    
    print(f"\nTotal battery parameters: {len(batt_params)}")
    
    # Also check for analog pins
    print("\n" + "=" * 60)
    print("ANALOG/ADC PIN PARAMETERS")
    print("=" * 60)
    
    for param in vehicle.parameters:
        if 'PIN' in param.upper() or 'ADC' in param.upper():
            value = vehicle.parameters[param]
            print(f"  {param:25} = {value}")
    
    # Check current battery reading
    print("\n" + "=" * 60)
    print("CURRENT BATTERY STATUS")
    print("=" * 60)
    
    battery = vehicle.battery
    print(f"  vehicle.battery.voltage = {battery.voltage if battery else 'None'}")
    print(f"  vehicle.battery.current = {battery.current if battery else 'None'}")
    print(f"  vehicle.battery.level   = {battery.level if battery else 'None'}")
    
    vehicle.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    
    list_battery_params(args.port, args.baud)
