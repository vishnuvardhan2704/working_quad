#!/usr/bin/env python3
"""
Test different BATT_MONITOR settings for RadioLink Pix6.
ArduCopter 4.6+ may use different monitor types.
"""

import argparse
import time
import sys

try:
    from dronekit import connect
except ImportError:
    print("ERROR: dronekit not installed")
    sys.exit(1)


def test_monitor_types(port='/dev/ttyACM1', baud=115200):
    print("=" * 60)
    print("TESTING DIFFERENT BATT_MONITOR TYPES")
    print("=" * 60)
    
    # Different monitor types to test
    monitor_types = {
        0: "Disabled",
        3: "Analog Voltage Only",
        4: "Analog Voltage and Current",
        5: "Solo (SMBus)",
        6: "Bebop",
        7: "SMBus-Generic",
        8: "DroneCAN-BatteryInfo",
        9: "ESC",
        10: "SumOfFollowing",
        11: "FuelFlow",
        12: "FuelLevel-PWM",
        13: "SMBUS-SUI3",
        14: "SMBUS-SUI6",
        15: "NeoDesign",
        16: "SMBus-Maxell",
        17: "Generator-Elec",
        18: "Generator-Fuel",
        19: "Rotoye",
        20: "MPPT",
        21: "INA2XX",
        22: "LTC2946",
        23: "Torqeedo",
    }
    
    print(f"\nConnecting to {port}...")
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected! Firmware: {vehicle.version}\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    current_monitor = vehicle.parameters.get('BATT_MONITOR', 0)
    print(f"Current BATT_MONITOR = {current_monitor} ({monitor_types.get(int(current_monitor), 'Unknown')})")
    
    # For RadioLink Pix6, the correct type should be 4 (Analog V+I)
    # But let's also check if there are board-specific parameters
    
    print("\n" + "=" * 60)
    print("CHECKING BOARD-SPECIFIC BATTERY SETTINGS")
    print("=" * 60)
    
    # Check for board ID
    board_params = ['BRD_TYPE', 'SERIAL0_BAUD', 'SERIAL0_PROTOCOL']
    for param in board_params:
        val = vehicle.parameters.get(param, 'N/A')
        print(f"  {param:20} = {val}")
    
    # Check if there are I2C battery settings
    print("\n" + "=" * 60)
    print("CHECKING I2C/SMBUS SETTINGS (if using smart battery)")
    print("=" * 60)
    
    i2c_params = []
    for param in vehicle.parameters:
        if 'I2C' in param.upper() or 'SMBUS' in param.upper():
            val = vehicle.parameters[param]
            print(f"  {param:20} = {val}")
            i2c_params.append(param)
    
    if not i2c_params:
        print("  No I2C/SMBus parameters found")
    
    # Manual voltage reading test
    print("\n" + "=" * 60)
    print("ATTEMPTING MANUAL VOLTAGE READ")
    print("=" * 60)
    
    print("\nIs your battery currently connected to the Pix6?")
    print("If YES and you have a multimeter, measure the battery voltage.")
    print("\nCurrent readings from Pix6:")
    
    for i in range(5):
        battery = vehicle.battery
        v = battery.voltage if battery and battery.voltage else 0
        c = battery.current if battery and battery.current else 0
        print(f"  [{i+1}] Voltage: {v:.2f}V, Current: {c:.2f}A")
        time.sleep(1)
    
    print("\n" + "=" * 60)
    print("RECOMMENDATIONS")
    print("=" * 60)
    
    if current_monitor == 4:
        print("\n✓ BATT_MONITOR is correctly set to 4 (Analog V+I)")
        print("\nSince voltage is still 0V, the issue is HARDWARE:")
        print("")
        print("1. POWER MODULE CONNECTION:")
        print("   - Locate the 6-pin POWER connector on your RadioLink Pix6")
        print("   - It's usually labeled 'POWER1' or 'POWER'")
        print("   - Ensure the power module's 6-pin connector is firmly inserted")
        print("")
        print("2. BATTERY CONNECTION:")
        print("   - The power module should have an XT60/XT30 connector")
        print("   - Connect your LiPo battery to this connector")
        print("   - The Pix6 LEDs should change (more lights, different pattern)")
        print("")
        print("3. VERIFY WITH MULTIMETER:")
        print("   - Measure battery voltage directly: should be 11.1V+ for 3S")
        print("   - Check power module output wires (usually red=V+, black=GND)")
        print("")
        print("4. RADIOLINK PIX6 SPECIFIC:")
        print("   Some Pix6 units may need this setting:")
        print("   - Try: BATT_MONITOR = 3 (Voltage only)")
        print("   - Command: param set BATT_MONITOR 3")
        print("")
    else:
        print(f"\n⚠ BATT_MONITOR is {current_monitor}, should be 4")
        print("   Run: python3 pix6_battery_setup.py")
    
    vehicle.close()
    print("\nTest complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    
    test_monitor_types(args.port, args.baud)
