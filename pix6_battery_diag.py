#!/usr/bin/env python3
"""
RadioLink Pix6 Battery & Telemetry Diagnostic Tool

This script diagnoses why battery voltage and other parameters show 0.0
on RadioLink Pix6 flight controllers.

Common issues:
1. Battery monitor not configured (BATT_MONITOR param)
2. Different MAVLink message streams
3. Voltage/current sensor pins not set correctly
4. Power module not connected or configured

Usage:
    python3 pix6_battery_diag.py
    python3 pix6_battery_diag.py --port /dev/ttyACM0
"""

import argparse
import time
import sys

try:
    from dronekit import connect
    from pymavlink import mavutil
except ImportError as e:
    print(f"ERROR: Missing dependencies: {e}")
    print("Run: pip install dronekit pymavlink")
    sys.exit(1)


def diagnose_battery(port='/dev/ttyACM0', baud=115200):
    """Run comprehensive battery diagnostics."""
    
    print("=" * 60)
    print("RADIOLINK PIX6 BATTERY DIAGNOSTIC")
    print("=" * 60)
    print(f"\nConnecting to {port} at {baud} baud...")
    
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected to vehicle")
        print(f"  Firmware: {vehicle.version}")
        print(f"  Autopilot: {vehicle._vehicle_type if hasattr(vehicle, '_vehicle_type') else 'Unknown'}")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    print("\n" + "=" * 60)
    print("1. DRONEKIT BATTERY OBJECT")
    print("=" * 60)
    
    battery = vehicle.battery
    print(f"\nvehicle.battery = {battery}")
    if battery:
        print(f"  .voltage = {battery.voltage}")
        print(f"  .current = {battery.current}")
        print(f"  .level   = {battery.level}")
    else:
        print("  ⚠ Battery object is None!")
    
    print("\n" + "=" * 60)
    print("2. BATTERY CONFIGURATION PARAMETERS")
    print("=" * 60)
    
    # Key battery parameters for RadioLink Pix6
    batt_params = [
        'BATT_MONITOR',      # 0=disabled, 3=analog voltage only, 4=analog V+I
        'BATT_VOLT_PIN',     # Analog pin for voltage (usually 2 or 13)
        'BATT_CURR_PIN',     # Analog pin for current (usually 3 or 14)
        'BATT_VOLT_MULT',    # Voltage multiplier
        'BATT_AMP_PERVLT',   # Amps per volt
        'BATT_ARM_VOLT',     # Minimum arming voltage
        'BATT_LOW_VOLT',     # Low battery warning voltage
        'BATT_CRT_VOLT',     # Critical battery voltage
        'BATT_CAPACITY',     # Battery capacity mAh
        'BATT2_MONITOR',     # Secondary battery monitor
    ]
    
    print("\nReading battery parameters...")
    for param in batt_params:
        try:
            value = vehicle.parameters.get(param, 'NOT FOUND')
            status = ""
            if param == 'BATT_MONITOR':
                if value == 0:
                    status = " ⚠ DISABLED! Set to 3 or 4"
                elif value == 3:
                    status = " (Analog Voltage Only)"
                elif value == 4:
                    status = " (Analog Voltage + Current)"
            elif param == 'BATT_VOLT_PIN' and value == -1:
                status = " ⚠ NOT SET!"
            elif param == 'BATT_VOLT_MULT' and value == 1:
                status = " ⚠ May need calibration"
            print(f"  {param:18} = {value}{status}")
        except Exception as e:
            print(f"  {param:18} = ERROR: {e}")
    
    print("\n" + "=" * 60)
    print("3. RAW MAVLINK BATTERY MESSAGES")
    print("=" * 60)
    
    print("\nRequesting data streams...")
    # Request all data streams
    vehicle._master.mav.request_data_stream_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10 Hz
        1    # Start
    )
    
    # Specifically request extended sys state
    vehicle._master.mav.request_data_stream_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,
        1
    )
    
    print("Listening for battery messages (10 seconds)...")
    print("-" * 60)
    
    battery_msgs = {
        'SYS_STATUS': None,
        'BATTERY_STATUS': None,
        'BATTERY2': None,
    }
    
    start_time = time.time()
    while time.time() - start_time < 10:
        try:
            msg = vehicle._master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'SYS_STATUS':
                    battery_msgs['SYS_STATUS'] = msg
                    print(f"\n[SYS_STATUS]")
                    print(f"  voltage_battery  = {msg.voltage_battery} mV ({msg.voltage_battery/1000:.2f}V)")
                    print(f"  current_battery  = {msg.current_battery} cA ({msg.current_battery/100:.2f}A)")
                    print(f"  battery_remaining = {msg.battery_remaining}%")
                    
                elif msg_type == 'BATTERY_STATUS':
                    battery_msgs['BATTERY_STATUS'] = msg
                    print(f"\n[BATTERY_STATUS]")
                    print(f"  id              = {msg.id}")
                    print(f"  voltages        = {msg.voltages[:4]} mV")  # First 4 cells
                    print(f"  current_battery = {msg.current_battery} cA")
                    print(f"  current_consumed = {msg.current_consumed} mAh")
                    print(f"  battery_remaining = {msg.battery_remaining}%")
                    print(f"  temperature     = {msg.temperature} cdegC")
                    
                elif msg_type == 'BATTERY2':
                    battery_msgs['BATTERY2'] = msg
                    print(f"\n[BATTERY2]")
                    print(f"  voltage = {msg.voltage} mV")
                    print(f"  current_battery = {msg.current_battery} cA")
                    
        except Exception as e:
            pass
    
    print("\n" + "=" * 60)
    print("4. ANALOG INPUT READINGS")
    print("=" * 60)
    
    # Try to read raw analog values
    analog_params = ['BATT_VOLT_PIN', 'BATT_CURR_PIN']
    
    print("\nChecking for SCALED_PRESSURE / SENSOR messages...")
    start_time = time.time()
    while time.time() - start_time < 3:
        msg = vehicle._master.recv_match(type=['RAW_IMU', 'SCALED_PRESSURE', 'POWER_STATUS'], blocking=True, timeout=1)
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'POWER_STATUS':
                print(f"\n[POWER_STATUS]")
                print(f"  Vcc   = {msg.Vcc} mV")
                print(f"  Vservo = {msg.Vservo} mV")
                print(f"  flags = {msg.flags}")
    
    print("\n" + "=" * 60)
    print("5. DIAGNOSIS & RECOMMENDATIONS")
    print("=" * 60)
    
    issues_found = []
    
    # Check BATT_MONITOR
    batt_monitor = vehicle.parameters.get('BATT_MONITOR', 0)
    if batt_monitor == 0:
        issues_found.append({
            'issue': 'Battery monitor is DISABLED',
            'fix': 'Set BATT_MONITOR to 4 (Analog Voltage and Current)',
            'command': 'param set BATT_MONITOR 4'
        })
    
    # Check voltage pin
    volt_pin = vehicle.parameters.get('BATT_VOLT_PIN', -1)
    if volt_pin == -1 or volt_pin == 0:
        issues_found.append({
            'issue': 'Battery voltage pin not configured',
            'fix': 'For RadioLink Pix6, try BATT_VOLT_PIN = 14 or 2',
            'command': 'param set BATT_VOLT_PIN 14'
        })
    
    # Check voltage multiplier
    volt_mult = vehicle.parameters.get('BATT_VOLT_MULT', 1)
    if volt_mult == 1:
        issues_found.append({
            'issue': 'Voltage multiplier may need calibration',
            'fix': 'For RadioLink power module, typical value is ~10.1',
            'command': 'param set BATT_VOLT_MULT 10.1'
        })
    
    # Check if voltage is 0
    if battery and battery.voltage == 0:
        issues_found.append({
            'issue': 'Battery voltage reads 0V',
            'fix': 'Check power module connection to POWER1 port',
            'command': None
        })
    
    # Check SYS_STATUS message
    if battery_msgs['SYS_STATUS']:
        sys_status = battery_msgs['SYS_STATUS']
        if sys_status.voltage_battery == 0 or sys_status.voltage_battery == 65535:
            issues_found.append({
                'issue': 'SYS_STATUS shows invalid voltage',
                'fix': 'Power module may not be detected',
                'command': None
            })
    
    if issues_found:
        print(f"\n⚠ Found {len(issues_found)} issue(s):\n")
        for i, issue in enumerate(issues_found, 1):
            print(f"Issue {i}: {issue['issue']}")
            print(f"  Fix: {issue['fix']}")
            if issue['command']:
                print(f"  Command: {issue['command']}")
            print()
    else:
        print("\n✓ No obvious configuration issues found")
        print("  Battery should be reading correctly")
    
    print("\n" + "=" * 60)
    print("6. RADIOLINK PIX6 SPECIFIC SETTINGS")
    print("=" * 60)
    
    print("""
RadioLink Pix6 typically uses these battery settings:

For RadioLink PM (Power Module):
  BATT_MONITOR    = 4    (Analog Voltage and Current)
  BATT_VOLT_PIN   = 14   (or 2, depends on firmware)
  BATT_CURR_PIN   = 15   (or 3, depends on firmware)
  BATT_VOLT_MULT  = 10.1 (calibrate with multimeter)
  BATT_AMP_PERVLT = 17.0 (for RadioLink PM)

To set these parameters via MAVProxy:
  param set BATT_MONITOR 4
  param set BATT_VOLT_PIN 14
  param set BATT_CURR_PIN 15
  param set BATT_VOLT_MULT 10.1
  param set BATT_AMP_PERVLT 17.0

Or via Mission Planner:
  Config/Tuning → Full Parameter List → Search "BATT"
    """)
    
    print("\n" + "=" * 60)
    print("7. CONTINUOUS MONITORING (Press Ctrl+C to stop)")
    print("=" * 60)
    
    print("\nMonitoring battery every 2 seconds...")
    try:
        while True:
            battery = vehicle.battery
            if battery:
                v = battery.voltage if battery.voltage else 0
                c = battery.current if battery.current else 0
                l = battery.level if battery.level else 0
                print(f"  Battery: {v:.2f}V, {c:.2f}A, {l}%")
            else:
                print("  Battery: None")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n\nStopped monitoring.")
    
    vehicle.close()
    print("\nDiagnostic complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='RadioLink Pix6 Battery Diagnostic')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    diagnose_battery(args.port, args.baud)
