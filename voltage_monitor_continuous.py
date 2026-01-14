#!/usr/bin/env python3
"""
Continuous voltage logger for RadioLink Pix6.
Monitors all battery voltage sources in real-time.
"""

import argparse
import time
import sys
from datetime import datetime

try:
    from dronekit import connect
    from pymavlink import mavutil
except ImportError as e:
    print(f"ERROR: {e}")
    sys.exit(1)


def monitor_voltage(port='/dev/ttyACM1', baud=115200):
    print("=" * 70)
    print("PIX6 CONTINUOUS VOLTAGE MONITOR")
    print("=" * 70)
    
    print(f"\nConnecting to {port}...")
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected! Firmware: {vehicle.version}")
    except Exception as e:
        print(f"✗ Failed: {e}")
        return
    
    # Request high-rate data
    master = vehicle._master
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, 1
    )
    
    # Check config
    print("\nConfiguration:")
    batt_mon = vehicle.parameters.get('BATT_MONITOR', 0)
    print(f"  BATT_MONITOR = {batt_mon}")
    
    if batt_mon == 0:
        print("\n⚠ BATT_MONITOR is DISABLED!")
        print("  Battery voltage cannot be read until enabled.")
        print("  Run: python3 pix6_battery_setup.py")
        vehicle.close()
        return
    
    print("\n" + "=" * 70)
    print("MONITORING (Ctrl+C to stop)")
    print("=" * 70)
    print(f"{'Time':<12} {'DroneKit':<15} {'MAVLink SYS':<18} {'Servo/Batt':<15}")
    print("-" * 70)
    
    sys_voltage = 0
    sys_current = 0
    batt_cell1 = 0
    power_vservo = 0
    
    try:
        counter = 0
        while True:
            # Poll MAVLink
            msg = master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'SYS_STATUS':
                    sys_voltage = msg.voltage_battery
                    sys_current = msg.current_battery
                    
                elif msg_type == 'BATTERY_STATUS':
                    if len(msg.voltages) > 0:
                        batt_cell1 = msg.voltages[0]
                        
                elif msg_type == 'POWER_STATUS':
                    power_vservo = msg.Vservo
            
            counter += 1
            if counter >= 10:
                counter = 0
                
                battery = vehicle.battery
                dk_v = battery.voltage if battery and battery.voltage else 0
                dk_a = battery.current if battery and battery.current else 0
                
                now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                # Convert mV to V
                sys_v_display = f"{sys_voltage/1000:.2f}V" if sys_voltage > 0 else "0.00V"
                batt_v_display = f"{batt_cell1/1000:.2f}V" if batt_cell1 not in [0, 65535] else "N/A"
                servo_v_display = f"{power_vservo/1000:.2f}V" if power_vservo > 0 else "0.00V"
                
                dk_str = f"{dk_v:.2f}V {dk_a:.1f}A"
                sys_str = f"{sys_v_display} {sys_current/100:.1f}A"
                hw_str = f"Serv:{servo_v_display} Cell:{batt_v_display}"
                
                print(f"{now:<12} {dk_str:<15} {sys_str:<18} {hw_str:<15}", end='')
                
                # Alert if voltage detected
                if dk_v > 0:
                    print("  ✓ WORKING!", end='')
                elif sys_voltage > 1000:
                    print(f"  ⚠ MAVLink sees {sys_voltage/1000:.1f}V but DroneKit=0", end='')
                else:
                    print("  ✗ NO VOLTAGE", end='')
                
                print()  # newline
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("FINAL STATUS")
        print("=" * 70)
        
        battery = vehicle.battery
        final_v = battery.voltage if battery and battery.voltage else 0
        
        print(f"\nDroneKit battery.voltage = {final_v}V")
        print(f"MAVLink SYS_STATUS       = {sys_voltage}mV ({sys_voltage/1000:.2f}V)")
        print(f"MAVLink BATTERY_STATUS   = {batt_cell1}mV ({batt_cell1/1000:.2f}V)")
        print(f"MAVLink POWER_STATUS     = {power_vservo}mV (servo rail)")
        
        if final_v == 0 and sys_voltage == 0:
            print("\n✗ NO BATTERY VOLTAGE DETECTED")
            print("\nThis is a HARDWARE issue. Check:")
            print("  1. Battery connected to power module?")
            print("  2. Power module 6-pin cable in Pix6 POWER1 port?")
            print("  3. Pix6 LEDs change when battery connected?")
            print("  4. Measure battery with multimeter (should be 11-12V)")
            print("\nThe failsafe system CANNOT work until voltage is detected.")
        elif final_v > 0:
            print(f"\n✓ Battery voltage OK: {final_v}V")
            print("  Failsafe system will work correctly.")
    
    vehicle.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    
    monitor_voltage(args.port, args.baud)
