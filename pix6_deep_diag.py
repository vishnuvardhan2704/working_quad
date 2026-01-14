#!/usr/bin/env python3
"""
Deep diagnostic for RadioLink Pix6 battery - checks raw MAVLink messages
and hardware detection.
"""

import argparse
import time
import sys

try:
    from dronekit import connect
    from pymavlink import mavutil
except ImportError as e:
    print(f"ERROR: {e}")
    sys.exit(1)


def deep_diagnostic(port='/dev/ttyACM1', baud=115200):
    print("=" * 60)
    print("PIX6 DEEP BATTERY DIAGNOSTIC")
    print("=" * 60)
    print(f"\nConnecting to {port}...")
    
    try:
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=60)
        print(f"✓ Connected! Firmware: {vehicle.version}")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    # Request all data streams at high rate
    print("\nRequesting data streams...")
    master = vehicle._master
    
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10, 1
    )
    
    print("\n" + "=" * 60)
    print("LISTENING FOR ALL BATTERY/POWER MESSAGES (15 seconds)")
    print("=" * 60)
    
    seen_types = set()
    start = time.time()
    
    while time.time() - start < 15:
        try:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg:
                msg_type = msg.get_type()
                
                # Only print each type once with full details
                if msg_type not in seen_types:
                    if msg_type == 'SYS_STATUS':
                        seen_types.add(msg_type)
                        print(f"\n[SYS_STATUS]")
                        print(f"  voltage_battery    = {msg.voltage_battery} mV ({msg.voltage_battery/1000:.2f}V)")
                        print(f"  current_battery    = {msg.current_battery} cA")
                        print(f"  battery_remaining  = {msg.battery_remaining}%")
                        print(f"  onboard_ctrl_sens  = {bin(msg.onboard_control_sensors_present)}")
                        
                    elif msg_type == 'BATTERY_STATUS':
                        seen_types.add(msg_type)
                        print(f"\n[BATTERY_STATUS]")
                        print(f"  id                = {msg.id}")
                        print(f"  voltages (cells)  = {list(msg.voltages[:6])} mV")
                        print(f"  current_battery   = {msg.current_battery} cA")
                        print(f"  current_consumed  = {msg.current_consumed} mAh")
                        print(f"  battery_remaining = {msg.battery_remaining}%")
                        print(f"  time_remaining    = {msg.time_remaining} sec")
                        print(f"  battery_function  = {msg.battery_function}")
                        print(f"  type              = {msg.type}")
                        
                    elif msg_type == 'POWER_STATUS':
                        seen_types.add(msg_type)
                        print(f"\n[POWER_STATUS]")
                        print(f"  Vcc    = {msg.Vcc} mV (5V rail)")
                        print(f"  Vservo = {msg.Vservo} mV")
                        print(f"  flags  = {bin(msg.flags)}")
                        
                    elif msg_type == 'HWSTATUS':
                        seen_types.add(msg_type)
                        print(f"\n[HWSTATUS]")
                        print(f"  Vcc    = {msg.Vcc} mV")
                        print(f"  I2Cerr = {msg.I2Cerr}")
                        
                    elif msg_type == 'HEARTBEAT':
                        if 'HEARTBEAT' not in seen_types:
                            seen_types.add('HEARTBEAT')
                            print(f"\n[HEARTBEAT]")
                            print(f"  autopilot = {msg.autopilot}")
                            print(f"  type      = {msg.type}")
                            print(f"  base_mode = {bin(msg.base_mode)}")
                            
        except Exception as e:
            pass
    
    print(f"\n\nMessage types seen: {seen_types}")
    
    print("\n" + "=" * 60)
    print("DIAGNOSIS")
    print("=" * 60)
    
    # Current reading
    battery = vehicle.battery
    print(f"\nDroneKit battery: voltage={battery.voltage}, current={battery.current}")
    
    if battery.voltage == 0 or battery.voltage is None:
        print("\n⚠ Battery voltage is 0V - Possible causes:")
        print("")
        print("1. POWER MODULE NOT CONNECTED:")
        print("   - Check power module is plugged into POWER1 port on Pix6")
        print("   - The 6-pin connector should be secure")
        print("")
        print("2. WRONG BATT_MONITOR TYPE:")
        print("   For RadioLink Pix6 with standard PM:")
        print("   - BATT_MONITOR = 4 (Analog V+I)")
        print("")
        print("3. NEEDS REBOOT:")
        print("   - Power cycle the Pix6 after parameter changes")
        print("")
        print("4. POWER MODULE DAMAGED:")
        print("   - Use a multimeter to verify power module output")
        print("   - Check if voltage sense wire is connected")
        print("")
        print("5. RADIOLINK PIX6 SPECIFIC:")
        print("   RadioLink Pix6 uses the same analog pins as standard Pixhawk")
        print("   Make sure you're using the POWER1 port (not POWER2)")
        print("")
        
        # Check what BATT_MONITOR is set to
        bm = vehicle.parameters.get('BATT_MONITOR', 0)
        print(f"Current BATT_MONITOR = {bm}")
        
        if bm == 0:
            print("  → DISABLED! Run: python3 pix6_battery_setup.py")
        elif bm == 4:
            print("  → Correctly set to Analog V+I")
            print("  → Issue is likely hardware (power module connection)")
    
    vehicle.close()
    print("\nDiagnostic complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    
    deep_diagnostic(args.port, args.baud)
