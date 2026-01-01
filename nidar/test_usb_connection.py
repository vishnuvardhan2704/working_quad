#!/usr/bin/env python3
"""
USB Connection Test Script for Pixhawk
=======================================
Tests Pixhawk communication via USB cable (no TX/RX wiring needed).

Usage:
    python test_usb_connection.py

Connect Pixhawk to Raspberry Pi via USB cable, then run this script.
"""

import sys
import time
import glob

def print_header():
    print("\n" + "="*60)
    print("   PIXHAWK USB CONNECTION TEST")
    print("   (No TX/RX wiring required)")
    print("="*60 + "\n")

def find_serial_ports():
    """Find all available serial ports."""
    ports = []
    
    # Common USB serial patterns
    patterns = [
        '/dev/ttyACM*',      # Pixhawk USB (most common)
        '/dev/ttyUSB*',      # USB-to-serial adapters
        '/dev/serial/by-id/*',  # By device ID
    ]
    
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    # Also check serial0 (UART)
    import os
    if os.path.exists('/dev/serial0'):
        ports.append('/dev/serial0')
    
    return list(set(ports))

def test_port_raw(port, baud=115200):
    """Test if a port has any data (raw serial test)."""
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=2)
        time.sleep(0.5)
        data = ser.read(50)
        ser.close()
        return len(data) > 0, data
    except Exception as e:
        return False, str(e)

def test_mavlink_connection(port, baud=115200):
    """Test MAVLink connection to Pixhawk."""
    try:
        from dronekit import connect
        
        print(f"  Attempting MAVLink connection...")
        vehicle = connect(port, baud=baud, wait_ready=True, timeout=30)
        
        # Get basic info
        info = {
            'firmware': str(vehicle.version),
            'armed': vehicle.armed,
            'mode': str(vehicle.mode.name),
            'gps': vehicle.gps_0.fix_type if vehicle.gps_0 else 'N/A',
            'battery_v': vehicle.battery.voltage if vehicle.battery else 'N/A',
        }
        
        vehicle.close()
        return True, info
        
    except Exception as e:
        return False, str(e)

def main():
    print_header()
    
    # Step 1: Find available ports
    print("[1/3] SCANNING FOR SERIAL PORTS...")
    print("-" * 40)
    
    ports = find_serial_ports()
    
    if not ports:
        print("  ✗ No serial ports found!")
        print("\n  Make sure Pixhawk is connected via USB cable.")
        print("  The USB port should create /dev/ttyACM0")
        return
    
    print(f"  Found {len(ports)} port(s):")
    for p in ports:
        print(f"    • {p}")
    
    # Step 2: Test each port for data
    print("\n[2/3] TESTING PORTS FOR DATA...")
    print("-" * 40)
    
    working_ports = []
    for port in ports:
        print(f"\n  Testing {port}...")
        has_data, result = test_port_raw(port)
        if has_data:
            print(f"    ✓ Port has data!")
            working_ports.append(port)
        else:
            print(f"    ✗ No data: {result}")
    
    if not working_ports:
        print("\n  ✗ No ports with data found!")
        print("  Troubleshooting:")
        print("    - Check USB cable connection")
        print("    - Make sure Pixhawk is powered on")
        print("    - Try a different USB cable")
        print("    - Check dmesg for USB detection: dmesg | tail -20")
        return
    
    # Step 3: Test MAVLink connection
    print("\n[3/3] TESTING MAVLINK CONNECTION...")
    print("-" * 40)
    
    for port in working_ports:
        print(f"\n  Testing MAVLink on {port}...")
        success, info = test_mavlink_connection(port)
        
        if success:
            print(f"    ✓ MAVLink connection successful!")
            print(f"\n    Vehicle Information:")
            print(f"      Firmware: {info['firmware']}")
            print(f"      Mode:     {info['mode']}")
            print(f"      Armed:    {info['armed']}")
            print(f"      GPS Fix:  {info['gps']}")
            print(f"      Battery:  {info['battery_v']}V")
            
            print("\n" + "="*60)
            print("   ✓ SUCCESS! Pixhawk is connected and working!")
            print(f"   Use port: {port}")
            print("="*60 + "\n")
            return
        else:
            print(f"    ✗ MAVLink failed: {info}")
    
    print("\n" + "="*60)
    print("   ✗ Could not establish MAVLink connection")
    print("   Troubleshooting:")
    print("     - Make sure Pixhawk firmware is installed")
    print("     - Check if another program is using the port")
    print("     - Try: sudo chmod 666 /dev/ttyACM0")
    print("="*60 + "\n")

if __name__ == "__main__":
    main()
