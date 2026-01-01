#!/usr/bin/env python3
"""
3DR Telemetry Radio Setup & Configuration Script

This script helps configure and bind 3DR telemetry radios for
communication between Raspberry Pi and Pixhawk.

Usage:
    python3 setup_3dr_radio.py              # Auto-detect and configure
    python3 setup_3dr_radio.py --port /dev/ttyUSB0
    python3 setup_3dr_radio.py --info       # Show current radio settings
"""

import serial
import time
import argparse
import sys


def find_radio_port():
    """Find the 3DR radio USB port."""
    import glob
    
    # Check common ports
    ports_to_check = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    for port in ports_to_check:
        try:
            # Try to open and send AT command
            ser = serial.Serial(port, 57600, timeout=1)
            time.sleep(0.1)
            ser.write(b'+++')
            time.sleep(1.1)  # Wait for command mode
            ser.write(b'ATI\r\n')
            time.sleep(0.5)
            response = ser.read(200).decode('utf-8', errors='ignore')
            ser.close()
            
            if 'SiK' in response or 'RFD' in response:
                return port
        except:
            continue
    
    return None


def enter_command_mode(ser):
    """Enter AT command mode on the radio."""
    time.sleep(1)
    ser.write(b'+++')
    time.sleep(1.1)
    # Read any response
    ser.read(100)
    return True


def send_at_command(ser, cmd, wait=0.5):
    """Send AT command and return response."""
    ser.write(f'{cmd}\r\n'.encode())
    time.sleep(wait)
    response = ser.read(500).decode('utf-8', errors='ignore').strip()
    return response


def get_radio_info(port):
    """Get current radio configuration."""
    print(f"\n{'='*60}")
    print("3DR RADIO CONFIGURATION")
    print(f"{'='*60}")
    print(f"Port: {port}\n")
    
    try:
        ser = serial.Serial(port, 57600, timeout=2)
        enter_command_mode(ser)
        
        # Get radio info
        info = {
            'ATI': 'Radio Version',
            'ATI2': 'Board Type',
            'ATI3': 'Firmware Version',
            'ATI4': 'Board Frequency',
            'ATI5': 'All Parameters',
        }
        
        for cmd, desc in info.items():
            response = send_at_command(ser, cmd)
            print(f"{desc} ({cmd}):")
            for line in response.split('\n'):
                if line.strip():
                    print(f"  {line.strip()}")
            print()
        
        # Exit command mode
        send_at_command(ser, 'ATO')
        ser.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False


def configure_radio(port, net_id=25, air_speed=64, baud=57600):
    """Configure radio parameters for binding."""
    print(f"\n{'='*60}")
    print("CONFIGURING 3DR RADIO")
    print(f"{'='*60}")
    
    try:
        ser = serial.Serial(port, 57600, timeout=2)
        enter_command_mode(ser)
        
        print(f"\nSetting parameters:")
        print(f"  NET_ID (S3):    {net_id}")
        print(f"  AIR_SPEED (S2): {air_speed}")
        print(f"  SERIAL BAUD:    {baud}")
        
        # Set NET_ID (S3) - both radios must match
        response = send_at_command(ser, f'ATS3={net_id}')
        print(f"\n  ATS3={net_id} -> {response}")
        
        # Set AIR_SPEED (S2) - both radios must match
        response = send_at_command(ser, f'ATS2={air_speed}')
        print(f"  ATS2={air_speed} -> {response}")
        
        # Save settings
        response = send_at_command(ser, 'AT&W')
        print(f"  AT&W (save) -> {response}")
        
        # Reboot radio
        print("\n  Rebooting radio...")
        send_at_command(ser, 'ATZ', wait=2)
        
        ser.close()
        
        print("\n✓ Configuration saved!")
        print("\n⚠️  IMPORTANT: Configure the Pixhawk radio with SAME settings:")
        print(f"    NET_ID = {net_id}")
        print(f"    AIR_SPEED = {air_speed}")
        print("    Use QGroundControl or Mission Planner to configure Pixhawk radio")
        
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False


def test_connection(port):
    """Test if radio can communicate with Pixhawk."""
    print(f"\n{'='*60}")
    print("TESTING MAVLINK CONNECTION VIA 3DR RADIO")
    print(f"{'='*60}")
    
    try:
        from dronekit import connect
        
        print(f"\nConnecting to Pixhawk via {port} @ 57600...")
        print("(Make sure Pixhawk radio is powered and bound)\n")
        
        vehicle = connect(port, baud=57600, wait_ready=True, timeout=30)
        
        print("✓ Connected to Pixhawk via 3DR radio!")
        print(f"\n  Firmware: {vehicle.version}")
        print(f"  Mode:     {vehicle.mode.name}")
        print(f"  Armed:    {vehicle.armed}")
        
        # Test telemetry
        att = vehicle.attitude
        import math
        print(f"\n  Roll:  {math.degrees(att.roll):+.1f}°")
        print(f"  Pitch: {math.degrees(att.pitch):+.1f}°")
        print(f"  Yaw:   {math.degrees(att.yaw):+.1f}°")
        
        vehicle.close()
        print("\n✓ Radio link working!")
        return True
        
    except Exception as e:
        print(f"\n✗ Connection failed: {e}")
        print("\nTroubleshooting:")
        print("  1. Is the Pixhawk radio powered? (connected to TELEM1)")
        print("  2. Are both radios configured with same NET_ID?")
        print("  3. Are the radios in range of each other?")
        print("  4. Check LED status on both radios:")
        print("     - Solid green = connected")
        print("     - Blinking = searching for pair")
        return False


def main():
    parser = argparse.ArgumentParser(description='3DR Radio Setup')
    parser.add_argument('--port', help='Radio serial port (auto-detect if not specified)')
    parser.add_argument('--info', action='store_true', help='Show radio info only')
    parser.add_argument('--configure', action='store_true', help='Configure radio')
    parser.add_argument('--test', action='store_true', help='Test MAVLink connection')
    parser.add_argument('--net-id', type=int, default=25, help='NET_ID (default: 25)')
    parser.add_argument('--air-speed', type=int, default=64, help='Air speed (default: 64)')
    args = parser.parse_args()
    
    # Find port
    port = args.port
    if not port:
        print("Searching for 3DR radio...")
        port = find_radio_port()
        if not port:
            # Fallback to ttyUSB0
            import os
            if os.path.exists('/dev/ttyUSB0'):
                port = '/dev/ttyUSB0'
                print(f"Using default port: {port}")
            else:
                print("✗ No 3DR radio found!")
                print("  Connect the radio USB and try again.")
                return False
        else:
            print(f"✓ Found radio at {port}")
    
    # If no specific action, show menu
    if not (args.info or args.configure or args.test):
        print(f"\n{'='*60}")
        print("3DR TELEMETRY RADIO SETUP")
        print(f"{'='*60}")
        print(f"\nRadio port: {port}")
        print("\nOptions:")
        print("  1. Show radio info     (--info)")
        print("  2. Configure radio     (--configure)")
        print("  3. Test connection     (--test)")
        print("\nExample:")
        print(f"  python3 setup_3dr_radio.py --info")
        print(f"  python3 setup_3dr_radio.py --configure --net-id 25")
        print(f"  python3 setup_3dr_radio.py --test")
        
        # Quick test
        print(f"\n{'='*60}")
        print("QUICK TEST")
        print(f"{'='*60}")
        get_radio_info(port)
        return True
    
    if args.info:
        return get_radio_info(port)
    
    if args.configure:
        return configure_radio(port, args.net_id, args.air_speed)
    
    if args.test:
        return test_connection(port)


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
