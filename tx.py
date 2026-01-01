#!/usr/bin/env python3
"""
Pixhawk Telemetry Transmitter via 3DR Radio

Reads real telemetry data from Pixhawk 2.4.8 connected via USB
and transmits it in CSV format through 3DR radio to a receiver (e.g., PuTTY).

Hardware Setup:
    - Pixhawk 2.4.8 connected via USB (usually /dev/ttyACM0)
    - 3DR Radio connected via USB (usually /dev/ttyUSB0)

CSV Format:
    TIME,LAT,LON,ALT,ROLL,PITCH,YAW,SPD,VOLT,CURR,BAT_REM,
    ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,
    MODE,ARM_STATUS,PERSON_STATUS,PLAT,PLON,CONF

Usage:
    python3 tx.py
    python3 tx.py --pixhawk /dev/ttyACM0 --radio /dev/ttyUSB0
"""

import sys
import time
import math
import argparse
from datetime import datetime
from dronekit import connect, VehicleMode
import serial
from pymavlink import mavutil


def request_message_interval(vehicle, message_id, frequency_hz):
    """
    Request MAVLink message at a specific rate.
    
    Args:
        vehicle: DroneKit vehicle object
        message_id: MAVLink message ID
        frequency_hz: Desired frequency in Hz
    """
    vehicle._master.mav.command_long_send(
        vehicle._master.target_system,
        vehicle._master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,  # Message ID
        1e6 / frequency_hz,  # Interval in microseconds
        0, 0, 0, 0, 0
    )


def get_telemetry_data(vehicle):
    """
    Extract telemetry data from Pixhawk.
    
    Returns:
        dict: Telemetry data dictionary
    """
    # TIME - UTC
    time_str = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
    
    # GPS Position
    if vehicle.location.global_relative_frame:
        lat = vehicle.location.global_relative_frame.lat or 0.0
        lon = vehicle.location.global_relative_frame.lon or 0.0
        alt = vehicle.location.global_relative_frame.alt or 0.0
    else:
        lat, lon, alt = 0.0, 0.0, 0.0
    
    # Attitude (in degrees)
    if vehicle.attitude:
        roll = math.degrees(vehicle.attitude.roll)
        pitch = math.degrees(vehicle.attitude.pitch)
        yaw = math.degrees(vehicle.attitude.yaw)
    else:
        roll, pitch, yaw = 0.0, 0.0, 0.0
    
    # Ground Speed (m/s)
    spd = vehicle.groundspeed or 0.0
    
    # Battery Data
    if vehicle.battery:
        volt = vehicle.battery.voltage or 0.0
        curr = vehicle.battery.current or 0.0
        bat_rem = vehicle.battery.level or 0  # Percentage remaining
    else:
        volt, curr, bat_rem = 0.0, 0.0, 0
    
    # IMU Data - Accelerometer (m/s^2) and Gyroscope (deg/s)
    # Try to get from raw_imu message
    try:
        # Access the last received RAW_IMU message
        msg = vehicle.message_factory
        raw_imu = vehicle._master.messages.get('RAW_IMU', None)
        
        if raw_imu:
            # Convert from milliG to m/s^2 (1G = 9.81 m/s^2)
            acc_x = (raw_imu.xacc / 1000.0) * 9.81
            acc_y = (raw_imu.yacc / 1000.0) * 9.81
            acc_z = (raw_imu.zacc / 1000.0) * 9.81
            # Convert from millirad/s to deg/s
            gyro_x = math.degrees(raw_imu.xgyro / 1000.0)
            gyro_y = math.degrees(raw_imu.ygyro / 1000.0)
            gyro_z = math.degrees(raw_imu.zgyro / 1000.0)
        else:
            # Fallback: Calculate from attitude rate if available
            acc_x, acc_y, acc_z = 0.0, 0.0, 9.81  # Assume stationary
            gyro_x = math.degrees(vehicle.attitude.rollspeed) if vehicle.attitude else 0.0
            gyro_y = math.degrees(vehicle.attitude.pitchspeed) if vehicle.attitude else 0.0
            gyro_z = math.degrees(vehicle.attitude.yawspeed) if vehicle.attitude else 0.0
    except Exception as e:
        # If all else fails, use attitude rates for gyro
        acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
        try:
            gyro_x = math.degrees(vehicle.attitude.rollspeed) if vehicle.attitude else 0.0
            gyro_y = math.degrees(vehicle.attitude.pitchspeed) if vehicle.attitude else 0.0
            gyro_z = math.degrees(vehicle.attitude.yawspeed) if vehicle.attitude else 0.0
        except:
            gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
    
    # Flight Mode
    mode = vehicle.mode.name if vehicle.mode else "UNKNOWN"
    
    # Armed Status
    arm_status = "ARM" if vehicle.armed else "DISARM"
    
    # Person Detection Data (placeholder - integrate with your detection system)
    # For now, setting to 0 (no person detected)
    person_status = 0
    plat = 0.0
    plon = 0.0
    conf = 0.0
    
    return {
        'time': time_str,
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
        'spd': spd,
        'volt': volt,
        'curr': curr,
        'bat_rem': bat_rem,
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z,
        'mode': mode,
        'arm_status': arm_status,
        'person_status': person_status,
        'plat': plat,
        'plon': plon,
        'conf': conf
    }


def format_csv_line(data):
    """
    Format telemetry data as CSV line.
    
    Args:
        data: Dictionary of telemetry data
        
    Returns:
        str: CSV formatted string
    """
    csv_line = (
        f"{data['time']},"
        f"{data['lat']:.6f},{data['lon']:.6f},{data['alt']:.2f},"
        f"{data['roll']:.2f},{data['pitch']:.2f},{data['yaw']:.2f},"
        f"{data['spd']:.2f},{data['volt']:.2f},{data['curr']:.2f},{data['bat_rem']},"
        f"{data['acc_x']:.3f},{data['acc_y']:.3f},{data['acc_z']:.3f},"
        f"{data['gyro_x']:.3f},{data['gyro_y']:.3f},{data['gyro_z']:.3f},"
        f"{data['mode']},{data['arm_status']},"
        f"{data['person_status']},{data['plat']:.6f},{data['plon']:.6f},{data['conf']:.2f}\n"
    )
    return csv_line


def main():
    parser = argparse.ArgumentParser(description='Pixhawk to 3DR telemetry transmitter')
    parser.add_argument('--pixhawk', default='/dev/ttyACM0', 
                       help='Pixhawk USB port (default: /dev/ttyACM0)')
    parser.add_argument('--pixhawk-baud', type=int, default=115200,
                       help='Pixhawk baud rate (default: 115200)')
    parser.add_argument('--radio', default='/dev/ttyUSB0',
                       help='3DR Radio port (default: /dev/ttyUSB0)')
    parser.add_argument('--radio-baud', type=int, default=57600,
                       help='3DR Radio baud rate (default: 57600)')
    parser.add_argument('--rate', type=float, default=1.0,
                       help='Transmission rate in Hz (default: 1.0)')
    parser.add_argument('--header', action='store_true',
                       help='Send CSV header on startup')
    args = parser.parse_args()

    print("=" * 70)
    print("  PIXHAWK TELEMETRY TRANSMITTER VIA 3DR RADIO")
    print("=" * 70)
    
    # Connect to Pixhawk
    print(f"\n[1/2] Connecting to Pixhawk on {args.pixhawk} @ {args.pixhawk_baud} baud...")
    try:
        vehicle = connect(args.pixhawk, baud=args.pixhawk_baud, wait_ready=True, timeout=30)
        print(f"✓ Connected to Pixhawk!")
        print(f"  Autopilot: {vehicle.version}")
        print(f"  Vehicle Type: {vehicle._vehicle_type}")
        
        # Request RAW_IMU message stream (message ID 27) at 10 Hz
        print("  Requesting IMU data stream...")
        request_message_interval(vehicle, 27, 10)
        time.sleep(1)  # Allow messages to start flowing
    except Exception as e:
        print(f"✗ Failed to connect to Pixhawk: {e}")
        print("\nTroubleshooting:")
        print("  1. Check USB connection")
        print("  2. Verify port with: ls /dev/ttyACM* or ls /dev/ttyUSB*")
        print("  3. Check permissions: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)
    
    # Connect to 3DR Radio
    print(f"\n[2/2] Opening 3DR Radio on {args.radio} @ {args.radio_baud} baud...")
    try:
        radio = serial.Serial(
            port=args.radio,
            baudrate=args.radio_baud,
            timeout=1
        )
        time.sleep(2)  # Allow radio to initialize
        print(f"✓ 3DR Radio ready!")
    except Exception as e:
        print(f"✗ Failed to open 3DR Radio: {e}")
        vehicle.close()
        print("\nTroubleshooting:")
        print("  1. Check 3DR radio USB connection")
        print("  2. Verify port with: ls /dev/ttyUSB*")
        print("  3. Check permissions: sudo chmod 666 /dev/ttyUSB0")
        sys.exit(1)
    
    # Send CSV header if requested
    if args.header:
        header = "TIME,LAT,LON,ALT,ROLL,PITCH,YAW,SPD,VOLT,CURR,BAT_REM,ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,MODE,ARM_STATUS,PERSON_STATUS,PLAT,PLON,CONF\n"
        radio.write(header.encode())
        print(f"\n✓ Sent CSV header")
    
    print(f"\n{'=' * 70}")
    print(f"Transmitting at {args.rate} Hz - Press Ctrl+C to stop")
    print(f"{'=' * 70}\n")
    
    packet_count = 0
    interval = 1.0 / args.rate
    
    try:
        while True:
            start_time = time.time()
            
            # Get telemetry from Pixhawk
            telemetry = get_telemetry_data(vehicle)
            
            # Format as CSV
            csv_line = format_csv_line(telemetry)
            
            # Transmit via 3DR Radio
            radio.write(csv_line.encode())
            packet_count += 1
            
            # Display status
            print(f"[{packet_count:04d}] {telemetry['time']} | "
                  f"Pos: {telemetry['lat']:.5f},{telemetry['lon']:.5f} | "
                  f"Alt: {telemetry['alt']:.1f}m | "
                  f"Mode: {telemetry['mode']} | {telemetry['arm_status']} | "
                  f"Bat: {telemetry['volt']:.1f}V {telemetry['curr']:.1f}A ({telemetry['bat_rem']}%) | "
                  f"IMU: A({telemetry['acc_x']:.1f},{telemetry['acc_y']:.1f},{telemetry['acc_z']:.1f}) "
                  f"G({telemetry['gyro_x']:.1f},{telemetry['gyro_y']:.1f},{telemetry['gyro_z']:.1f})")
            
            # Maintain transmission rate
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print(f"\n\n{'=' * 70}")
        print(f"Transmission stopped. Total packets sent: {packet_count}")
        print(f"{'=' * 70}")
    except Exception as e:
        print(f"\n\nERROR: {e}")
    finally:
        print("\nClosing connections...")
        radio.close()
        vehicle.close()
        print("✓ Cleanup complete")


if __name__ == "__main__":
    main()
