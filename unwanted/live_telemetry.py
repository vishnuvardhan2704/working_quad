#!/usr/bin/env python3
"""
Live Telemetry Monitor for Pixhawk with 3DR Radio Transmission

Takes telemetry from Pixhawk 2.4.8 and transmits via 3DR radio to remote Windows PC.
Checks for radio binding before transmission.

CSV Format: TIME,LAT,LON,ALT,ROLL,PITCH,YAW,SPD,VOLT,MODE,PERSON_STATUS,PLAT,PLON,CONF

Usage:
    python3 live_telemetry.py                        # Local display only  
    python3 live_telemetry.py --tx-port /dev/ttyUSB0 # Transmit CSV via 3DR radio
"""

import sys
import time
import math
import argparse
import serial


def check_tx_radio_binding(port):
    """Check if TX radio is connected and bound to remote radio."""
    try:
        print(f"  Testing connection...")
        ser = serial.Serial(port, 57600, timeout=2)
        time.sleep(0.5)
        
        # Enter AT command mode
        ser.write(b'+++')
        time.sleep(1.2)
        
        # Check RSSI and link status
        ser.write(b'ATI5\r\n')
        time.sleep(0.5)
        response = ser.read(500).decode('utf-8', errors='ignore')
        
        # Exit AT mode
        ser.write(b'ATO\r\n')
        time.sleep(0.3)
        ser.close()
        
        # If we got a response, radio is connected
        if response and ('S' in response or 'OK' in response):
            print(f"  ✓ Radio responding")
            
            # Ask user to confirm LEDs show binding
            print("\n  Check 3DR radio LED status:")
            print("    🟢 Solid GREEN = Radios are BOUND ✓")
            print("    🔴 Blinking    = Searching (NOT bound)")
            
            user_input = input("\n  Are both radios showing solid green? (y/n): ").strip().lower()
            if user_input == 'y':
                return True
            else:
                print("  ✗ Radios not bound - configure with same NET_ID first")
                return False
        else:
            print(f"  ✗ No response from radio")
            return False
            
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False


def send_csv_packet(tx_ser, timestamp, loc, loc_rel, att, vel, bat, mode):
    """Send telemetry data as CSV packet in required format."""
    try:
        if not tx_ser or not tx_ser.is_open:
            return
        
        # Extract location (drone)
        lat = loc.lat if loc and loc.lat else 0.0
        lon = loc.lon if loc.lon else 0.0
        alt_rel = loc_rel.alt if loc_rel and loc_rel.alt else 0.0
        
        # Extract attitude
        roll = math.degrees(att.roll) if att and att.roll else 0.0
        pitch = math.degrees(att.pitch) if att and att.pitch else 0.0
        yaw = math.degrees(att.yaw) if att and att.yaw else 0.0
        
        # Extract velocity (ground speed)
        if vel:
            vx = vel[0] if vel[0] else 0.0
            vy = vel[1] if vel[1] else 0.0
            ground_speed = math.sqrt(vx**2 + vy**2)
        else:
            ground_speed = 0.0
        
        # Extract battery
        voltage = bat.voltage if bat and bat.voltage else 0.0
        
        # Person detection (not implemented yet - defaults)
        person_status = 0
        person_lat = 0.0
        person_lon = 0.0
        confidence = 0.0
        
        # Format: TIME,LAT,LON,ALT,ROLL,PITCH,YAW,SPD,VOLT,MODE,PERSON_STATUS,PLAT,PLON,CONF
        csv_line = f"{timestamp},{lat:.8f},{lon:.8f},{alt_rel:.2f},{roll:.2f},{pitch:.2f},{yaw:.2f},{ground_speed:.2f},{voltage:.2f},{mode},{person_status},{person_lat:.8f},{person_lon:.8f},{confidence:.2f}"
        
        tx_ser.write(f"{csv_line}\n".encode('utf-8'))
        tx_ser.flush()
    except Exception as e:
        pass


def main():
    parser = argparse.ArgumentParser(description='Live Telemetry Monitor with 3DR TX')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Pixhawk port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--tx-port', help='3DR TX radio port (e.g., /dev/ttyUSB0)')
    args = parser.parse_args()
    
    print("="*65)
    print("   PIXHAWK LIVE TELEMETRY - Press Ctrl+C to stop")
    print("="*65)
    
    # Check for TX radio and binding
    tx_radio = None
    tx_enabled = False
    if args.tx_port:
        print(f"\nChecking TX radio binding at {args.tx_port}...")
        print("="*65)
        if check_tx_radio_binding(args.tx_port):
            try:
                tx_radio = serial.Serial(args.tx_port, 57600, timeout=0.1)
                tx_enabled = True
                print(f"\n✓ TX radio BOUND - will transmit CSV to remote PC")
            except Exception as e:
                print(f"\n⚠ TX radio error: {e}")
                print(f"  Displaying locally only")
        else:
            print(f"\n⚠ TX radio NOT BOUND or not responding")
            print(f"  Displaying locally only")
        print("="*65)
        print()
    
    print(f"Connecting to Pixhawk at {args.port} @ {args.baud}...")
    
    from dronekit import connect
    vehicle = connect(args.port, baud=args.baud, wait_ready=False, timeout=30)
    print("✓ Connected to Pixhawk!")
    time.sleep(2)
    print()
    
    count = 0
    
    # Send CSV header if TX enabled
    if tx_enabled:
        header = "# TIME,LAT,LON,ALT,ROLL,PITCH,YAW,SPD,VOLT,MODE,PERSON_STATUS,PLAT,PLON,CONF"
        tx_radio.write(f"{header}\n".encode('utf-8'))
        tx_radio.flush()
        print("✓ CSV header sent to remote")
        print()
    
    try:
        while True:
            count += 1
            ts = time.strftime('%H:%M:%S')
            
            # Gather data
            gps = vehicle.gps_0
            loc = vehicle.location.global_frame
            loc_rel = vehicle.location.global_relative_frame
            att = vehicle.attitude
            vel = vehicle.velocity
            bat = vehicle.battery
            mode = vehicle.mode.name
            armed = vehicle.armed
            
            # Send CSV packet to TX radio
            if tx_enabled:
                send_csv_packet(tx_radio, ts, loc, loc_rel, att, vel, bat, mode)
            
            # Clear and print local display
            print("\033[2J\033[H", end="")
            
            status_line = "📡 TX: CSV" if tx_enabled else "🖥️  LOCAL"
            
            print("╔═══════════════════════════════════════════════════════════════╗")
            print("║        🚁 PIXHAWK LIVE TELEMETRY MONITOR 🚁                   ║")
            print("║                  Press Ctrl+C to stop                         ║")
            print("╚═══════════════════════════════════════════════════════════════╝")
            print(f"  [{ts}]  Update #{count}  |  {status_line}")
            print()
            
            # GPS
            print("┌─── 📡 GPS ──────────────────────────────────────────────────────")
            if gps:
                fix_names = {0:"No GPS", 1:"No Fix", 2:"2D", 3:"3D", 4:"DGPS", 5:"RTK Float", 6:"RTK Fix"}
                fix_str = fix_names.get(gps.fix_type, f"Type {gps.fix_type}")
                print(f"│ Fix: {fix_str:12}  Satellites: {gps.satellites_visible or 0}")
                
            if loc:
                lat = loc.lat if loc.lat else 0
                lon = loc.lon if loc.lon else 0
                alt = loc.alt if loc.alt else 0
                print(f"│ Lat:  {lat:14.8f}°")
                print(f"│ Lon:  {lon:14.8f}°")
                print(f"│ Alt:  {alt:10.2f} m (MSL)")
                
            if loc_rel and loc_rel.alt:
                print(f"│ Rel:  {loc_rel.alt:10.2f} m (above home)")
            print()
            
            # IMU / Attitude
            print("┌─── 🎯 IMU / ATTITUDE ─────────────────────────────────────────")
            if att:
                roll = math.degrees(att.roll) if att.roll else 0
                pitch = math.degrees(att.pitch) if att.pitch else 0
                yaw = math.degrees(att.yaw) if att.yaw else 0
                print(f"│ Roll:   {roll:+8.2f}°")
                print(f"│ Pitch:  {pitch:+8.2f}°")
                print(f"│ Yaw:    {yaw:+8.2f}° (heading)")
            print()
            
            # Velocity
            print("┌─── 💨 VELOCITY ───────────────────────────────────────────────")
            if vel:
                vx, vy, vz = vel[0] or 0, vel[1] or 0, vel[2] or 0
                gspeed = math.sqrt(vx**2 + vy**2)
                print(f"│ Vx (N): {vx:+8.2f} m/s")
                print(f"│ Vy (E): {vy:+8.2f} m/s")
                print(f"│ Vz (D): {vz:+8.2f} m/s")
                print(f"│ Ground: {gspeed:8.2f} m/s")
            print()
            
            # Power / Battery
            print("┌─── 🔋 POWER ──────────────────────────────────────────────────")
            if bat:
                v = bat.voltage if bat.voltage else 0
                c = bat.current if bat.current else 0
                l = bat.level if bat.level else 0
                if v > 0:
                    pct = min(100, max(0, (v - 10.5) / 2.1 * 100))
                    bar = "█" * int(pct/5) + "░" * (20 - int(pct/5))
                else:
                    bar = "░" * 20
                print(f"│ Voltage: {v:6.2f} V  [{bar}]")
                print(f"│ Current: {c:6.2f} A")
                print(f"│ Level:   {l:6}%")
            print()
            
            # Barometer
            print("┌─── 🌡️  BAROMETER ──────────────────────────────────────────────")
            if loc_rel and loc_rel.alt is not None:
                print(f"│ Baro Alt: {loc_rel.alt:8.2f} m (relative)")
            if loc and loc.alt is not None:
                print(f"│ Abs Alt:  {loc.alt:8.2f} m (MSL)")
            print()
            
            # Status
            print("┌─── ⚙️  STATUS ─────────────────────────────────────────────────")
            armed_display = "🔴 ARMED" if armed else "🟢 DISARMED"
            print(f"│ Mode:  {mode}")
            print(f"│ State: {armed_display}")
            print()
            
            print("─"*65)
            print("  Press Ctrl+C to stop")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
    finally:
        vehicle.close()
        print("✓ Disconnected from Pixhawk")
        if tx_radio and tx_radio.is_open:
            tx_radio.close()
            print("✓ TX radio closed")


if __name__ == "__main__":
    main()
