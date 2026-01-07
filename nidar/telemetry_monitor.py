#!/usr/bin/env python3
"""
Standalone Real-Time Telemetry Monitor

Run this script to continuously monitor and send vehicle status to QGroundControl
or any MAVLink-compatible ground station. Shows:
- Battery voltage, current, percentage
- GPS status and satellite count
- EKF status
- Failsafe events
- Mode changes
- Arm/disarm events
- Pre-arm check failures
- Sensor health
- System messages from flight controller

Usage:
    python3 telemetry_monitor.py --connect /dev/ttyUSB0     # Real Pixhawk via USB
    python3 telemetry_monitor.py --connect udp:127.0.0.1:14551  # SITL

All logs appear in QGroundControl's message panel.
"""

import time
import argparse
import signal
import sys
from datetime import datetime

from dronekit import connect
from utils.telemetry_logger import (
    TelemetryLogger,
    VehicleStatusMonitor,
    PrearmCheckReporter,
    FailsafeMonitor
)


class TelemetryMonitor:
    """
    Standalone telemetry monitoring application.
    Connects to vehicle and streams status to ground station.
    """
    
    def __init__(self, connection_string):
        """
        Initialize telemetry monitor.
        
        Args:
            connection_string: MAVLink connection string
        """
        self.connection_string = connection_string
        self.vehicle = None
        self.telem = None
        self.status_monitor = None
        self.prearm_reporter = None
        self.failsafe_monitor = None
        self.running = False
        
    def connect(self):
        """Connect to vehicle."""
        print(f"[{self._timestamp()}] Connecting to {self.connection_string}...")
        
        try:
            if self.connection_string.startswith(('udp:', 'tcp:')):
                self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            else:
                self.vehicle = connect(self.connection_string, baud=57600, wait_ready=True, timeout=60)
            
            print(f"[{self._timestamp()}] Connected to vehicle")
            print(f"[{self._timestamp()}] Autopilot: {self.vehicle.version}")
            return True
            
        except Exception as e:
            print(f"[{self._timestamp()}] ERROR: Connection failed: {e}")
            return False
    
    def _timestamp(self):
        """Get formatted timestamp."""
        return datetime.now().strftime("%H:%M:%S")
    
    def setup_telemetry(self):
        """Initialize telemetry system."""
        print(f"[{self._timestamp()}] Setting up telemetry...")
        
        # Create telemetry logger
        self.telem = TelemetryLogger(self.vehicle)
        self.telem.start()
        
        # Create monitors
        self.status_monitor = VehicleStatusMonitor(self.vehicle, self.telem)
        self.prearm_reporter = PrearmCheckReporter(self.vehicle, self.telem)
        self.failsafe_monitor = FailsafeMonitor(self.vehicle, self.telem)
        
        # Start status monitor
        self.status_monitor.start()
        
        # Initial reports
        time.sleep(0.5)
        self.telem.info("=== TELEM MONITOR STARTED ===")
        
        # Report failsafe configuration
        self.failsafe_monitor.report_failsafe_config()
        
        # Report initial pre-arm status
        self.prearm_reporter.report_prearm_status()
        
        # Send full initial status
        self.status_monitor.send_full_status()
        
        print(f"[{self._timestamp()}] Telemetry active - logs visible in QGC")
    
    def run(self, status_interval=10):
        """
        Run continuous telemetry monitoring.
        
        Args:
            status_interval: Seconds between full status reports
        """
        self.running = True
        last_status = time.time()
        
        print(f"[{self._timestamp()}] Monitoring started (Ctrl+C to stop)")
        print(f"[{self._timestamp()}] Full status report every {status_interval}s")
        print("-" * 50)
        
        while self.running:
            try:
                # Periodic full status report
                if time.time() - last_status >= status_interval:
                    self.status_monitor.send_full_status()
                    last_status = time.time()
                    
                    # Also print local summary
                    self._print_local_status()
                
                time.sleep(1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[{self._timestamp()}] Monitor error: {e}")
                time.sleep(1)
        
        self.shutdown()
    
    def _print_local_status(self):
        """Print status summary to local console."""
        battery = self.vehicle.battery
        gps = self.vehicle.gps_0
        
        print(f"\n[{self._timestamp()}] --- Local Status ---")
        print(f"  Mode: {self.vehicle.mode.name}")
        print(f"  Armed: {self.vehicle.armed}")
        
        if battery.voltage:
            print(f"  Battery: {battery.voltage:.2f}V ({battery.level}%)")
        
        print(f"  GPS: fix={gps.fix_type}, sats={gps.satellites_visible}")
        print(f"  EKF: {'OK' if self.vehicle.ekf_ok else 'NOT OK'}")
        print(f"  Alt: {self.vehicle.location.global_relative_frame.alt:.1f}m")
        print("-" * 50)
    
    def shutdown(self):
        """Clean shutdown."""
        print(f"\n[{self._timestamp()}] Shutting down...")
        
        self.running = False
        
        if self.telem:
            self.telem.info("=== TELEM MONITOR STOPPED ===")
        
        if self.status_monitor:
            self.status_monitor.stop()
        
        if self.telem:
            self.telem.stop()
        
        if self.vehicle:
            self.vehicle.close()
        
        print(f"[{self._timestamp()}] Shutdown complete")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nInterrupt received...")
    sys.exit(0)


def main():
    """Main entry point."""
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Real-time telemetry monitor - sends logs to QGroundControl',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  SITL:       python3 telemetry_monitor.py --connect udp:127.0.0.1:14551
  USB:        python3 telemetry_monitor.py --connect /dev/ttyACM0
  Telemetry:  python3 telemetry_monitor.py --connect /dev/ttyUSB0
  
Logs appear in QGroundControl's message panel (speech bubble icon).
        """
    )
    parser.add_argument(
        '--connect',
        default='udp:127.0.0.1:14551',
        help='Vehicle connection string'
    )
    parser.add_argument(
        '--interval',
        type=int,
        default=10,
        help='Seconds between full status reports (default: 10)'
    )
    
    args = parser.parse_args()
    
    # Banner
    print("=" * 50)
    print("REAL-TIME TELEMETRY MONITOR")
    print("=" * 50)
    print(f"Connection: {args.connect}")
    print(f"Status interval: {args.interval}s")
    print("=" * 50)
    
    # Create and run monitor
    monitor = TelemetryMonitor(args.connect)
    
    if not monitor.connect():
        print("Failed to connect - exiting")
        sys.exit(1)
    
    monitor.setup_telemetry()
    monitor.run(status_interval=args.interval)


if __name__ == "__main__":
    main()
