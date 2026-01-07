#!/usr/bin/env python3
"""
Autonomous Drone Mission Control System
ArduPilot + DroneKit Implementation

This is the main entry point for waypoint-based autonomous navigation.
Includes real-time telemetry logging to ground station.
"""

import time
import argparse
import threading
from dronekit import VehicleMode

from utils.connection import VehicleConnection
from utils.logger import MissionLogger
from utils.telemetry_logger import (
    TelemetryLogger, 
    VehicleStatusMonitor, 
    PrearmCheckReporter, 
    FailsafeMonitor,
    create_telemetry_system
)
from safety.preflight import PreflightChecks
from safety.abort import SafetyAbort
from mission.waypoint_mission import WaypointMission
from mission.mission_data import MissionData


class MissionController:
    """Main mission control orchestrator."""
    
    def __init__(self, connection_string, enable_telemetry=True):
        """
        Initialize mission controller.
        
        Args:
            connection_string: MAVLink connection string
            enable_telemetry: Whether to enable telemetry logging to ground station
        """
        self.connection_string = connection_string
        self.enable_telemetry = enable_telemetry
        self.vehicle = None
        self.preflight = None
        self.safety = None
        self.mission = None
        
        # Telemetry system components
        self.telem_logger = None
        self.status_monitor = None
        self.prearm_reporter = None
        self.failsafe_monitor = None
        
        # Abort flag and lock for thread-safe abort handling
        self.abort_flag = False
        self.abort_lock = threading.Lock()
    
    def connect(self):
        """Establish connection to vehicle and initialize telemetry."""
        self.vehicle = VehicleConnection.connect_vehicle(self.connection_string)
        if not self.vehicle:
            return False
        
        # Initialize telemetry system
        if self.enable_telemetry:
            self._setup_telemetry()
        
        # Initialize subsystems
        self.preflight = PreflightChecks(self.vehicle)
        self.safety = SafetyAbort(self.vehicle)
        self.mission = WaypointMission(self.vehicle)
        
        return True
    
    def _setup_telemetry(self):
        """Initialize telemetry logging system."""
        MissionLogger.info("Initializing telemetry system...")
        
        try:
            # Create telemetry components
            (self.telem_logger, 
             self.status_monitor, 
             self.prearm_reporter, 
             self.failsafe_monitor) = create_telemetry_system(self.vehicle)
            
            # Start telemetry logger
            self.telem_logger.start()
            
            # Connect MissionLogger to telemetry
            MissionLogger.set_telemetry_logger(self.telem_logger)
            
            # Start status monitor (continuous monitoring)
            self.status_monitor.start()
            
            # Send initial status and failsafe config to ground station
            time.sleep(0.5)  # Let system initialize
            self.telem_logger.info("=== TELEMETRY ACTIVE ===")
            self.failsafe_monitor.report_failsafe_config()
            
            MissionLogger.success("Telemetry system initialized - logs visible in GCS")
            
        except Exception as e:
            MissionLogger.warning(f"Telemetry init failed: {e} - continuing without telemetry")
            self.enable_telemetry = False
    
    def arm_and_takeoff(self, altitude):
        """
        Arm vehicle and takeoff to target altitude in GUIDED mode.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if successful
        """
        MissionLogger.header("ARMING AND TAKEOFF")
        
        # Set mode to GUIDED
        MissionLogger.info("Setting GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        
        # Wait for mode change
        while self.vehicle.mode.name != "GUIDED":
            MissionLogger.info("Waiting for GUIDED mode...")
            time.sleep(1)
        
        MissionLogger.success("GUIDED mode set")
        
        # Arm vehicle
        MissionLogger.info("Arming motors...")
        self.vehicle.armed = True
        
        # Wait for arming
        while not self.vehicle.armed:
            MissionLogger.info("Waiting for arming...")
            time.sleep(1)
        
        MissionLogger.success("Vehicle armed")
        
        # Takeoff
        MissionLogger.state(f"Taking off to {altitude}m...")
        self.vehicle.simple_takeoff(altitude)
        
        # Wait until altitude is reached
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            MissionLogger.info(f"Altitude: {current_alt:.1f}m / {altitude}m")
            
            if current_alt >= altitude * 0.95:
                MissionLogger.success(f"Reached target altitude: {current_alt:.1f}m")
                break
            
            time.sleep(1)
        
        return True
    
    def execute_mission(self):
        """Execute the complete autonomous mission sequence with abort capability."""
        
        try:
            # Report pre-arm check status via telemetry
            if self.enable_telemetry and self.prearm_reporter:
                MissionLogger.info("Sending pre-arm status to ground station...")
                self.prearm_reporter.report_prearm_status()
            
            # Step 1: Pre-flight checks
            if not self.preflight.run_all_checks():
                MissionLogger.error("Pre-flight checks failed - aborting mission")
                return False
            
            # Check abort before starting
            with self.abort_lock:
                if self.abort_flag:
                    MissionLogger.warning("Mission aborted before start")
                    return False
            
            # Step 2: Upload mission
            MissionLogger.header("MISSION UPLOAD")
            waypoints = MissionData.get_waypoints()
            MissionLogger.info(MissionData.get_mission_summary())
            
            if not self.mission.upload_mission(waypoints):
                MissionLogger.error("Mission upload failed - aborting")
                return False
            
            # Step 3: Auto-start countdown (no manual confirmation)
            MissionLogger.header("AUTO-START SEQUENCE")
            MissionLogger.info("Mission ready - starting in 3 seconds...")
            MissionLogger.info(f"Takeoff altitude: {MissionData.TAKEOFF_ALTITUDE}m")
            MissionLogger.info(f"Waypoints: {len(waypoints)} (5-second hover at each)")
            
            for i in range(3, 0, -1):
                with self.abort_lock:
                    if self.abort_flag:
                        MissionLogger.warning("Mission aborted during countdown")
                        return False
                MissionLogger.info(f"Starting in {i}...")
                time.sleep(1)
            
            # Check abort before arming
            with self.abort_lock:
                if self.abort_flag:
                    MissionLogger.warning("Mission aborted before arm")
                    return False
            
            # Step 4: Arm and takeoff in GUIDED mode
            if not self.arm_and_takeoff(MissionData.TAKEOFF_ALTITUDE):
                MissionLogger.error("Takeoff failed - aborting")
                return False
            
            # Check abort after takeoff
            with self.abort_lock:
                if self.abort_flag:
                    MissionLogger.warning("Mission aborted - initiating RTL")
                    self.safety.return_to_launch()
                    return False
            
            # Step 5: Switch to AUTO mode for waypoint navigation
            MissionLogger.header("AUTONOMOUS NAVIGATION")
            if not self.mission.start_mission():
                MissionLogger.error("Failed to start AUTO mode - initiating RTL")
                self.safety.return_to_launch()
                return False
            
            # Step 6: Monitor mission progress with abort checking
            last_waypoint = 0
            while True:
                # Check abort flag
                with self.abort_lock:
                    if self.abort_flag:
                        MissionLogger.warning("Mission aborted during flight - initiating RTL")
                        self.safety.return_to_launch()
                        return False
                
                status = self.mission.get_mission_status()
                current_wp = status['current_waypoint']
                
                # Log waypoint transitions
                if current_wp != last_waypoint:
                    MissionLogger.state(
                        f"Waypoint {current_wp}/{status['total_waypoints']} | "
                        f"Alt: {status['altitude']:.1f}m | "
                        f"Speed: {status['groundspeed']:.1f}m/s"
                    )
                    last_waypoint = current_wp
                
                # Check if mission complete
                if self.mission.monitor_mission():
                    break
                
                time.sleep(1)
            
            # Step 7: Return to launch after mission complete
            MissionLogger.header("MISSION COMPLETE - RETURNING HOME")
            self.safety.return_to_launch()
            
            # Step 8: Monitor landing
            while not self.safety.monitor_landing():
                time.sleep(2)
            
            MissionLogger.success("Mission completed successfully!")
            return True
            time.sleep(2)
            
            MissionLogger.success("Mission completed successfully!")
            return True
            
        except KeyboardInterrupt:
            MissionLogger.warning("\nMission interrupted by user - initiating emergency landing")
            self.safety.emergency_land()
            
            # Wait for landing
            while not self.safety.monitor_landing():
                time.sleep(2)
            
            return False
        
        except Exception as e:
            MissionLogger.error(f"Mission error: {str(e)}")
            MissionLogger.warning("Initiating emergency landing")
            self.safety.emergency_land()
            return False
    
    def shutdown(self):
        """Clean shutdown sequence."""
        MissionLogger.header("SHUTDOWN")
        
        # Stop telemetry system
        if self.enable_telemetry:
            if self.status_monitor:
                self.status_monitor.stop()
            if self.telem_logger:
                self.telem_logger.info("Telemetry shutting down")
                self.telem_logger.stop()
            MissionLogger.set_telemetry_logger(None)
        
        if self.vehicle:
            # Ensure vehicle is disarmed
            if self.vehicle.armed:
                MissionLogger.warning("Vehicle still armed - disarming...")
                self.safety.force_disarm()
            
            # Close connection
            VehicleConnection.close_vehicle(self.vehicle)
        
        MissionLogger.success("Shutdown complete")


def abort_listener(controller):
    """Listen for abort command in separate thread."""
    while not controller.abort_flag:
        try:
            user_input = input().strip().lower()
            if user_input == 'abort':
                with controller.abort_lock:
                    controller.abort_flag = True
                MissionLogger.warning("⚠️  ABORT COMMAND RECEIVED!")
                break
        except:
            break


def main():
    """Main entry point."""
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Autonomous Drone Mission Control')
    parser.add_argument(
        '--connect',
        default='udp:127.0.0.1:14551',
        help='Vehicle connection string (default: udp:127.0.0.1:14551 for SITL)'
    )
    parser.add_argument(
        '--no-telemetry',
        action='store_true',
        help='Disable telemetry logging to ground station'
    )
    args = parser.parse_args()
    
    enable_telemetry = not args.no_telemetry
    
    # Display banner
    MissionLogger.header("AUTONOMOUS DRONE NAVIGATION SYSTEM")
    MissionLogger.info("ArduPilot + DroneKit Mission Control")
    MissionLogger.info(f"Connection: {args.connect}")
    MissionLogger.info(f"Telemetry to GCS: {'ENABLED' if enable_telemetry else 'DISABLED'}")
    MissionLogger.info("Type 'abort' at any time to stop the mission")
    
    # Create controller
    controller = MissionController(args.connect, enable_telemetry=enable_telemetry)
    
    try:
        # Connect to vehicle
        if not controller.connect():
            MissionLogger.error("Failed to connect to vehicle")
            return
        
        # Start abort listener thread
        abort_thread = threading.Thread(target=abort_listener, args=(controller,), daemon=True)
        abort_thread.start()
        
        # Execute mission (will auto-start after pre-flight)
        controller.execute_mission()
        
    finally:
        # Always shutdown cleanly
        controller.shutdown()


if __name__ == "__main__":
    main()
