"""
Telemetry Logger - Sends real-time logs to ground station via MAVLink.

This module enables viewing all system logs in QGroundControl or any MAVLink-compatible
ground station. Logs include:
- Pre-arm check status
- Battery voltage and current
- Failsafe events (battery, GPS, GCS, etc.)
- System status messages
- EKF status
- GPS status
- RC signal status
- Mode changes
- Arm/disarm events
"""

import time
import threading
from datetime import datetime
from pymavlink import mavutil


class TelemetryLogger:
    """
    Sends log messages to ground station via MAVLink STATUSTEXT messages.
    Messages appear in QGroundControl's message panel in real-time.
    """
    
    # MAVLink severity levels (matching QGC display)
    SEVERITY_EMERGENCY = 0  # System is unusable
    SEVERITY_ALERT = 1      # Action must be taken immediately
    SEVERITY_CRITICAL = 2   # Critical conditions
    SEVERITY_ERROR = 3      # Error conditions
    SEVERITY_WARNING = 4    # Warning conditions
    SEVERITY_NOTICE = 5     # Normal but significant
    SEVERITY_INFO = 6       # Informational
    SEVERITY_DEBUG = 7      # Debug messages
    
    def __init__(self, vehicle):
        """
        Initialize telemetry logger.
        
        Args:
            vehicle: DroneKit Vehicle object
        """
        self.vehicle = vehicle
        self._message_queue = []
        self._lock = threading.Lock()
        self._running = False
        self._sender_thread = None
        self._debug = False  # Set True to see local output of sent messages
        
    def start(self):
        """Start the telemetry message sender thread."""
        self._running = True
        self._sender_thread = threading.Thread(target=self._message_sender, daemon=True)
        self._sender_thread.start()
        # Send immediate test message
        self._send_statustext_immediate(self.SEVERITY_INFO, "Telemetry logger started")
        
    def stop(self):
        """Stop the telemetry message sender thread."""
        self._running = False
        if self._sender_thread:
            self._sender_thread.join(timeout=2)
    
    def _message_sender(self):
        """Background thread that sends queued messages."""
        while self._running:
            with self._lock:
                if self._message_queue:
                    severity, message = self._message_queue.pop(0)
                    self._send_statustext(severity, message)
            time.sleep(0.1)  # Don't flood the link
    
    def _send_statustext(self, severity, message):
        """
        Send STATUSTEXT message via MAVLink.
        
        Args:
            severity: MAVLink severity level (0-7)
            message: Text message (max 50 chars)
        """
        try:
            # Truncate message to MAVLink limit (50 chars for STATUSTEXT)
            msg_text = message[:50] if len(message) > 50 else message
            
            # Get the MAVLink connection from DroneKit vehicle
            # Use the underlying _master connection to send raw MAVLink
            mav = self.vehicle._master.mav
            
            # Send STATUSTEXT message
            # This sends to all connected GCS (QGroundControl, Mission Planner, etc.)
            mav.statustext_send(
                severity,
                msg_text.encode('utf-8')
            )
            
            if self._debug:
                print(f"[TELEM->GCS] {message}")
            
        except Exception as e:
            # Fallback - print locally if MAVLink send fails
            print(f"[TELEM LOCAL] {message}")
    
    def _send_statustext_immediate(self, severity, message):
        """Send message immediately without queuing."""
        self._send_statustext(severity, message)
    
    def _queue_message(self, severity, message):
        """Add message to send queue."""
        with self._lock:
            # Limit queue size to prevent memory issues
            if len(self._message_queue) < 100:
                self._message_queue.append((severity, message))
    
    def send_now(self, message, severity=None):
        """Send a message immediately (not queued). Use for critical messages."""
        if severity is None:
            severity = self.SEVERITY_NOTICE
        self._send_statustext_immediate(severity, message)
    
    # Convenience methods for different log levels
    def emergency(self, message):
        """Log emergency message (system unusable)."""
        self._queue_message(self.SEVERITY_EMERGENCY, f"EMERG: {message}")
        
    def alert(self, message):
        """Log alert message (immediate action required)."""
        self._queue_message(self.SEVERITY_ALERT, f"ALERT: {message}")
        
    def critical(self, message):
        """Log critical message."""
        self._queue_message(self.SEVERITY_CRITICAL, f"CRIT: {message}")
        
    def error(self, message):
        """Log error message."""
        self._queue_message(self.SEVERITY_ERROR, f"ERR: {message}")
        
    def warning(self, message):
        """Log warning message."""
        self._queue_message(self.SEVERITY_WARNING, f"WARN: {message}")
        
    def notice(self, message):
        """Log notice message."""
        self._queue_message(self.SEVERITY_NOTICE, message)
        
    def info(self, message):
        """Log info message."""
        self._queue_message(self.SEVERITY_INFO, message)
        
    def debug(self, message):
        """Log debug message."""
        self._queue_message(self.SEVERITY_DEBUG, f"DBG: {message}")


class VehicleStatusMonitor:
    """
    Monitors vehicle status and sends telemetry for important events.
    Tracks failsafes, battery, GPS, EKF, RC, and system status.
    """
    
    def __init__(self, vehicle, telemetry_logger):
        """
        Initialize vehicle status monitor.
        
        Args:
            vehicle: DroneKit Vehicle object
            telemetry_logger: TelemetryLogger instance
        """
        self.vehicle = vehicle
        self.telem = telemetry_logger
        self._running = False
        self._monitor_thread = None
        
        # Track previous states to detect changes
        self._prev_mode = None
        self._prev_armed = None
        self._prev_battery_voltage = None
        self._prev_gps_fix = None
        self._prev_ekf_ok = None
        self._prev_system_status = None
        self._last_heartbeat = time.time()
        
        # Failsafe thresholds
        self.battery_warn_voltage = 11.1  # 3.7V per cell (3S)
        self.battery_critical_voltage = 10.5  # 3.5V per cell
        self.min_satellites = 6
        
        # Register message listeners for failsafe events
        self._setup_listeners()
    
    def _setup_listeners(self):
        """Setup MAVLink message listeners for failsafe events."""
        
        # Store reference to self for use in closures
        monitor_self = self
        
        @self.vehicle.on_message('STATUSTEXT')
        def statustext_listener(self_vehicle, name, message):
            """Forward ArduPilot STATUSTEXT messages."""
            # These are messages from the flight controller itself
            text = message.text.decode('utf-8') if isinstance(message.text, bytes) else message.text
            # Don't re-send our own messages to avoid loops
            if not text.startswith(('EMERG:', 'ALERT:', 'CRIT:', 'ERR:', 'WARN:', 'DBG:', '[TELEM]')):
                severity = message.severity
                fc_msg = f"FC: {text}"
                
                # Map MAVLink severity to logging method
                if severity <= 2:  # EMERGENCY, ALERT, CRITICAL
                    monitor_self.telem.critical(fc_msg)
                elif severity == 3:  # ERROR
                    monitor_self.telem.error(fc_msg)
                elif severity == 4:  # WARNING
                    monitor_self.telem.warning(fc_msg)
                else:  # NOTICE, INFO, DEBUG
                    monitor_self.telem.info(fc_msg)
        
        @self.vehicle.on_message('SYS_STATUS')
        def sys_status_listener(self_vehicle, name, message):
            """Monitor system status for failsafe conditions."""
            # Check for sensor health issues
            sensors_present = message.onboard_control_sensors_present
            sensors_enabled = message.onboard_control_sensors_enabled
            sensors_health = message.onboard_control_sensors_health
            
            # Detect unhealthy sensors
            unhealthy = sensors_enabled & ~sensors_health
            if unhealthy:
                self._report_sensor_issues(unhealthy)
        
        @self.vehicle.on_message('HEARTBEAT')
        def heartbeat_listener(self_vehicle, name, message):
            """Track heartbeat for GCS failsafe detection."""
            self._last_heartbeat = time.time()
            
            # Check system status changes
            if message.system_status != self._prev_system_status:
                self._report_system_status(message.system_status)
                self._prev_system_status = message.system_status
    
    def _report_sensor_issues(self, unhealthy_mask):
        """Report unhealthy sensors."""
        sensors = {
            0x01: "GYRO",
            0x02: "ACCEL", 
            0x04: "MAG",
            0x08: "ABS_PRESS",
            0x10: "DIFF_PRESS",
            0x20: "GPS",
            0x40: "OPTICAL_FLOW",
            0x80: "VISION_POS",
            0x100: "LASER_POS",
            0x200: "EXT_GROUND_TRUTH",
            0x400: "3D_GYRO2",
            0x800: "3D_ACCEL2",
            0x1000: "3D_MAG2",
            0x2000: "GEOFENCE",
            0x4000: "AHRS",
            0x8000: "TERRAIN",
            0x10000: "REV_THROTTLE",
            0x20000: "LOGGING",
            0x40000: "BATTERY",
            0x80000: "PROXIMITY",
            0x100000: "SATCOM",
            0x200000: "PREARM_CHECK",
        }
        
        for mask, name in sensors.items():
            if unhealthy_mask & mask:
                self.telem.warning(f"Sensor UNHEALTHY: {name}")
    
    def _report_system_status(self, status):
        """Report system status changes."""
        status_names = {
            0: "UNINIT",
            1: "BOOT",
            2: "CALIBRATING",
            3: "STANDBY",
            4: "ACTIVE",
            5: "CRITICAL",
            6: "EMERGENCY",
            7: "POWEROFF",
            8: "FLIGHT_TERMINATION",
        }
        
        status_name = status_names.get(status, f"UNKNOWN({status})")
        
        if status == 5:  # CRITICAL
            self.telem.critical(f"SYS_STATUS: {status_name}")
        elif status == 6:  # EMERGENCY
            self.telem.emergency(f"SYS_STATUS: {status_name}")
        else:
            self.telem.info(f"SYS_STATUS: {status_name}")
    
    def start(self):
        """Start the status monitor thread."""
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self.telem.info("Status monitor started")
    
    def stop(self):
        """Stop the status monitor thread."""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
    
    def _monitor_loop(self):
        """Main monitoring loop - checks vehicle status periodically."""
        while self._running:
            try:
                self._check_mode_change()
                self._check_arm_change()
                self._check_battery()
                self._check_gps()
                self._check_ekf()
                self._check_gcs_heartbeat()
                self._check_rc_signal()
            except Exception as e:
                # Replace % with %% to avoid format string errors
                err_msg = str(e)[:30].replace('%', '%%')
                self.telem.error(f"Monitor error: {err_msg}")
            
            time.sleep(1)  # Check every second
    
    def _check_mode_change(self):
        """Detect and report flight mode changes."""
        current_mode = self.vehicle.mode.name
        if current_mode != self._prev_mode:
            if self._prev_mode is not None:
                self.telem.notice(f"MODE: {self._prev_mode} -> {current_mode}")
            self._prev_mode = current_mode
    
    def _check_arm_change(self):
        """Detect and report arm/disarm events."""
        current_armed = self.vehicle.armed
        if current_armed != self._prev_armed:
            if current_armed:
                self.telem.notice("ARMED")
            else:
                self.telem.notice("DISARMED")
            self._prev_armed = current_armed
    
    def _check_battery(self):
        """Check battery status and report issues."""
        battery = self.vehicle.battery
        
        # Use dummy voltage 0.0 if battery object is None
        voltage = battery.voltage if (battery and battery.voltage) else 0.0
        
        # Only process if we have real voltage data (voltage > 0)
        if voltage > 0:
            # Report voltage drops
            if self._prev_battery_voltage is not None:
                voltage_drop = self._prev_battery_voltage - voltage
                if voltage_drop > 0.5:  # Significant drop
                    self.telem.warning(f"BATT DROP: {voltage_drop:.2f}V")
            
            # Check thresholds
            if voltage < self.battery_critical_voltage:
                self.telem.critical(f"BATT CRITICAL: {voltage:.2f}V")
            elif voltage < self.battery_warn_voltage:
                self.telem.warning(f"BATT LOW: {voltage:.2f}V")
            
            self._prev_battery_voltage = voltage
    
    def _check_gps(self):
        """Check GPS status and report issues."""
        gps = self.vehicle.gps_0
        
        # Use dummy values if GPS object is None
        current_fix = gps.fix_type if gps else -1
        satellites = gps.satellites_visible if gps else 0
        
        # Report GPS fix changes
        if current_fix != self._prev_gps_fix:
            fix_names = {-1: "NO_GPS_OBJ", 0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
            fix_name = fix_names.get(current_fix, f"UNKNOWN({current_fix})")
            
            if current_fix < 2:
                self.telem.error(f"GPS: {fix_name} ({satellites} sats)")
            elif current_fix < 3:
                self.telem.warning(f"GPS: {fix_name} ({satellites} sats)")
            else:
                self.telem.info(f"GPS: {fix_name} ({satellites} sats)")
            
            self._prev_gps_fix = current_fix
        
        # Warn if satellites drop below threshold
        if satellites < self.min_satellites and current_fix >= 2:
            self.telem.warning(f"GPS LOW SATS: {satellites}")
    
    def _check_ekf(self):
        """Check EKF status and report issues."""
        current_ekf = self.vehicle.ekf_ok
        
        if current_ekf != self._prev_ekf_ok:
            if current_ekf:
                self.telem.info("EKF: OK")
            else:
                self.telem.error("EKF: NOT OK - Check IMU/GPS")
            self._prev_ekf_ok = current_ekf
    
    def _check_gcs_heartbeat(self):
        """Check for GCS heartbeat timeout (simulated)."""
        # This would be more relevant for real GCS failsafe
        time_since_hb = time.time() - self._last_heartbeat
        if time_since_hb > 5:
            self.telem.warning(f"No FC heartbeat: {time_since_hb:.0f}s")
    
    def _check_rc_signal(self):
        """Check RC signal status."""
        try:
            # Check if RC channels are receiving
            channels = self.vehicle.channels
            if channels and all(v == 0 for v in list(channels.values())[:4]):
                self.telem.warning("RC: No signal detected")
        except:
            pass  # RC might not be available in SITL
    
    def send_full_status(self):
        """Send a complete status report."""
        battery = self.vehicle.battery
        gps = self.vehicle.gps_0
        
        self.telem.info(f"--- STATUS REPORT ---")
        self.telem.info(f"Mode: {self.vehicle.mode.name}")
        self.telem.info(f"Armed: {self.vehicle.armed}")
        
        if battery and battery.voltage:
            self.telem.info(f"Batt: {battery.voltage:.2f}V {battery.level}%")
        
        if battery and battery.current:
            self.telem.info(f"Current: {battery.current:.1f}A")
        
        if gps:
            self.telem.info(f"GPS: fix={gps.fix_type} sats={gps.satellites_visible}")
        
        self.telem.info(f"EKF: {'OK' if self.vehicle.ekf_ok else 'NOT OK'}")
        
        if self.vehicle.location.global_relative_frame:
            self.telem.info(f"Alt: {self.vehicle.location.global_relative_frame.alt:.1f}m")
        
        self.telem.info(f"--- END REPORT ---")


class PrearmCheckReporter:
    """
    Reports pre-arm check status via telemetry.
    Shows exactly why the drone cannot arm.
    """
    
    def __init__(self, vehicle, telemetry_logger):
        """
        Initialize pre-arm check reporter.
        
        Args:
            vehicle: DroneKit Vehicle object
            telemetry_logger: TelemetryLogger instance
        """
        self.vehicle = vehicle
        self.telem = telemetry_logger
    
    def report_prearm_status(self):
        """Report all pre-arm checks and their status."""
        self.telem.info("=== PRE-ARM CHECKS ===")
        
        # Check if armable
        if self.vehicle.is_armable:
            self.telem.info("PREARM: All checks PASSED")
            return True
        
        # Report individual issues
        self._check_gps_prearm()
        self._check_ekf_prearm()
        self._check_battery_prearm()
        self._check_safety_switch()
        self._check_compass()
        self._check_accel()
        
        self.telem.warning("PREARM: Checks FAILED")
        return False
    
    def _check_gps_prearm(self):
        """Check GPS pre-arm requirements."""
        gps = self.vehicle.gps_0
        
        # Use dummy values if GPS object is None (-1 fix won't trigger failures)
        fix_type = gps.fix_type if gps else -1
        satellites = gps.satellites_visible if gps else 0
        
        # Only process if we have real GPS data (fix_type >= 0)
        if fix_type >= 0:
            if fix_type < 3:
                self.telem.error(f"PREARM FAIL: GPS (fix={fix_type}, need 3)")
            else:
                self.telem.info(f"PREARM OK: GPS ({satellites} sats)")
    
    def _check_ekf_prearm(self):
        """Check EKF pre-arm status."""
        if not self.vehicle.ekf_ok:
            self.telem.error("PREARM FAIL: EKF not ready")
        else:
            self.telem.info("PREARM OK: EKF")
    
    def _check_battery_prearm(self):
        """Check battery pre-arm status."""
        battery = self.vehicle.battery
        
        # Use dummy voltage 0.0 if battery object is None
        voltage = battery.voltage if (battery and battery.voltage) else 0.0
        
        if voltage > 0 and voltage < 10.0:
            self.telem.error(f"PREARM FAIL: Battery ({voltage:.1f}V)")
        elif voltage > 0:
            self.telem.info(f"PREARM OK: Battery ({voltage:.1f}V)")
    
    def _check_safety_switch(self):
        """Check safety switch status."""
        try:
            # Safety switch status (if available)
            if hasattr(self.vehicle, 'parameters'):
                brd_safety = self.vehicle.parameters.get('BRD_SAFETY_DEFLT', None)
                if brd_safety == 1:
                    self.telem.info("PREARM: Safety switch may need press")
        except:
            pass
    
    def _check_compass(self):
        """Check compass calibration."""
        try:
            heading = self.vehicle.heading
            if heading is not None:
                self.telem.info(f"PREARM OK: Compass (hdg={heading}°)")
        except:
            self.telem.warning("PREARM: Compass check unavailable")
    
    def _check_accel(self):
        """Check accelerometer status."""
        try:
            # Attitude data indicates working accelerometers
            attitude = self.vehicle.attitude
            if attitude:
                self.telem.info("PREARM OK: Accel (attitude valid)")
        except:
            pass


class FailsafeMonitor:
    """
    Dedicated failsafe event monitoring and reporting.
    Monitors battery, GPS, GCS, fence, and other failsafes.
    """
    
    def __init__(self, vehicle, telemetry_logger):
        """
        Initialize failsafe monitor.
        
        Args:
            vehicle: DroneKit Vehicle object
            telemetry_logger: TelemetryLogger instance
        """
        self.vehicle = vehicle
        self.telem = telemetry_logger
        
        # Set defaults first
        self.batt_fs_enabled = False
        self.batt_fs_voltage = 10.5
        self.batt_crt_voltage = 10.0
        self.gps_fs_enabled = True
        self.gcs_fs_enabled = False
        self.gcs_timeout = 5
        self.thr_fs_enabled = True
        self.fence_enabled = False
        self.fence_alt_max = 100
        self.fence_radius = 300
        self.params_loaded = False
        
        # Try to load actual params
        self._setup_failsafe_params()
    
    def _setup_failsafe_params(self):
        """Read failsafe parameters from vehicle."""
        try:
            params = self.vehicle.parameters
            
            # Battery failsafe
            batt_act = params.get('BATT_FS_CRT_ACT', None)
            if batt_act is not None:
                self.batt_fs_enabled = batt_act > 0
            
            batt_low = params.get('BATT_LOW_VOLT', None)
            if batt_low is not None:
                self.batt_fs_voltage = batt_low
            
            batt_crt = params.get('BATT_CRT_VOLT', None)
            if batt_crt is not None:
                self.batt_crt_voltage = batt_crt
            
            # GPS failsafe  
            gps_en = params.get('FS_GPS_ENABLE', None)
            if gps_en is not None:
                self.gps_fs_enabled = gps_en > 0
            
            # GCS failsafe
            gcs_en = params.get('FS_GCS_ENABLE', None)
            if gcs_en is not None:
                self.gcs_fs_enabled = gcs_en > 0
            
            gcs_to = params.get('FS_GCS_TIMEOUT', None)
            if gcs_to is not None:
                self.gcs_timeout = gcs_to
            
            # Radio failsafe
            thr_en = params.get('FS_THR_ENABLE', None)
            if thr_en is not None:
                self.thr_fs_enabled = thr_en > 0
            
            # Geofence
            fence_en = params.get('FENCE_ENABLE', None)
            if fence_en is not None:
                self.fence_enabled = fence_en > 0
            
            fence_alt = params.get('FENCE_ALT_MAX', None)
            if fence_alt is not None:
                self.fence_alt_max = fence_alt
            
            fence_rad = params.get('FENCE_RADIUS', None)
            if fence_rad is not None:
                self.fence_radius = fence_rad
            
            self.params_loaded = True
            self.telem.info("Failsafe params loaded")
            
        except Exception as e:
            self.telem.warning(f"FS params: using defaults")
    
    def report_failsafe_config(self):
        """Report current failsafe configuration."""
        self.telem.info("=== FAILSAFE CONFIG ===")
        
        # Battery
        if self.batt_fs_enabled:
            self.telem.info(f"BATT FS: ON (low={self.batt_fs_voltage}V)")
        else:
            self.telem.warning("BATT FS: OFF")
        
        # GPS
        if self.gps_fs_enabled:
            self.telem.info("GPS FS: ON")
        else:
            self.telem.warning("GPS FS: OFF")
        
        # GCS
        if self.gcs_fs_enabled:
            self.telem.info(f"GCS FS: ON ({self.gcs_timeout}s timeout)")
        else:
            self.telem.info("GCS FS: OFF")
        
        # Radio
        if self.thr_fs_enabled:
            self.telem.info("RC FS: ON")
        else:
            self.telem.warning("RC FS: OFF")
        
        # Fence
        if self.fence_enabled:
            self.telem.info(f"FENCE: ON (alt={self.fence_alt_max}m, rad={self.fence_radius}m)")
        else:
            self.telem.info("FENCE: OFF")
        
        self.telem.info("=== END CONFIG ===")


class RadioTelemetryLogger:
    """
    Sends telemetry logs to ground station via serial radio (LoRa/3DR).
    Use this when communicating with tx_commands.py on a laptop.
    """
    
    # Log levels
    LEVEL_EMERGENCY = 0
    LEVEL_ALERT = 1
    LEVEL_CRITICAL = 2
    LEVEL_ERROR = 3
    LEVEL_WARNING = 4
    LEVEL_NOTICE = 5
    LEVEL_INFO = 6
    LEVEL_DEBUG = 7
    
    LEVEL_NAMES = {
        0: "EMERG",
        1: "ALERT", 
        2: "CRIT",
        3: "ERROR",
        4: "WARN",
        5: "NOTICE",
        6: "INFO",
        7: "DEBUG"
    }
    
    def __init__(self, radio_serial, prefix="[TELEM]"):
        """
        Initialize radio telemetry logger.
        
        Args:
            radio_serial: Serial port object for radio communication
            prefix: Prefix for telemetry messages
        """
        self.radio = radio_serial
        self.prefix = prefix
        self._lock = threading.Lock()
        self._enabled = True
        self._min_level = self.LEVEL_INFO  # Only send INFO and above by default
        self._error_count = 0
        self._max_errors_before_suppress = 5
        self._suppressed = False
        
    def set_min_level(self, level):
        """Set minimum log level to send (0=all, 7=debug only)."""
        self._min_level = level
        
    def enable(self, enabled=True):
        """Enable or disable telemetry sending."""
        self._enabled = enabled
        if enabled:
            # Reset error state when re-enabled
            self._error_count = 0
            self._suppressed = False
        
    def set_radio(self, radio_serial):
        """Update the radio serial object (used after reconnection)."""
        with self._lock:
            self.radio = radio_serial
            self._error_count = 0
            self._suppressed = False
        
    def _send(self, level, message):
        """Send a telemetry message via radio."""
        if not self._enabled or not self.radio or self._suppressed:
            return
        
        if level > self._min_level:
            return  # Skip messages below minimum level
            
        try:
            with self._lock:
                # Add timestamp for sync verification with TX logs
                from datetime import datetime
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
                level_name = self.LEVEL_NAMES.get(level, "INFO")
                # Format: [HH:MM:SS.mmm] [TELEM][LEVEL] message
                line = f"[{timestamp}] {self.prefix}[{level_name}] {message}\n"
                self.radio.write(line.encode())
                # Reset error count on success
                self._error_count = 0
        except (OSError, IOError) as e:
            # I/O errors indicate radio disconnection
            self._error_count += 1
            if self._error_count >= self._max_errors_before_suppress:
                if not self._suppressed:
                    print(f"[RadioTelem] Radio disconnected - suppressing telemetry until reconnect")
                    self._suppressed = True
            elif self._error_count == 1:
                print(f"[RadioTelem] Send failed: {e}")
        except Exception as e:
            print(f"[RadioTelem] Send failed: {e}")
    
    # Convenience methods matching TelemetryLogger interface
    def emergency(self, message):
        """Log emergency message."""
        self._send(self.LEVEL_EMERGENCY, message)
        
    def alert(self, message):
        """Log alert message."""
        self._send(self.LEVEL_ALERT, message)
        
    def critical(self, message):
        """Log critical message."""
        self._send(self.LEVEL_CRITICAL, message)
        
    def error(self, message):
        """Log error message."""
        self._send(self.LEVEL_ERROR, message)
        
    def warning(self, message):
        """Log warning message."""
        self._send(self.LEVEL_WARNING, message)
        
    def notice(self, message):
        """Log notice message."""
        self._send(self.LEVEL_NOTICE, message)
        
    def info(self, message):
        """Log info message."""
        self._send(self.LEVEL_INFO, message)
        
    def debug(self, message):
        """Log debug message."""
        self._send(self.LEVEL_DEBUG, message)
    
    # Start/stop methods for compatibility
    def start(self):
        """Start telemetry (no-op for radio, always ready)."""
        self._send(self.LEVEL_INFO, "Telemetry active")
        
    def stop(self):
        """Stop telemetry."""
        self._enabled = False


class RadioStatusMonitor:
    """
    Monitors vehicle status and sends telemetry via radio.
    Simplified version for radio communication.
    """
    
    def __init__(self, vehicle, radio_telem):
        """
        Initialize radio status monitor.
        
        Args:
            vehicle: DroneKit Vehicle object
            radio_telem: RadioTelemetryLogger instance
        """
        self.vehicle = vehicle
        self.telem = radio_telem
        self._running = False
        self._monitor_thread = None
        self._interval = 5  # Status update interval in seconds
        
        # Track previous states
        self._prev_mode = None
        self._prev_armed = None
        self._prev_voltage = None
        self._prev_gps_fix = None
        
    def start(self, interval=5):
        """Start monitoring."""
        self._interval = interval
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self.telem.info("Status monitor started")
        
    def stop(self):
        """Stop monitoring."""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
            
    def _monitor_loop(self):
        """Main monitoring loop."""
        last_status = 0
        
        while self._running:
            try:
                # Check for state changes (always)
                self._check_mode()
                self._check_armed()
                self._check_battery()
                self._check_gps()
                
                # Periodic full status
                if time.time() - last_status >= self._interval:
                    self.send_status()
                    last_status = time.time()
                    
            except Exception as e:
                # Replace % with %% to avoid format string errors when exception contains %
                err_msg = str(e)[:25].replace('%', '%%')
                self.telem.error(f"Monitor err: {err_msg}")
                
            time.sleep(1)
    
    def _check_mode(self):
        """Check for mode changes."""
        mode = self.vehicle.mode.name
        if mode != self._prev_mode:
            if self._prev_mode is not None:
                self.telem.notice(f"MODE: {self._prev_mode}->{mode}")
            self._prev_mode = mode
            
    def _check_armed(self):
        """Check for arm/disarm."""
        armed = self.vehicle.armed
        if armed != self._prev_armed:
            if armed:
                self.telem.notice("ARMED")
            else:
                self.telem.notice("DISARMED")
            self._prev_armed = armed
            
    def _check_battery(self):
        """Check battery voltage."""
        batt = self.vehicle.battery
        # Use dummy voltage 0.0 if battery object is None (very low threshold won't trigger alerts)
        voltage = batt.voltage if (batt and batt.voltage) else 0.0
        
        if voltage > 0:  # Only process if we have real voltage data
            # Alert on low battery
            if voltage < 10.5:
                self.telem.critical(f"BATT CRITICAL: {voltage:.1f}V")
            elif voltage < 11.1:
                self.telem.warning(f"BATT LOW: {voltage:.1f}V")
            
            # Alert on voltage drop
            if self._prev_voltage and (self._prev_voltage - voltage) > 0.5:
                self.telem.warning(f"BATT DROP: {self._prev_voltage:.1f}->{voltage:.1f}V")
            
            self._prev_voltage = voltage
            
    def _check_gps(self):
        """Check GPS status."""
        gps = self.vehicle.gps_0
        # Use dummy values if GPS object is None or attributes are None
        fix_type = gps.fix_type if (gps and gps.fix_type is not None) else -1
        satellites = gps.satellites_visible if (gps and gps.satellites_visible is not None) else 0
        
        if fix_type != self._prev_gps_fix:
            fix_names = {-1: "NO_GPS_OBJ", 0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLT", 6: "RTK_FIX"}
            fix_name = fix_names.get(fix_type, f"?{fix_type}")
            
            if fix_type < 2:
                self.telem.error(f"GPS: {fix_name} ({satellites}sat)")
            elif fix_type < 3:
                self.telem.warning(f"GPS: {fix_name} ({satellites}sat)")
            else:
                self.telem.info(f"GPS: {fix_name} ({satellites}sat)")
            
            self._prev_gps_fix = fix_type
    
    def send_status(self):
        """Send full status update."""
        batt = self.vehicle.battery
        gps = self.vehicle.gps_0
        
        # Compact status line with dummy thresholds (0.0V, -1 fix type)
        batt_voltage = batt.voltage if (batt and batt.voltage is not None) else 0.0
        batt_str = f"{batt_voltage:.1f}V" if batt_voltage > 0 else "?V"
        
        fix_type = gps.fix_type if (gps and gps.fix_type is not None) else -1
        satellites = gps.satellites_visible if (gps and gps.satellites_visible is not None) else 0
        gps_str = f"{fix_type}({satellites})" if gps else "No GPS"
        
        alt_str = "?m"
        if self.vehicle.location.global_relative_frame:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt is not None:
                alt_str = f"{alt:.1f}m"
        
        status = f"M:{self.vehicle.mode.name} A:{'Y' if self.vehicle.armed else 'N'} B:{batt_str} G:{gps_str} H:{alt_str}"
        self.telem.info(status)


def create_telemetry_system(vehicle):
    """
    Factory function to create complete telemetry system.
    
    Args:
        vehicle: DroneKit Vehicle object
        
    Returns:
        tuple: (TelemetryLogger, VehicleStatusMonitor, PrearmCheckReporter, FailsafeMonitor)
    """
    telem_logger = TelemetryLogger(vehicle)
    status_monitor = VehicleStatusMonitor(vehicle, telem_logger)
    prearm_reporter = PrearmCheckReporter(vehicle, telem_logger)
    failsafe_monitor = FailsafeMonitor(vehicle, telem_logger)
    
    return telem_logger, status_monitor, prearm_reporter, failsafe_monitor


def create_radio_telemetry_system(vehicle, radio_serial):
    """
    Factory function to create radio-based telemetry system.
    Use this when sending telemetry to tx_commands.py via LoRa/3DR radio.
    
    Args:
        vehicle: DroneKit Vehicle object
        radio_serial: Serial port object for radio
        
    Returns:
        tuple: (RadioTelemetryLogger, RadioStatusMonitor)
    """
    radio_telem = RadioTelemetryLogger(radio_serial)
    status_monitor = RadioStatusMonitor(vehicle, radio_telem)
    
    return radio_telem, status_monitor
