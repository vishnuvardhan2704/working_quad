#!/usr/bin/env python3
"""
Visual Servoing Module for Precision Drone Delivery

Uses Intel RealSense D435 camera to:
1. Detect humans using YOLO
2. Center the drone exactly over the target person
3. Perform controlled descent while maintaining centering
4. Hover for delivery and log completion

Flow:
1. Fly to approximate GPS location (from scouting drone) at 15m
2. Detect human and calculate pixel offset from frame center
3. Translate pixel offset to real-world movement commands
4. Adjust drone position until human is centered
5. Descend to 6m while continuously re-centering
6. Hover 5 seconds for delivery
7. Send completion log to GCS
8. Proceed to next waypoint

Author: DART Team
Date: January 2026
"""

import os
import sys
import time
import math
import threading
import numpy as np
from datetime import datetime

# Add paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NIDAR_DIR = os.path.join(SCRIPT_DIR, 'nidar')
sys.path.insert(0, NIDAR_DIR)

# Try imports
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    print("[ERROR] DroneKit not available")

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[WARN] RealSense not available, will try OpenCV camera")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[WARN] OpenCV not available")

try:
    from yolo_detector import YOLODetector
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[WARN] YOLO detector not available")

# Try to import RPi.GPIO for stepper motor control
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO not available - delivery motor disabled")


class StepperMotorController:
    """
    NEMA 17 Stepper Motor Controller for Package Delivery Mechanism.
    
    Wiring (RPi to A4988/DRV8825 Driver):
        GPIO 18 -> STEP
        GPIO 23 -> DIR (Direction)
        GPIO 24 -> ENABLE (Active LOW)
        RESET & SLEEP -> Shorted together on driver
        MS1, MS2, MS3 -> GND (Full step mode)
        3.3V -> VDD
        GND -> GND
    """
    
    # GPIO Pin assignments
    STEP_PIN = 18
    DIR_PIN = 23
    ENABLE_PIN = 24
    
    # Motor parameters - matched to test_stepper_motor.py working config
    # NEMA 17 = 200 steps/revolution (1.8° per step) in full-step mode
    MICROSTEP_MODE = 1       # 1=full, 2=half, 4=quarter, 8=eighth, 16=sixteenth
    STEPS_PER_REV = 200 * MICROSTEP_MODE  # 200 steps for full step mode
    RELEASE_DEGREES = 180    # Degrees to rotate for package release (half rotation)
    STEP_DELAY = 0.003       # 3ms delay between steps (matched to working test code)
    
    def __init__(self, logger=None, init_gpio=False):
        """
        Initialize stepper motor controller.
        
        Args:
            logger: Logging function
            init_gpio: If False, GPIO is only initialized when needed (default: False)
                       This keeps motor completely off until delivery time
        """
        self.log = logger if logger else print
        self.initialized = False
        self.gpio_setup = False
        
        if not GPIO_AVAILABLE:
            self.log("[WARN] GPIO not available - stepper motor disabled")
            return
        
        if init_gpio:
            self._setup_gpio()
    
    def _setup_gpio(self):
        """Setup GPIO pins for motor control."""
        if self.gpio_setup:
            return True
            
        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configure pins as outputs
            GPIO.setup(self.STEP_PIN, GPIO.OUT)
            GPIO.setup(self.DIR_PIN, GPIO.OUT)
            GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
            
            # Initialize states - motor DISABLED
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            GPIO.output(self.DIR_PIN, GPIO.LOW)
            GPIO.output(self.ENABLE_PIN, GPIO.HIGH)  # HIGH = Disabled (active LOW)
            
            self.gpio_setup = True
            self.initialized = True
            self.log("[OK] Stepper motor GPIO initialized (motor disabled)")
            return True
            
        except Exception as e:
            self.log(f"[ERROR] Stepper motor GPIO init failed: {e}")
            self.initialized = False
            return False
    
    def enable_motor(self):
        """Enable the motor driver (ENABLE pin is active LOW)."""
        # Setup GPIO on first use if not already done
        if not self.gpio_setup:
            if not self._setup_gpio():
                return False
        
        if not self.initialized:
            return False
        GPIO.output(self.ENABLE_PIN, GPIO.LOW)  # Active LOW
        time.sleep(0.01)  # Small delay for driver to enable
        self.log("[MOTOR] Enabled")
        return True
    
    def disable_motor(self):
        """Disable the motor driver to save power and reduce heat."""
        if not self.initialized:
            return
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
        self.log("[MOTOR] Disabled")
    
    def set_direction(self, clockwise=True):
        """Set motor rotation direction."""
        if not self.initialized:
            return
        GPIO.output(self.DIR_PIN, GPIO.HIGH if clockwise else GPIO.LOW)
        direction = "Clockwise" if clockwise else "Counter-Clockwise"
        self.log(f"[MOTOR] Direction: {direction}")
    
    def step(self, steps=1, delay=None):
        """
        Execute specified number of steps.
        
        Args:
            steps: Number of steps to execute
            delay: Delay between steps (seconds), default is STEP_DELAY
        """
        if not self.initialized:
            return False
        
        if delay is None:
            delay = self.STEP_DELAY
        
        self.log(f"[MOTOR] Executing {steps} steps (delay: {delay*1000:.1f}ms)...")
        
        for i in range(steps):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(delay / 2)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(delay / 2)
            
            # Progress indicator every 50 steps
            if (i + 1) % 50 == 0:
                self.log(f"     Step {i + 1}/{steps}")
        
        self.log(f"[MOTOR] Completed {steps} steps")
        return True
    
    def rotate(self, rotations=1, clockwise=True):
        """
        Rotate the motor by specified number of full rotations.
        
        Args:
            rotations: Number of full rotations
            clockwise: Direction of rotation
        """
        # Enable motor (this also sets up GPIO on first use)
        if not self.enable_motor():
            self.log("[WARN] Motor not available - cannot rotate")
            return False
        
        total_steps = int(rotations * self.STEPS_PER_REV)
        self.set_direction(clockwise)
        
        self.log(f"[MOTOR] Rotating {rotations} turns ({total_steps} steps)...")
        success = self.step(total_steps)
        
        self.disable_motor()
        return success
    
    def release_package(self):
        """
        Execute package release sequence.
        Rotates motor to open delivery mechanism and release package.
        
        Returns:
            bool: True if package release executed successfully
        """
        self.log("[DELIVERY] Releasing package...")
        
        # Enable motor (this also sets up GPIO on first use)
        if not self.enable_motor():
            self.log("[WARN] Motor not available - simulating package release")
            time.sleep(1.0)  # Simulate release time
            return False
        
        # Set direction (clockwise to release)
        self.set_direction(clockwise=True)
        
        # Execute release rotation (180 degrees = half rotation)
        total_steps = int((self.RELEASE_DEGREES / 360.0) * self.STEPS_PER_REV)
        self.log(f"[DELIVERY] Rotating {self.RELEASE_DEGREES}° ({total_steps} steps) to release...")
        
        success = self.step(total_steps)
        
        # Brief pause
        time.sleep(0.5)
        
        # Disable motor
        self.disable_motor()
        
        if success:
            self.log("[DELIVERY] Package released successfully!")
        else:
            self.log("[ERROR] Package release failed")
        
        return success
    
    def rotate_degrees(self, degrees, clockwise=True):
        """
        Rotate the motor by specified degrees.
        
        Args:
            degrees: Degrees to rotate (e.g., 90, 180, 360)
            clockwise: Direction of rotation
            
        Returns:
            bool: True if rotation completed
        """
        # Enable motor (this also sets up GPIO on first use)
        if not self.enable_motor():
            self.log("[WARN] Motor not available - cannot rotate")
            return False
        
        total_steps = int((degrees / 360.0) * self.STEPS_PER_REV)
        self.set_direction(clockwise)
        
        self.log(f"[MOTOR] Rotating {degrees}° ({total_steps} steps) {'CW' if clockwise else 'CCW'}...")
        success = self.step(total_steps)
        
        self.disable_motor()
        return success
    
    def retract(self):
        """
        Retract/reset the delivery mechanism after release.
        Rotates motor in reverse to reset position (180 degrees back).
        """
        if not self.initialized:
            return False
        
        self.log("[DELIVERY] Retracting mechanism...")
        
        self.enable_motor()
        self.set_direction(clockwise=False)  # Reverse direction
        
        total_steps = int((self.RELEASE_DEGREES / 360.0) * self.STEPS_PER_REV)
        success = self.step(total_steps)
        
        self.disable_motor()
        
        if success:
            self.log("[DELIVERY] Mechanism retracted")
        
        return success
    
    def cleanup(self):
        """Clean up GPIO resources."""
        if GPIO_AVAILABLE:
            try:
                self.disable_motor()
                GPIO.cleanup([self.STEP_PIN, self.DIR_PIN, self.ENABLE_PIN])
                self.log("[OK] Stepper motor GPIO cleaned up")
            except Exception as e:
                self.log(f"[WARN] GPIO cleanup error: {e}")


class VisualServoController:
    """
    Visual servoing controller for precision positioning over human targets.
    
    Uses pixel-based error calculation to translate camera frame offset
    into real-world drone movement commands.
    """
    
    # Camera parameters for Intel RealSense D435 at 640x480
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    FRAME_CENTER_X = FRAME_WIDTH // 2   # 320
    FRAME_CENTER_Y = FRAME_HEIGHT // 2  # 240
    
    # Horizontal FOV of RealSense D435 (in degrees)
    HFOV_DEG = 87.0
    VFOV_DEG = 58.0
    
    # Altitude stages (meters)
    APPROACH_ALTITUDE = 15.0  # Initial approach altitude
    DELIVERY_ALTITUDE = 6.0   # Final delivery altitude
    DESCENT_STEP = 1.0        # Descend 1 meter at a time
    
    # Centering thresholds (pixels from center)
    COARSE_THRESHOLD = 50    # Acceptable at 15m (coarse centering)
    FINE_THRESHOLD = 20      # Required at 6m (fine centering)
    
    # Movement parameters
    MAX_VELOCITY = 1.0       # Max horizontal velocity (m/s)
    MIN_VELOCITY = 0.2       # Min velocity for fine adjustments
    POSITION_GAIN = 0.003    # Proportional gain for pixel-to-velocity
    
    # Timing
    HOVER_TIME = 5.0         # Seconds to hover for delivery
    CENTERING_TIMEOUT = 60   # Max seconds to attempt centering
    DETECTION_TIMEOUT = 30   # Max seconds to wait for human detection
    
    def __init__(self, vehicle, detector=None, logger=None, send_response=None):
        """
        Initialize visual servo controller.
        
        Args:
            vehicle: DroneKit vehicle object
            detector: YOLODetector instance (optional, will create if None)
            logger: Logging function (optional)
            send_response: Callback to send messages to GCS (optional)
        """
        self.vehicle = vehicle
        self.detector = detector
        self.log = logger if logger else self._default_log
        self.send_gcs = send_response  # Callback to send logs to GCS
        
        # Camera pipeline (RealSense or OpenCV)
        self.pipeline = None
        self.config = None
        self.cv_camera = None
        self.use_realsense = REALSENSE_AVAILABLE
        self.camera_running = False
        
        # State tracking
        self.current_target = None  # (lat, lon) of approximate target
        self.target_index = 0       # Which delivery target we're on
        self.abort_flag = False
        self.mission_running = False
        
        # Delivery log
        self.delivery_log = []
        
        # Stepper motor for package delivery
        self.motor = StepperMotorController(logger=self._motor_log)
        
        # Calculate meters-per-pixel at different altitudes
        self._calculate_scale_factors()
    
    def _motor_log(self, message):
        """Logger wrapper for motor controller."""
        self.log("info", message)
        
    def _default_log(self, level, message):
        """Default logger if none provided."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level.upper()}] {message}")
        
    def _calculate_scale_factors(self):
        """
        Pre-calculate meters-per-pixel at various altitudes.
        
        At altitude h, with horizontal FOV θ:
        Ground width visible = 2 * h * tan(θ/2)
        Meters per pixel = ground_width / frame_width
        """
        self.scale_factors = {}
        hfov_rad = math.radians(self.HFOV_DEG)
        vfov_rad = math.radians(self.VFOV_DEG)
        
        for alt in range(1, 21):  # 1m to 20m
            ground_width = 2 * alt * math.tan(hfov_rad / 2)
            ground_height = 2 * alt * math.tan(vfov_rad / 2)
            
            self.scale_factors[alt] = {
                'x': ground_width / self.FRAME_WIDTH,   # meters per pixel (horizontal)
                'y': ground_height / self.FRAME_HEIGHT  # meters per pixel (vertical)
            }
            
        self.log("info", f"Scale factors calculated (15m: {self.scale_factors[15]['x']:.3f} m/px)")
        
    def get_meters_per_pixel(self, altitude):
        """Get meters-per-pixel scale factor for given altitude."""
        alt_int = max(1, min(20, int(round(altitude))))
        return self.scale_factors.get(alt_int, self.scale_factors[15])
        
    def start_camera(self):
        """Initialize and start camera (RealSense or OpenCV fallback)."""
        # Try RealSense first
        if REALSENSE_AVAILABLE:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                
                # Configure color stream
                self.config.enable_stream(
                    rs.stream.color, 
                    self.FRAME_WIDTH, 
                    self.FRAME_HEIGHT, 
                    rs.format.bgr8, 
                    30
                )
                
                # Start streaming
                self.pipeline.start(self.config)
                self.use_realsense = True
                self.camera_running = True
                
                # Warm up camera
                for _ in range(10):
                    self.pipeline.wait_for_frames()
                    
                self.log("success", "RealSense camera started")
                return True
                
            except Exception as e:
                self.log("warning", f"RealSense failed: {e}, trying OpenCV...")
                self.use_realsense = False
        
        # Fallback to OpenCV camera
        if CV2_AVAILABLE:
            try:
                self.cv_camera = cv2.VideoCapture(0)
                if self.cv_camera.isOpened():
                    self.cv_camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
                    self.cv_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
                    self.camera_running = True
                    self.log("success", "OpenCV camera started")
                    return True
                else:
                    self.log("error", "Failed to open OpenCV camera")
                    return False
            except Exception as e:
                self.log("error", f"OpenCV camera failed: {e}")
                return False
                
        self.log("error", "No camera available")
        return False
            
    def stop_camera(self):
        """Stop camera."""
        if self.use_realsense and self.pipeline and self.camera_running:
            try:
                self.pipeline.stop()
                self.log("info", "RealSense camera stopped")
            except Exception as e:
                self.log("error", f"Error stopping RealSense: {e}")
        elif self.cv_camera:
            try:
                self.cv_camera.release()
                self.log("info", "OpenCV camera stopped")
            except Exception as e:
                self.log("error", f"Error stopping OpenCV camera: {e}")
                
        self.camera_running = False
                
    def get_frame(self):
        """Capture a frame from camera."""
        if not self.camera_running:
            return None
            
        try:
            if self.use_realsense:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    return None
                    
                return np.asanyarray(color_frame.get_data())
            else:
                ret, frame = self.cv_camera.read()
                return frame if ret else None
                
        except Exception as e:
            self.log("error", f"Frame capture error: {e}")
            return None
            
    def detect_human(self, frame):
        """
        Detect human in frame and return bounding box center.
        
        Returns:
            tuple: (center_x, center_y, confidence, bbox) or None if not detected
        """
        if self.detector is None:
            if YOLO_AVAILABLE:
                try:
                    self.detector = YOLODetector()
                except Exception as e:
                    self.log("error", f"Failed to initialize YOLO: {e}")
                    return None
            else:
                self.log("error", "YOLO detector not available")
                return None
                
        try:
            detections = self.detector.detect(frame)
            
            if not detections:
                return None
                
            # Find the largest/most confident human detection
            best_detection = None
            best_score = 0
            
            for det in detections:
                # Detection format: (x1, y1, x2, y2, confidence, class_id)
                if len(det) >= 5:
                    x1, y1, x2, y2, conf = det[:5]
                    
                    # Calculate area as tiebreaker
                    area = (x2 - x1) * (y2 - y1)
                    score = conf * 0.7 + (area / (self.FRAME_WIDTH * self.FRAME_HEIGHT)) * 0.3
                    
                    if score > best_score:
                        best_score = score
                        best_detection = det
                        
            if best_detection is None:
                return None
                
            x1, y1, x2, y2, conf = best_detection[:5]
            
            # Calculate center of bounding box
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            return (center_x, center_y, conf, (x1, y1, x2, y2))
            
        except Exception as e:
            self.log("error", f"Detection error: {e}")
            return None
            
    def calculate_offset(self, human_x, human_y):
        """
        Calculate pixel offset from frame center.
        
        Positive X offset = human is to the RIGHT of center (drone should move RIGHT)
        Positive Y offset = human is BELOW center (drone should move FORWARD)
        
        Note: In camera frame, Y increases downward
              In drone NED frame, positive X is forward (North)
        
        Returns:
            tuple: (offset_x, offset_y) in pixels
        """
        offset_x = human_x - self.FRAME_CENTER_X  # Positive = right
        offset_y = human_y - self.FRAME_CENTER_Y  # Positive = down/forward
        
        return (offset_x, offset_y)
        
    def calculate_velocity_command(self, offset_x, offset_y, altitude):
        """
        Convert pixel offset to velocity commands.
        
        Uses proportional control with altitude-based scaling.
        
        Args:
            offset_x: Horizontal pixel offset (positive = right)
            offset_y: Vertical pixel offset (positive = down/forward)
            altitude: Current altitude in meters
            
        Returns:
            tuple: (velocity_north, velocity_east) in m/s
        """
        scale = self.get_meters_per_pixel(altitude)
        
        # Convert pixel offset to meters
        error_east = offset_x * scale['x']   # Positive = move east/right
        error_north = -offset_y * scale['y']  # Negative because camera Y is inverted
        
        # Proportional control with gain
        vel_east = error_east * self.POSITION_GAIN * 10
        vel_north = error_north * self.POSITION_GAIN * 10
        
        # Clamp velocities
        vel_east = max(-self.MAX_VELOCITY, min(self.MAX_VELOCITY, vel_east))
        vel_north = max(-self.MAX_VELOCITY, min(self.MAX_VELOCITY, vel_north))
        
        # Apply minimum velocity threshold for fine movements
        if abs(vel_east) < self.MIN_VELOCITY and abs(vel_east) > 0.05:
            vel_east = self.MIN_VELOCITY * (1 if vel_east > 0 else -1)
        if abs(vel_north) < self.MIN_VELOCITY and abs(vel_north) > 0.05:
            vel_north = self.MIN_VELOCITY * (1 if vel_north > 0 else -1)
            
        return (vel_north, vel_east)
        
    def send_velocity_command(self, vel_north, vel_east, vel_down=0):
        """
        Send velocity command to drone in NED frame.
        
        Args:
            vel_north: Velocity north (m/s)
            vel_east: Velocity east (m/s)
            vel_down: Velocity down (m/s), positive = descend
        """
        # Ensure we're in GUIDED mode
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.5)
            
        # Create SET_POSITION_TARGET_LOCAL_NED message
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only velocity enabled)
            0, 0, 0,  # position (ignored)
            vel_north, vel_east, vel_down,  # velocity
            0, 0, 0,  # acceleration (ignored)
            0, 0      # yaw, yaw_rate (ignored)
        )
        
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        
    def is_centered(self, offset_x, offset_y, threshold):
        """Check if human is within centering threshold."""
        return abs(offset_x) < threshold and abs(offset_y) < threshold
        
    def center_over_human(self, threshold, timeout=None):
        """
        Adjust drone position until human is centered in frame.
        
        Args:
            threshold: Pixel threshold for "centered" (smaller = more precise)
            timeout: Max time to attempt centering (seconds)
            
        Returns:
            bool: True if successfully centered, False otherwise
        """
        if timeout is None:
            timeout = self.CENTERING_TIMEOUT
            
        start_time = time.time()
        last_detection_time = time.time()
        consecutive_centered = 0
        required_consecutive = 5  # Must be centered for 5 consecutive frames
        
        self.log("info", f"Centering over human (threshold: {threshold}px)...")
        
        while time.time() - start_time < timeout:
            if self.abort_flag:
                self.log("warning", "Centering aborted")
                return False
                
            # Get frame and detect human
            frame = self.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue
                
            detection = self.detect_human(frame)
            
            if detection is None:
                # Lost detection - stop and wait
                self.send_velocity_command(0, 0, 0)
                
                if time.time() - last_detection_time > self.DETECTION_TIMEOUT:
                    self.log("error", "Lost human detection for too long")
                    return False
                    
                time.sleep(0.1)
                continue
                
            last_detection_time = time.time()
            human_x, human_y, conf, bbox = detection
            
            # Calculate offset
            offset_x, offset_y = self.calculate_offset(human_x, human_y)
            
            # Get current altitude
            altitude = self.vehicle.location.global_relative_frame.alt or self.APPROACH_ALTITUDE
            
            # Check if centered
            if self.is_centered(offset_x, offset_y, threshold):
                consecutive_centered += 1
                self.send_velocity_command(0, 0, 0)  # Hold position
                
                if consecutive_centered >= required_consecutive:
                    self.log("success", f"Centered! Offset: ({offset_x}, {offset_y}) px")
                    return True
            else:
                consecutive_centered = 0
                
                # Calculate and send velocity command
                vel_north, vel_east = self.calculate_velocity_command(offset_x, offset_y, altitude)
                self.send_velocity_command(vel_north, vel_east, 0)
                
                self.log("info", f"Offset: ({offset_x:+4d}, {offset_y:+4d}) px | "
                               f"Vel: N={vel_north:+.2f} E={vel_east:+.2f} m/s")
                               
            time.sleep(0.1)  # 10 Hz control loop
            
        self.log("error", "Centering timeout")
        return False
        
    def descend_with_centering(self, target_altitude):
        """
        Descend to target altitude while maintaining centering.
        
        Descends in steps, re-centering at each altitude.
        
        Args:
            target_altitude: Target altitude in meters
            
        Returns:
            bool: True if successfully descended and centered
        """
        current_alt = self.vehicle.location.global_relative_frame.alt or self.APPROACH_ALTITUDE
        
        self.log("info", f"Descending from {current_alt:.1f}m to {target_altitude:.1f}m with centering")
        
        while current_alt > target_altitude + 0.5:
            if self.abort_flag:
                return False
                
            # Calculate next altitude (descend by DESCENT_STEP)
            next_alt = max(target_altitude, current_alt - self.DESCENT_STEP)
            
            # Descend
            self.log("info", f"Descending to {next_alt:.1f}m...")
            
            # Send descent command
            descent_start = time.time()
            while self.vehicle.location.global_relative_frame.alt > next_alt + 0.3:
                if self.abort_flag:
                    return False
                    
                # Get frame and check if we're still centered
                frame = self.get_frame()
                if frame:
                    detection = self.detect_human(frame)
                    if detection:
                        human_x, human_y, conf, bbox = detection
                        offset_x, offset_y = self.calculate_offset(human_x, human_y)
                        
                        # Adjust horizontally while descending
                        vel_north, vel_east = self.calculate_velocity_command(
                            offset_x, offset_y, 
                            self.vehicle.location.global_relative_frame.alt
                        )
                        
                        # Slow descent + horizontal correction
                        self.send_velocity_command(vel_north, vel_east, 0.5)
                    else:
                        # Lost detection - pause descent
                        self.send_velocity_command(0, 0, 0)
                        
                if time.time() - descent_start > 30:
                    self.log("error", "Descent timeout")
                    break
                    
                time.sleep(0.1)
                
            # Re-center at new altitude with finer threshold
            current_alt = self.vehicle.location.global_relative_frame.alt or next_alt
            
            # Threshold gets smaller as we descend
            alt_ratio = (current_alt - target_altitude) / (self.APPROACH_ALTITUDE - target_altitude)
            alt_ratio = max(0, min(1, alt_ratio))
            threshold = int(self.FINE_THRESHOLD + (self.COARSE_THRESHOLD - self.FINE_THRESHOLD) * alt_ratio)
            
            if not self.center_over_human(threshold, timeout=30):
                self.log("warning", f"Failed to center at {current_alt:.1f}m, continuing...")
                
            current_alt = self.vehicle.location.global_relative_frame.alt or next_alt
            
        self.log("success", f"Reached delivery altitude: {current_alt:.1f}m")
        return True
        
    def fly_to_waypoint(self, lat, lon, altitude):
        """
        Fly to waypoint at specified altitude.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            altitude: Target altitude (meters AGL)
            
        Returns:
            bool: True if reached waypoint
        """
        self.log("info", f"Flying to waypoint: {lat:.6f}, {lon:.6f} at {altitude}m")
        
        # Ensure GUIDED mode
        if self.vehicle.mode.name != "GUIDED":
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            
        # Create target location
        target = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.simple_goto(target, groundspeed=5)
        
        # Wait to reach waypoint
        start_time = time.time()
        timeout = 120  # 2 minutes max
        
        while time.time() - start_time < timeout:
            if self.abort_flag:
                return False
                
            current = self.vehicle.location.global_relative_frame
            
            # Calculate distance to target
            dlat = target.lat - current.lat
            dlon = target.lon - current.lon
            distance = math.sqrt(dlat**2 + dlon**2) * 111319.9  # Approx meters
            
            if distance < 2.0:  # Within 2 meters
                self.log("success", f"Reached waypoint (distance: {distance:.1f}m)")
                return True
                
            self.log("info", f"Distance to waypoint: {distance:.1f}m")
            time.sleep(1)
            
        self.log("error", "Waypoint timeout")
        return False
        
    def hover(self, duration, release_package=True):
        """
        Hover in place for specified duration and optionally release package.
        
        Args:
            duration: Time to hover in seconds
            release_package: If True, trigger package release during hover
        """
        self.log("info", f"Hovering for {duration} seconds...")
        
        # Hold position while releasing package
        start_time = time.time()
        package_released = False
        
        while time.time() - start_time < duration:
            if self.abort_flag:
                return False
            
            # Send zero velocity to maintain position
            self.send_velocity_command(0, 0, 0)
            
            # Trigger package release at midpoint of hover (after stabilizing)
            elapsed = time.time() - start_time
            if release_package and not package_released and elapsed >= 1.0:
                self.log("success", "=== RELEASING PACKAGE ===")
                self.release_package()
                package_released = True
            
            time.sleep(0.5)
        
        return True
    
    def release_package(self):
        """
        Trigger the package release mechanism.
        Activates the NEMA 17 stepper motor to release the delivery payload.
        
        Returns:
            bool: True if package released successfully
        """
        self.log("info", "Triggering package release mechanism...")
        
        # Get current position for logging
        pos = self.vehicle.location.global_relative_frame
        lat = pos.lat if pos.lat else 0
        lon = pos.lon if pos.lon else 0
        alt = pos.alt if pos.alt else 0
        
        if self.motor and self.motor.initialized:
            # Send GCS notification before release
            if self.send_gcs:
                self.send_gcs(f"MOTOR: Activating package release (180° rotation)")
            
            success = self.motor.release_package()
            
            if success:
                self.log("success", "Package delivered successfully!")
                # Send success log to GCS
                if self.send_gcs:
                    self.send_gcs(f"PACKAGE RELEASED at {lat:.6f},{lon:.6f},{alt:.1f}m")
            else:
                self.log("error", "Package release mechanism failed")
                if self.send_gcs:
                    self.send_gcs(f"ERROR: Package release mechanism failed at {lat:.6f},{lon:.6f},{alt:.1f}m")
            return success
        else:
            self.log("warning", "Motor not available - simulating package release")
            if self.send_gcs:
                self.send_gcs(f"WARNING: Motor not available - simulating release at {lat:.6f},{lon:.6f},{alt:.1f}m")
            time.sleep(1.0)  # Simulate release time
            return False
        
    def execute_delivery(self, target_lat, target_lon):
        """
        Execute complete delivery sequence to a single target.
        
        1. Fly to approximate location at 15m
        2. Detect human and center
        3. Descend to 6m while re-centering
        4. Hover for 5 seconds
        5. Log completion
        
        Args:
            target_lat: Approximate target latitude (from scout drone)
            target_lon: Approximate target longitude (from scout drone)
            
        Returns:
            dict: Delivery result log
        """
        delivery_start = datetime.now()
        result = {
            'target_lat': target_lat,
            'target_lon': target_lon,
            'start_time': delivery_start.isoformat(),
            'success': False,
            'final_lat': None,
            'final_lon': None,
            'final_alt': None,
            'error': None
        }
        
        try:
            # Step 1: Fly to approximate location at approach altitude
            self.log("info", "=" * 50)
            self.log("info", f"DELIVERY TARGET: {target_lat:.6f}, {target_lon:.6f}")
            self.log("info", "=" * 50)
            
            if not self.fly_to_waypoint(target_lat, target_lon, self.APPROACH_ALTITUDE):
                result['error'] = "Failed to reach waypoint"
                return result
                
            # Step 2: Initial coarse centering at 15m
            self.log("info", "Starting coarse centering at 15m...")
            if not self.center_over_human(self.COARSE_THRESHOLD):
                result['error'] = "Failed to center at approach altitude"
                return result
                
            # Step 3: Descend to delivery altitude while re-centering
            if not self.descend_with_centering(self.DELIVERY_ALTITUDE):
                result['error'] = "Failed during descent"
                return result
                
            # Step 4: Final fine centering at 6m
            self.log("info", "Final fine centering at delivery altitude...")
            if not self.center_over_human(self.FINE_THRESHOLD):
                result['error'] = "Failed to achieve fine centering"
                return result
                
            # Step 5: Hover for delivery
            self.log("success", "DELIVERING - Hovering over target...")
            self.hover(self.HOVER_TIME)
            
            # Record final position
            final_pos = self.vehicle.location.global_relative_frame
            result['final_lat'] = final_pos.lat
            result['final_lon'] = final_pos.lon
            result['final_alt'] = final_pos.alt
            result['success'] = True
            
            self.log("success", f"DELIVERY COMPLETE at {final_pos.lat:.6f}, {final_pos.lon:.6f}")
            
        except Exception as e:
            result['error'] = str(e)
            self.log("error", f"Delivery error: {e}")
            
        result['end_time'] = datetime.now().isoformat()
        result['duration'] = (datetime.now() - delivery_start).total_seconds()
        
        self.delivery_log.append(result)
        return result
        
    def execute_delivery_mission(self, waypoints):
        """
        Execute delivery mission to multiple waypoints.
        
        Args:
            waypoints: List of (lat, lon) tuples from GCS
            
        Returns:
            list: Results for each delivery
        """
        self.mission_running = True
        self.abort_flag = False
        results = []
        
        # Start camera
        if not self.start_camera():
            self.log("error", "Cannot start camera - aborting mission")
            return results
            
        try:
            for i, (lat, lon) in enumerate(waypoints):
                if self.abort_flag:
                    self.log("warning", "Mission aborted")
                    break
                    
                self.target_index = i + 1
                self.log("info", f"Processing delivery {i+1}/{len(waypoints)}")
                
                result = self.execute_delivery(lat, lon)
                results.append(result)
                
                # After delivery, climb back to approach altitude
                if result['success'] and i < len(waypoints) - 1:
                    self.log("info", "Climbing back to approach altitude...")
                    self.fly_to_waypoint(lat, lon, self.APPROACH_ALTITUDE)
                    
        finally:
            self.stop_camera()
            self.mission_running = False
            
        return results
        
    def abort(self):
        """Abort current delivery operation."""
        self.abort_flag = True
        self.send_velocity_command(0, 0, 0)
        self.log("warning", "Delivery aborted!")


class DeliveryCommandHandler:
    """
    Handles delivery commands from GCS.
    
    Commands:
        DELIVER:lat,lon           - Deliver to single location
        DELIVER:lat1,lon1;lat2,lon2  - Deliver to multiple locations
        DELIVER:ABORT             - Abort current delivery
        DELIVER:STATUS            - Get delivery status
    """
    
    def __init__(self, vehicle, send_response_func, log_func=None):
        self.vehicle = vehicle
        self.send_response = send_response_func
        self.log = log_func if log_func else self._default_log
        self.servo_controller = None  # VisualServoController instance
        self.delivery_thread = None
    
    def _default_log(self, level, message):
        """Default logger if none provided."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level.upper()}] {message}")
        
    def handle_command(self, cmd):
        """Process delivery command."""
        cmd_upper = cmd.strip().upper()
        
        if cmd_upper == "DELIVER:ABORT":
            return self.abort_delivery()
            
        if cmd_upper == "DELIVER:STATUS":
            return self.get_status()
            
        if cmd_upper.startswith("DELIVER:"):
            waypoints_str = cmd.strip()[8:]  # Keep original case for parsing
            return self.start_delivery(waypoints_str)
            
        return False
        
    def start_delivery(self, waypoints_str):
        """
        Start delivery mission.
        
        Args:
            waypoints_str: "lat1,lon1;lat2,lon2;..." or "lat,lon"
        """
        try:
            # Parse waypoints
            waypoints = []
            for wp in waypoints_str.split(';'):
                parts = wp.strip().split(',')
                if len(parts) >= 2:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    waypoints.append((lat, lon))
                    
            if not waypoints:
                self.send_response("ERROR: No valid waypoints")
                return False
                
            self.send_response(f"DELIVERY: Starting mission with {len(waypoints)} waypoint(s)")
            
            # Create servo controller with GCS callback
            self.servo_controller = VisualServoController(
                self.vehicle, 
                logger=self.log,
                send_response=self.send_response
            )
            
            # Run in background thread
            self.delivery_thread = threading.Thread(
                target=self._run_delivery,
                args=(waypoints,),
                daemon=True
            )
            self.delivery_thread.start()
            
            return True
            
        except Exception as e:
            self.send_response(f"ERROR: {e}")
            return False
            
    def _run_delivery(self, waypoints):
        """Background delivery execution."""
        try:
            results = self.servo.execute_delivery_mission(waypoints)
            
            # Send summary to GCS
            success_count = sum(1 for r in results if r['success'])
            self.send_response(f"DELIVERY COMPLETE: {success_count}/{len(results)} successful")
            
            # Send detailed log
            for i, r in enumerate(results):
                if r['success']:
                    self.send_response(
                        f"DELIVERY {i+1}: SUCCESS at {r['final_lat']:.6f},{r['final_lon']:.6f}"
                    )
                else:
                    self.send_response(f"DELIVERY {i+1}: FAILED - {r['error']}")
                    
        except Exception as e:
            self.send_response(f"DELIVERY ERROR: {e}")
            
    def abort_delivery(self):
        """Abort current delivery."""
        if self.servo:
            self.servo.abort()
            self.send_response("DELIVERY: Aborting...")
            return True
        self.send_response("DELIVERY: No active delivery")
        return False
        
    def get_status(self):
        """Get delivery status."""
        if self.servo and self.servo.mission_running:
            self.send_response(
                f"DELIVERY: Active - Target {self.servo.target_index}, "
                f"Completed: {len(self.servo.delivery_log)}"
            )
        else:
            self.send_response("DELIVERY: Idle")
        return True


# =============================================================================
# Test / Demo
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Visual Servo Delivery System - Test Mode")
    print("=" * 60)
    
    # Test scale factor calculation
    class MockVehicle:
        class Location:
            class GRF:
                lat = 12.9716
                lon = 77.5946
                alt = 15.0
            global_relative_frame = GRF()
        location = Location()
        mode = type('obj', (object,), {'name': 'GUIDED'})()
        def send_mavlink(self, msg): pass
        def flush(self): pass
        message_factory = None
        
    print("\nScale Factor Test:")
    print("-" * 40)
    
    servo = VisualServoController(MockVehicle())
    
    for alt in [6, 10, 15, 20]:
        sf = servo.scale_factors[alt]
        print(f"  {alt:2d}m: X={sf['x']:.4f} m/px, Y={sf['y']:.4f} m/px")
        print(f"       Ground coverage: {sf['x']*640:.1f}m x {sf['y']*480:.1f}m")
        
    print("\nOffset Correction Test:")
    print("-" * 40)
    
    # Simulate pixel offsets
    test_cases = [
        (100, 50, 15),   # 100px right, 50px down at 15m
        (50, -30, 10),   # 50px right, 30px up at 10m
        (-80, 60, 6),    # 80px left, 60px down at 6m
    ]
    
    for ox, oy, alt in test_cases:
        scale = servo.get_meters_per_pixel(alt)
        vel_n, vel_e = servo.calculate_velocity_command(ox, oy, alt)
        error_m = (ox * scale['x'], -oy * scale['y'])
        print(f"  Offset ({ox:+4d}, {oy:+4d}) px at {alt}m:")
        print(f"    Error: ({error_m[0]:+.2f}, {error_m[1]:+.2f}) meters")
        print(f"    Velocity: N={vel_n:+.2f}, E={vel_e:+.2f} m/s")
        
    print("\n" + "=" * 60)
    print("Commands:")
    print("  DELIVER:12.9716,77.5946        - Single delivery")
    print("  DELIVER:12.97,77.59;12.98,77.60 - Multiple deliveries")
    print("  DELIVER:ABORT                  - Abort delivery")
    print("  DELIVER:STATUS                 - Get status")
    print("=" * 60)
