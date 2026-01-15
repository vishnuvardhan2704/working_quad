#!/usr/bin/env python3
"""
Stepper Motor Test Script for Package Delivery Mechanism

Tests the NEMA 17 stepper motor connected to RPi via A4988/DRV8825 driver.

Wiring (RPi GPIO BCM to Driver):
    GPIO 18 -> STEP
    GPIO 23 -> DIR (Direction)
    GPIO 24 -> ENABLE (Active LOW)
    RESET & SLEEP -> Shorted together on driver
    MS1, MS2, MS3 -> GND (Full step mode)
    3.3V -> VDD
    GND -> GND

Usage:
    python3 test_stepper_motor.py              # Interactive menu
    python3 test_stepper_motor.py --release    # Test package release (180°)
    python3 test_stepper_motor.py --retract    # Test retract (180° back)
    python3 test_stepper_motor.py --full       # Full rotation test (360°)
    python3 test_stepper_motor.py --step 50    # Move specific steps
    python3 test_stepper_motor.py --degrees 90 # Rotate specific degrees
"""

import sys
import time
import argparse

# Try to import GPIO
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[ERROR] RPi.GPIO not available!")
    print("Install with: sudo apt-get install python3-rpi.gpio")
    print("Or run on Raspberry Pi")


class StepperMotorTest:
    """Test class for NEMA 17 stepper motor."""
    
    # GPIO Pin assignments (BCM mode)
    STEP_PIN = 18
    DIR_PIN = 23
    ENABLE_PIN = 24
    
    # Motor parameters
    # NEMA 17 = 200 steps/revolution (1.8° per step) in full-step mode
    # With 1/2 microstepping (common driver default): 400 steps/rev
    # Adjust MICROSTEP_MODE based on your driver's MS1/MS2/MS3 configuration
    MICROSTEP_MODE = 1    # 1=full, 2=half, 4=quarter, 8=eighth, 16=sixteenth
    STEPS_PER_REV = 200 * MICROSTEP_MODE  # 800 steps for 1/4 microstepping
    STEP_DELAY = 0.003  # 1ms delay between steps (faster for microstepping)
    
    def __init__(self):
        """Initialize the stepper motor test."""
        self.initialized = False
        
        if not GPIO_AVAILABLE:
            print("[ERROR] Cannot initialize - GPIO not available")
            return
        
        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configure pins as outputs
            GPIO.setup(self.STEP_PIN, GPIO.OUT)
            GPIO.setup(self.DIR_PIN, GPIO.OUT)
            GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
            
            # Initialize states (motor disabled)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            GPIO.output(self.DIR_PIN, GPIO.LOW)
            GPIO.output(self.ENABLE_PIN, GPIO.HIGH)  # Disabled (active LOW)
            
            self.initialized = True
            print("[OK] Stepper motor GPIO initialized")
            print(f"     STEP: GPIO {self.STEP_PIN}")
            print(f"     DIR:  GPIO {self.DIR_PIN}")
            print(f"     EN:   GPIO {self.ENABLE_PIN}")
            
        except Exception as e:
            print(f"[ERROR] GPIO initialization failed: {e}")
            self.initialized = False
    
    def enable_motor(self):
        """Enable the motor driver."""
        if not self.initialized:
            return False
        GPIO.output(self.ENABLE_PIN, GPIO.LOW)  # Active LOW
        time.sleep(0.01)
        print("[MOTOR] Enabled")
        return True
    
    def disable_motor(self):
        """Disable the motor driver."""
        if not self.initialized:
            return
        GPIO.output(self.ENABLE_PIN, GPIO.HIGH)
        print("[MOTOR] Disabled")
    
    def set_direction(self, clockwise=True):
        """Set motor rotation direction."""
        if not self.initialized:
            return
        GPIO.output(self.DIR_PIN, GPIO.HIGH if clockwise else GPIO.LOW)
        direction = "Clockwise" if clockwise else "Counter-Clockwise"
        print(f"[MOTOR] Direction: {direction}")
    
    def step(self, steps, delay=None):
        """Execute specified number of steps."""
        if not self.initialized:
            print("[ERROR] Motor not initialized")
            return False
        
        if delay is None:
            delay = self.STEP_DELAY
        
        print(f"[MOTOR] Executing {steps} steps (delay: {delay*1000:.1f}ms)...")
        
        for i in range(steps):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(delay / 2)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(delay / 2)
            
            # Progress indicator every 50 steps
            if (i + 1) % 50 == 0:
                print(f"     Step {i + 1}/{steps}")
        
        print(f"[MOTOR] Completed {steps} steps")
        return True
    
    def rotate_degrees(self, degrees, clockwise=True):
        """Rotate motor by specified degrees."""
        if not self.initialized:
            return False
        
        total_steps = int((degrees / 360.0) * self.STEPS_PER_REV)
        direction = "CW" if clockwise else "CCW"
        
        print(f"\n{'='*50}")
        print(f"[TEST] Rotating {degrees}° {direction}")
        print(f"       = {total_steps} steps")
        print(f"{'='*50}")
        
        self.enable_motor()
        self.set_direction(clockwise)
        success = self.step(total_steps)
        self.disable_motor()
        
        return success
    
    def test_release(self):
        """Test the package release motion (180° clockwise)."""
        print("\n" + "="*50)
        print("[TEST] PACKAGE RELEASE - 180° Clockwise")
        print("="*50)
        return self.rotate_degrees(180, clockwise=True)
    
    def test_retract(self):
        """Test the retract motion (180° counter-clockwise)."""
        print("\n" + "="*50)
        print("[TEST] RETRACT - 180° Counter-Clockwise")
        print("="*50)
        return self.rotate_degrees(180, clockwise=False)
    
    def test_full_rotation(self):
        """Test full rotation (360°) in both directions."""
        print("\n" + "="*50)
        print("[TEST] FULL ROTATION TEST")
        print("="*50)
        
        print("\n[1/2] Full rotation clockwise...")
        self.rotate_degrees(360, clockwise=True)
        
        time.sleep(1)
        
        print("\n[2/2] Full rotation counter-clockwise...")
        self.rotate_degrees(360, clockwise=False)
        
        print("\n[TEST] Full rotation test complete!")
        return True
    
    def test_delivery_cycle(self):
        """Test complete delivery cycle: release + retract."""
        print("\n" + "="*50)
        print("[TEST] COMPLETE DELIVERY CYCLE")
        print("="*50)
        
        print("\n[1/2] Release (180° CW)...")
        self.test_release()
        
        print("\nWaiting 2 seconds (simulating package drop)...")
        time.sleep(2)
        
        print("\n[2/2] Retract (180° CCW)...")
        self.test_retract()
        
        print("\n[TEST] Delivery cycle complete!")
        return True
    
    def interactive_menu(self):
        """Interactive test menu."""
        print("\n" + "="*50)
        print("   STEPPER MOTOR TEST - Interactive Mode")
        print("="*50)
        
        while True:
            print("\nOptions:")
            print("  1. Test Release (180° CW)")
            print("  2. Test Retract (180° CCW)")
            print("  3. Full Delivery Cycle (Release + Retract)")
            print("  4. Full Rotation Test (360° both ways)")
            print("  5. Custom Degrees")
            print("  6. Custom Steps")
            print("  7. Toggle Direction & Single Step")
            print("  q. Quit")
            
            choice = input("\nEnter choice: ").strip().lower()
            
            if choice == '1':
                self.test_release()
            elif choice == '2':
                self.test_retract()
            elif choice == '3':
                self.test_delivery_cycle()
            elif choice == '4':
                self.test_full_rotation()
            elif choice == '5':
                try:
                    degrees = float(input("Enter degrees (e.g., 90): "))
                    direction = input("Direction (cw/ccw) [cw]: ").strip().lower()
                    clockwise = direction != 'ccw'
                    self.rotate_degrees(degrees, clockwise)
                except ValueError:
                    print("[ERROR] Invalid input")
            elif choice == '6':
                try:
                    steps = int(input("Enter number of steps: "))
                    direction = input("Direction (cw/ccw) [cw]: ").strip().lower()
                    clockwise = direction != 'ccw'
                    
                    self.enable_motor()
                    self.set_direction(clockwise)
                    self.step(steps)
                    self.disable_motor()
                except ValueError:
                    print("[ERROR] Invalid input")
            elif choice == '7':
                # Single step toggle for fine adjustment
                direction = input("Direction (cw/ccw) [cw]: ").strip().lower()
                clockwise = direction != 'ccw'
                self.enable_motor()
                self.set_direction(clockwise)
                
                print("Press Enter for each step, 'q' to quit...")
                while True:
                    key = input()
                    if key.lower() == 'q':
                        break
                    self.step(1)
                
                self.disable_motor()
            elif choice == 'q':
                print("\nExiting...")
                break
            else:
                print("[ERROR] Invalid choice")
    
    def cleanup(self):
        """Clean up GPIO resources."""
        if GPIO_AVAILABLE:
            try:
                self.disable_motor()
                GPIO.cleanup([self.STEP_PIN, self.DIR_PIN, self.ENABLE_PIN])
                print("[OK] GPIO cleaned up")
            except Exception as e:
                print(f"[WARN] Cleanup error: {e}")


def main():
    parser = argparse.ArgumentParser(description='Test NEMA 17 stepper motor for delivery mechanism')
    parser.add_argument('--release', action='store_true', help='Test package release (180° CW)')
    parser.add_argument('--retract', action='store_true', help='Test retract (180° CCW)')
    parser.add_argument('--full', action='store_true', help='Full rotation test (360° both ways)')
    parser.add_argument('--cycle', action='store_true', help='Complete delivery cycle test')
    parser.add_argument('--step', type=int, help='Move specific number of steps')
    parser.add_argument('--degrees', type=float, help='Rotate specific degrees')
    parser.add_argument('--ccw', action='store_true', help='Counter-clockwise direction (default: CW)')
    
    args = parser.parse_args()
    
    if not GPIO_AVAILABLE:
        print("\n[ERROR] Cannot run test - GPIO not available")
        print("This script must be run on Raspberry Pi with RPi.GPIO installed")
        sys.exit(1)
    
    # Initialize motor
    motor = StepperMotorTest()
    
    if not motor.initialized:
        print("[ERROR] Motor initialization failed")
        sys.exit(1)
    
    try:
        # Handle command line arguments
        if args.release:
            motor.test_release()
        elif args.retract:
            motor.test_retract()
        elif args.full:
            motor.test_full_rotation()
        elif args.cycle:
            motor.test_delivery_cycle()
        elif args.step:
            clockwise = not args.ccw
            motor.enable_motor()
            motor.set_direction(clockwise)
            motor.step(args.step)
            motor.disable_motor()
        elif args.degrees:
            clockwise = not args.ccw
            motor.rotate_degrees(args.degrees, clockwise)
        else:
            # Interactive mode
            motor.interactive_menu()
            
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Stopping motor...")
    finally:
        motor.cleanup()
    
    print("\nTest complete!")


if __name__ == "__main__":
    main()
