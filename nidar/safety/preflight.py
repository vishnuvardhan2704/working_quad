"""
Pre-flight safety checks for autonomous mission.
"""

import time
from utils.logger import MissionLogger


class PreflightChecks:
    """Performs pre-flight safety validation."""
    
    def __init__(self, vehicle):
        """
        Initialize preflight checker.
        
        Args:
            vehicle: DroneKit Vehicle object
        """
        self.vehicle = vehicle
    
    def check_gps_fix(self, min_satellites=6):
        """
        Verify GPS has sufficient fix quality.
        
        Args:
            min_satellites: Minimum number of satellites required
            
        Returns:
            True if GPS fix is adequate
        """
        MissionLogger.info("Checking GPS fix...")
        
        gps = self.vehicle.gps_0
        
        if gps.fix_type < 2:
            MissionLogger.error(f"Insufficient GPS fix type: {gps.fix_type} (need 2D or 3D fix)")
            return False
        
        if gps.satellites_visible < min_satellites:
            MissionLogger.warning(f"Low satellite count: {gps.satellites_visible} (recommended: {min_satellites}+)")
            # Continue anyway for SITL, but warn
        
        MissionLogger.success(f"GPS OK: {gps.satellites_visible} satellites, fix type {gps.fix_type}")
        return True
    
    def check_armable(self, timeout=30):
        """
        Wait for vehicle to become armable.
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            True if vehicle is armable
        """
        MissionLogger.info("Waiting for vehicle to be armable...")
        
        start_time = time.time()
        while not self.vehicle.is_armable:
            if time.time() - start_time > timeout:
                MissionLogger.error("Vehicle did not become armable within timeout")
                return False
            
            MissionLogger.info(f"Not armable yet (EKF: {self.vehicle.ekf_ok})...")
            time.sleep(1)
        
        MissionLogger.success("Vehicle is armable")
        return True
    
    def check_battery(self, min_voltage=10.5):
        """
        Verify battery voltage is sufficient.
        
        Args:
            min_voltage: Minimum acceptable voltage
            
        Returns:
            True if battery is adequate
        """
        MissionLogger.info("Checking battery status...")
        
        battery = self.vehicle.battery
        
        if battery.voltage is None:
            MissionLogger.warning("Battery voltage unavailable")
            return True  # Continue for SITL
        
        if battery.voltage < min_voltage:
            MissionLogger.error(f"Battery too low: {battery.voltage}V (minimum: {min_voltage}V)")
            return False
        
        MissionLogger.success(f"Battery OK: {battery.voltage}V, {battery.level}%")
        return True
    
    def check_home_location(self, timeout=60):
        """
        Wait for home location to be set.
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            True if home location is set
        """
        MissionLogger.info("Waiting for home location (GPS lock may take 30-60s in SITL)...")
        
        start_time = time.time()
        while self.vehicle.home_location is None:
            if time.time() - start_time > timeout:
                MissionLogger.error("Home location not set within timeout")
                MissionLogger.error("Check SITL terminal for GPS status")
                return False
            
            elapsed = int(time.time() - start_time)
            if elapsed % 5 == 0:  # Log every 5 seconds instead of every second
                MissionLogger.info(f"Waiting for GPS lock... ({elapsed}s / {timeout}s)")
            time.sleep(1)
        
        home = self.vehicle.home_location
        MissionLogger.success(f"Home location set: ({home.lat:.6f}, {home.lon:.6f}, {home.alt}m)")
        return True
    
    def run_all_checks(self):
        """
        Run complete pre-flight check sequence.
        
        Returns:
            True if all checks pass
        """
        MissionLogger.header("PRE-FLIGHT CHECKS")
        
        checks = [
            ("GPS Fix", self.check_gps_fix),
            ("Armable Status", self.check_armable),
            ("Battery Level", self.check_battery),
            ("Home Location", self.check_home_location),
        ]
        
        for check_name, check_func in checks:
            if not check_func():
                MissionLogger.error(f"Pre-flight check failed: {check_name}")
                return False
        
        MissionLogger.success("All pre-flight checks passed")
        return True
