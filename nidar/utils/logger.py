"""
Console logging utility for mission events and state transitions.
Supports optional telemetry forwarding to ground station via MAVLink.
"""

import time
from datetime import datetime


class MissionLogger:
    """
    Provides structured console logging for the autonomous mission.
    Optionally forwards logs to ground station via TelemetryLogger.
    """
    
    # ANSI color codes
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    
    # Class-level telemetry logger reference
    _telemetry_logger = None
    _telemetry_enabled = False
    
    @classmethod
    def set_telemetry_logger(cls, telemetry_logger):
        """
        Set the telemetry logger for forwarding messages to ground station.
        
        Args:
            telemetry_logger: TelemetryLogger instance or None to disable
        """
        cls._telemetry_logger = telemetry_logger
        cls._telemetry_enabled = telemetry_logger is not None
    
    @classmethod
    def enable_telemetry(cls, enabled=True):
        """Enable or disable telemetry forwarding."""
        cls._telemetry_enabled = enabled and cls._telemetry_logger is not None
    
    @staticmethod
    def _timestamp():
        """Returns formatted timestamp."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    @classmethod
    def _send_to_telemetry(cls, level, message):
        """Forward message to telemetry logger if enabled."""
        if cls._telemetry_enabled and cls._telemetry_logger:
            try:
                # Map log levels to telemetry methods
                if level == 'ERROR':
                    cls._telemetry_logger.error(message)
                elif level == 'WARNING':
                    cls._telemetry_logger.warning(message)
                elif level == 'SUCCESS':
                    cls._telemetry_logger.notice(message)
                elif level == 'STATE':
                    cls._telemetry_logger.notice(message)
                else:  # INFO
                    cls._telemetry_logger.info(message)
            except Exception:
                pass  # Don't let telemetry errors break logging
    
    @classmethod
    def info(cls, message):
        """Log informational message."""
        print(f"{cls.CYAN}[{cls._timestamp()}] INFO:{cls.RESET} {message}")
        cls._send_to_telemetry('INFO', message)
    
    @classmethod
    def success(cls, message):
        """Log success message."""
        print(f"{cls.GREEN}[{cls._timestamp()}] SUCCESS:{cls.RESET} {message}")
        cls._send_to_telemetry('SUCCESS', message)
    
    @classmethod
    def warning(cls, message):
        """Log warning message."""
        print(f"{cls.YELLOW}[{cls._timestamp()}] WARNING:{cls.RESET} {message}")
        cls._send_to_telemetry('WARNING', message)
    
    @classmethod
    def error(cls, message):
        """Log error message."""
        print(f"{cls.RED}[{cls._timestamp()}] ERROR:{cls.RESET} {message}")
        cls._send_to_telemetry('ERROR', message)
    
    @classmethod
    def state(cls, message):
        """Log state transition."""
        print(f"{cls.BOLD}{cls.BLUE}[{cls._timestamp()}] STATE:{cls.RESET} {message}")
        cls._send_to_telemetry('STATE', message)
    
    @classmethod
    def header(cls, message):
        """Log section header."""
        print(f"\n{cls.BOLD}{'=' * 60}{cls.RESET}")
        print(f"{cls.BOLD}{message}{cls.RESET}")
        print(f"{cls.BOLD}{'=' * 60}{cls.RESET}\n")
        cls._send_to_telemetry('INFO', f"=== {message} ===")
