"""
Console logging utility for mission events and state transitions.
"""

import time
from datetime import datetime


class MissionLogger:
    """Provides structured console logging for the autonomous mission."""
    
    # ANSI color codes
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    
    @staticmethod
    def _timestamp():
        """Returns formatted timestamp."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    @staticmethod
    def info(message):
        """Log informational message."""
        print(f"{MissionLogger.CYAN}[{MissionLogger._timestamp()}] INFO:{MissionLogger.RESET} {message}")
    
    @staticmethod
    def success(message):
        """Log success message."""
        print(f"{MissionLogger.GREEN}[{MissionLogger._timestamp()}] SUCCESS:{MissionLogger.RESET} {message}")
    
    @staticmethod
    def warning(message):
        """Log warning message."""
        print(f"{MissionLogger.YELLOW}[{MissionLogger._timestamp()}] WARNING:{MissionLogger.RESET} {message}")
    
    @staticmethod
    def error(message):
        """Log error message."""
        print(f"{MissionLogger.RED}[{MissionLogger._timestamp()}] ERROR:{MissionLogger.RESET} {message}")
    
    @staticmethod
    def state(message):
        """Log state transition."""
        print(f"{MissionLogger.BOLD}{MissionLogger.BLUE}[{MissionLogger._timestamp()}] STATE:{MissionLogger.RESET} {message}")
    
    @staticmethod
    def header(message):
        """Log section header."""
        print(f"\n{MissionLogger.BOLD}{'=' * 60}{MissionLogger.RESET}")
        print(f"{MissionLogger.BOLD}{message}{MissionLogger.RESET}")
        print(f"{MissionLogger.BOLD}{'=' * 60}{MissionLogger.RESET}\n")
