"""
File-based session logger for LoRa service.
Automatically saves logs from each session to timestamped files.

Logs are saved to: /home/dart/quadtest/logs/sessions/
Each session creates a new log file with timestamp.

Log Format:
===========
    [HH:MM:SS.mmm] [CATEGORY] Message
    
Categories:
    TX >>    - Commands/data sent TO ground station
    RX <<    - Commands received FROM ground station
    INFO     - General information
    SUCCESS  - Successful operations
    WARNING  - Warnings
    ERROR    - Errors
    STATE    - State transitions
    DETECT   - Human detection events
    MISSION  - Mission events
    TELEM    - Telemetry data
"""

import os
import sys
from datetime import datetime
from threading import Lock


class SessionFileLogger:
    """
    File-based logger that saves all session output to timestamped log files.
    Formatted for easy reading with clear TX/RX distinction.
    """
    
    # Base log directory
    LOG_BASE_DIR = "/home/dart/quadtest/logs/sessions"
    
    # ANSI color codes for console output
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\033[95m"
    WHITE = "\033[97m"
    
    _instance = None
    _session_file = None
    _file_handle = None
    _write_lock = Lock()
    _session_start = None
    _command_count = 0
    _tx_count = 0
    _rx_count = 0
    _error_count = 0
    _radio_telemetry = None  # Radio telemetry logger for forwarding to TX
    
    def __new__(cls):
        """Singleton pattern - only one logger instance."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self._setup_session_logging()
    
    def _setup_session_logging(self):
        """Set up file logging for this session."""
        # Create log directory structure
        os.makedirs(self.LOG_BASE_DIR, exist_ok=True)
        
        # Create session-specific log file
        self._session_start = datetime.now()
        date_dir = self._session_start.strftime("%Y-%m-%d")
        date_path = os.path.join(self.LOG_BASE_DIR, date_dir)
        os.makedirs(date_path, exist_ok=True)
        
        # Session file name with timestamp
        session_name = self._session_start.strftime("%H-%M-%S")
        self._session_file = os.path.join(date_path, f"session_{session_name}.log")
        
        # Open file handle directly for more control
        self._file_handle = open(self._session_file, 'a', encoding='utf-8', buffering=1)
        
        # Write session header
        self._write_session_header()
    
    def _write_session_header(self):
        """Write formatted session start header to log file."""
        self._write_raw("""
================================================================================
                        LORA SERVICE SESSION LOG                              
================================================================================
  Start Time : {start_time}
  Log File   : {log_file}
  PID        : {pid}
--------------------------------------------------------------------------------
  LEGEND:
    TX >>     = Data/commands sent TO ground station
    RX <<     = Commands received FROM ground station
    INFO      = General information
    SUCCESS   = Successful operations  
    ERROR     = Errors and failures
    STATE     = State changes
    DETECT    = Human detection events
    MISSION   = Mission-related events
================================================================================

""".format(
            start_time=self._session_start.strftime('%Y-%m-%d %H:%M:%S'),
            log_file=os.path.basename(self._session_file),
            pid=str(os.getpid())
        ))
    
    def _write_raw(self, text):
        """Write raw text to log file."""
        if self._file_handle:
            with self._write_lock:
                try:
                    self._file_handle.write(text)
                    self._file_handle.flush()
                except Exception:
                    pass
    
    @classmethod
    def set_radio_telemetry(cls, radio_telem):
        """Set the radio telemetry logger for forwarding logs to TX."""
        instance = cls()
        instance._radio_telemetry = radio_telem
    
    def _format_log_line(self, category, message, symbol=""):
        """Format a log line with timestamp and category."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if symbol:
            return f"[{timestamp}] {symbol} [{category:<8}] {message}\n"
        return f"[{timestamp}]    [{category:<8}] {message}\n"
    
    @property
    def session_file(self):
        """Get the current session log file path."""
        return self._session_file
    
    def _timestamp(self):
        """Returns formatted timestamp for console."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    # =========================================================================
    # TX/RX Command Logging (Most Important)
    # =========================================================================
    
    def tx(self, message):
        """Log data/command sent TO ground station."""
        self._tx_count += 1
        line = self._format_log_line("TX", message, ">>")
        self._write_raw(line)
        print(f"{self.BOLD}{self.GREEN}[{self._timestamp()}] >> TX:{self.RESET} {message}")
    
    def rx(self, command):
        """Log command received FROM ground station."""
        self._rx_count += 1
        self._command_count += 1
        separator = "-" * 60
        self._write_raw(f"\n{separator}\n")
        line = self._format_log_line("RX", command, "<<")
        self._write_raw(line)
        print(f"{self.BOLD}{self.MAGENTA}[{self._timestamp()}] << RX:{self.RESET} {command}")
    
    def response(self, message):
        """Log response sent back for a command."""
        line = self._format_log_line("RESP", message, ">>")
        self._write_raw(line)
        print(f"{self.CYAN}[{self._timestamp()}] >> RESP:{self.RESET} {message}")
    
    # =========================================================================
    # Standard Logging
    # =========================================================================
    
    def info(self, message):
        """Log informational message."""
        line = self._format_log_line("INFO", message)
        self._write_raw(line)
        print(f"{self.CYAN}[{self._timestamp()}] INFO:{self.RESET} {message}")
    
    def success(self, message):
        """Log success message."""
        line = self._format_log_line("SUCCESS", message, "[OK]")
        self._write_raw(line)
        print(f"{self.GREEN}[{self._timestamp()}] [OK] SUCCESS:{self.RESET} {message}")
    
    def warning(self, message):
        """Log warning message."""
        line = self._format_log_line("WARNING", message, "[!]")
        self._write_raw(line)
        print(f"{self.YELLOW}[{self._timestamp()}] [!] WARNING:{self.RESET} {message}")
    
    def error(self, message):
        """Log error message."""
        self._error_count += 1
        line = self._format_log_line("ERROR", message, "[X]")
        self._write_raw(line)
        print(f"{self.RED}[{self._timestamp()}] [X] ERROR:{self.RESET} {message}")
    
    def critical(self, message):
        """Log critical error message."""
        self._error_count += 1
        border = "!" * 60
        self._write_raw(f"\n{border}\n")
        line = self._format_log_line("CRITICAL", message, "[XX]")
        self._write_raw(line)
        self._write_raw(f"{border}\n\n")
        print(f"{self.BOLD}{self.RED}[{self._timestamp()}] [XX] CRITICAL:{self.RESET} {message}")
    
    def debug(self, message):
        """Log debug message."""
        line = self._format_log_line("DEBUG", message)
        self._write_raw(line)
        print(f"{self.BLUE}[{self._timestamp()}] DEBUG:{self.RESET} {message}")
    
    # =========================================================================
    # Specialized Logging
    # =========================================================================
    
    def state(self, message):
        """Log state transition."""
        line = self._format_log_line("STATE", message, "[*]")
        self._write_raw(line)
        print(f"{self.BOLD}{self.BLUE}[{self._timestamp()}] [*] STATE:{self.RESET} {message}")
    
    def detect(self, message):
        """Log human detection event."""
        line = self._format_log_line("DETECT", message, "[D]")
        self._write_raw(line)
        print(f"{self.BOLD}{self.YELLOW}[{self._timestamp()}] [D] DETECT:{self.RESET} {message}")
    
    def mission(self, message):
        """Log mission event."""
        line = self._format_log_line("MISSION", message, "[M]")
        self._write_raw(line)
        print(f"{self.BOLD}{self.CYAN}[{self._timestamp()}] [M] MISSION:{self.RESET} {message}")
    
    def telemetry(self, message):
        """Log telemetry data."""
        line = self._format_log_line("TELEM", message)
        self._write_raw(line)
        # Telemetry is verbose, only write to file by default
    
    def output(self, message):
        """Log general output (stdout capture)."""
        if message.strip():
            line = self._format_log_line("OUTPUT", message.strip())
            self._write_raw(line)
    
    # =========================================================================
    # Section Headers
    # =========================================================================
    
    def header(self, message):
        """Log section header."""
        border = "=" * 60
        header_text = f"""
{border}
  {message}
{border}
"""
        self._write_raw(header_text)
        print(f"\n{self.BOLD}{'=' * 60}{self.RESET}")
        print(f"{self.BOLD}  {message}{self.RESET}")
        print(f"{self.BOLD}{'=' * 60}{self.RESET}\n")
    
    def section(self, title):
        """Log a smaller section divider."""
        self._write_raw(f"\n--- {title} ---\n")
        print(f"\n{self.CYAN}--- {title} ---{self.RESET}")
    
    # =========================================================================
    # Legacy compatibility (for existing code)
    # =========================================================================
    
    def command(self, cmd):
        """Legacy: Log received command (use rx() instead)."""
        self.rx(cmd)
    
    def exception(self, message, exc_info=True):
        """Log exception with traceback."""
        import traceback
        self._error_count += 1
        tb = traceback.format_exc() if exc_info else ""
        self._write_raw(f"\n{'!' * 60}\n")
        line = self._format_log_line("EXCEPTION", message, "[XX]")
        self._write_raw(line)
        if tb and tb.strip() != "NoneType: None":
            self._write_raw(f"Traceback:\n{tb}\n")
        self._write_raw(f"{'!' * 60}\n\n")
        print(f"{self.RED}[{self._timestamp()}] [XX] EXCEPTION:{self.RESET} {message}")
        if tb and tb.strip() != "NoneType: None":
            print(f"{self.RED}{tb}{self.RESET}")
    
    # =========================================================================
    # Session Management
    # =========================================================================
    
    def close(self):
        """Close the session logger and write footer with summary."""
        if self._file_handle:
            end_time = datetime.now()
            duration = end_time - self._session_start if self._session_start else "N/A"
            
            summary = f"""

================================================================================
                           SESSION ENDED
================================================================================
  End Time     : {end_time.strftime('%Y-%m-%d %H:%M:%S')}
  Duration     : {str(duration).split('.')[0]}
--------------------------------------------------------------------------------
  STATISTICS:
    Commands Received (RX) : {self._rx_count}
    Responses Sent (TX)    : {self._tx_count}
    Errors                 : {self._error_count}
================================================================================
"""
            self._write_raw(summary)
            
            with self._write_lock:
                try:
                    self._file_handle.close()
                except Exception:
                    pass
            self._file_handle = None
    
    @classmethod
    def setup_global_logging(cls):
        """
        Set up global logging to capture all output to session file.
        Call this at the start of main.py to capture everything.
        
        Returns:
            SessionFileLogger instance
        """
        logger = cls()
        
        # Also capture stdout/stderr to log file
        sys.stdout = TeeStream(sys.stdout, logger)
        sys.stderr = TeeStream(sys.stderr, logger, is_stderr=True)
        
        return logger


class TeeStream:
    """
    Stream wrapper that writes to both original stream and log file.
    """
    
    def __init__(self, original_stream, logger, is_stderr=False):
        self.original = original_stream
        self.logger = logger
        self.is_stderr = is_stderr
        self._buffer = ""
    
    def write(self, message):
        self.original.write(message)
        
        # Buffer incomplete lines
        self._buffer += message
        
        # Process complete lines
        while '\n' in self._buffer:
            line, self._buffer = self._buffer.split('\n', 1)
            if line.strip():
                # Skip if already formatted (from our logger)
                if not line.startswith('[') or 'INFO:' not in line:
                    if self.is_stderr:
                        # Log to file
                        self.logger._write_raw(
                            self.logger._format_log_line("STDERR", line.strip(), "[!]")
                        )
                        # Forward to radio telemetry (TX laptop)
                        if self.logger._radio_telemetry:
                            # Detect severity from autopilot messages
                            if 'CRITICAL:autopilot:' in line or 'PreArm:' in line or 'Arm:' in line:
                                # Extract just the message part
                                if 'CRITICAL:autopilot:' in line:
                                    msg = line.split('CRITICAL:autopilot:', 1)[1].strip()
                                    self.logger._radio_telemetry.critical(f"FC: {msg}")
                                else:
                                    self.logger._radio_telemetry.warning(line.strip())
                            elif 'ERROR:' in line:
                                self.logger._radio_telemetry.error(line.strip())
                            elif 'WARNING:' in line or 'WARN:' in line:
                                self.logger._radio_telemetry.warning(line.strip())
                            else:
                                self.logger._radio_telemetry.info(line.strip())
                    else:
                        self.logger.output(line)
    
    def flush(self):
        self.original.flush()
    
    def fileno(self):
        return self.original.fileno()


# Global logger instance for easy import
session_logger = None


def get_session_logger():
    """Get or create the global session logger."""
    global session_logger
    if session_logger is None:
        session_logger = SessionFileLogger()
    return session_logger


def init_session_logging():
    """
    Initialize session logging. Call at the start of main.py.
    
    Returns:
        SessionFileLogger instance
    """
    global session_logger
    session_logger = SessionFileLogger.setup_global_logging()
    return session_logger
