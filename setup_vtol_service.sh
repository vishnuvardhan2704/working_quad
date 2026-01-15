#!/bin/bash
#
# Setup script for LoRa Quadcopter Scout Drone autonomous service
# Run with: sudo bash setup_vtol_service.sh
#
# This configures the systemd service to auto-start
# the scout drone controller on boot, listening for LoRa commands.
# Hardware: RadioLink Pix6 + ArduCopter firmware
#

set -e

# Configuration for this device
SERVICE_NAME="lora"
WORKING_DIR="/home/dart2/duplicate_scout_drone/working_quad"
PYTHON_PATH="/usr/bin/python3"
USER_NAME="dart2"
GROUP_NAME="dart2"

echo "========================================"
echo "LoRa Quadcopter Scout Service Setup"
echo "========================================"
echo "Working Dir: ${WORKING_DIR}"
echo "User: ${USER_NAME}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run as root (sudo bash setup_vtol_service.sh)"
    exit 1
fi

# Check if main.py exists
if [ ! -f "${WORKING_DIR}/main.py" ]; then
    echo "ERROR: ${WORKING_DIR}/main.py not found"
    exit 1
fi

# Check if user exists
if ! id "${USER_NAME}" &>/dev/null; then
    echo "ERROR: User '${USER_NAME}' does not exist"
    exit 1
fi

echo "[1/5] Creating systemd service file..."

cat > /etc/systemd/system/${SERVICE_NAME}.service << EOF
[Unit]
Description=LoRa Quadcopter Scout Drone Controller
After=multi-user.target
After=network.target
Wants=network.target

[Service]
Type=simple
User=${USER_NAME}
Group=${GROUP_NAME}
WorkingDirectory=${WORKING_DIR}
ExecStart=${PYTHON_PATH} ${WORKING_DIR}/main.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# Environment
Environment=PYTHONUNBUFFERED=1

# Hardware access - needed for serial ports (Pixhawk, LoRa radio)
SupplementaryGroups=dialout

# Give time for USB devices to initialize on boot
ExecStartPre=/bin/sleep 10

# Watchdog - restart if unresponsive
WatchdogSec=300

[Install]
WantedBy=multi-user.target
EOF

echo "    Created: /etc/systemd/system/${SERVICE_NAME}.service"

echo "[2/5] Setting up udev rules for consistent serial device names..."

cat > /etc/udev/rules.d/99-vtol-serial.rules << EOF
# Quadcopter Scout Drone Serial Device Rules
# RadioLink Pix6 USB connection (typically shows as ttyACM0)
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk", MODE="0666"
# RadioLink Pix6 alternate USB IDs
SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", ATTRS{idProduct}=="1011", SYMLINK+="ttyPixhawk", MODE="0666"

# 3DR/LoRa Radio (Silicon Labs CP210x or FTDI)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyRadio", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyRadio", MODE="0666"

# Generic fallback for USB serial adapters
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", MODE="0666"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666"
EOF

echo "    Created: /etc/udev/rules.d/99-vtol-serial.rules"

echo "[3/5] Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "[4/5] Reloading systemd and enabling service..."
systemctl daemon-reload
systemctl enable ${SERVICE_NAME}.service

echo "[5/5] Starting service..."
systemctl start ${SERVICE_NAME}.service || true

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Service status:"
systemctl status ${SERVICE_NAME}.service --no-pager -l || true
echo ""
echo "========================================"
echo "Useful Commands:"
echo "========================================"
echo "  sudo systemctl status ${SERVICE_NAME}     - Check status"
echo "  sudo systemctl restart ${SERVICE_NAME}    - Restart service"
echo "  sudo systemctl stop ${SERVICE_NAME}       - Stop service"
echo "  sudo systemctl disable ${SERVICE_NAME}    - Disable auto-start"
echo "  sudo journalctl -u ${SERVICE_NAME} -f     - View live logs"
echo "  sudo journalctl -u ${SERVICE_NAME} -n 100 - View last 100 lines"
echo ""
echo "========================================"
echo "Hardware Check:"
echo "========================================"
echo "Expected devices:"
echo "  RadioLink Pix6: /dev/ttyACM0 (or /dev/ttyPixhawk after replug)"
echo "  LoRa Radio:     /dev/ttyUSB0 (or /dev/ttyRadio after replug)"
echo ""
ls -la /dev/tty{ACM,USB}* 2>/dev/null || echo "  No USB serial devices found yet"
echo ""
