#!/bin/bash
#
# Setup script for LoRa autonomous drone service
# Run with: sudo bash setup_service.sh
#

set -e

# Configurationsudo bash setup_service.sh
SERVICE_NAME="lora"
WORKING_DIR="/home/dart/quadtest"
PYTHON_PATH="/usr/bin/python3"
USER_NAME="dart"
GROUP_NAME="dart"

echo "========================================"
echo "LoRa Drone Service Setup"
echo "========================================"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run as root (sudo bash setup_service.sh)"
    exit 1
fi

# Check if main.py exists
if [ ! -f "${WORKING_DIR}/main.py" ]; then
    echo "ERROR: ${WORKING_DIR}/main.py not found"
    exit 1
fi

echo "[1/4] Creating systemd service file..."

cat > /etc/systemd/system/${SERVICE_NAME}.service << EOF
[Unit]
Description=LoRa Autonomous Drone Controller
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
RestartSec=5
StandardOutput=journal
StandardError=journal

# Environment
Environment=PYTHONUNBUFFERED=1

# Hardware access - needed for serial ports
SupplementaryGroups=dialout

# Give time for USB devices to initialize
ExecStartPre=/bin/sleep 5

[Install]
WantedBy=multi-user.target
EOF

echo "    Created: /etc/systemd/system/${SERVICE_NAME}.service"

echo "[2/4] Reloading systemd..."
systemctl daemon-reload

echo "[3/4] Enabling service at boot..."
systemctl enable ${SERVICE_NAME}.service

echo "[4/4] Starting service..."
systemctl start ${SERVICE_NAME}.service

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Service status:"
systemctl status ${SERVICE_NAME}.service --no-pager || true
echo ""
echo "Useful commands:"
echo "  sudo systemctl status ${SERVICE_NAME}     - Check status"
echo "  sudo systemctl restart ${SERVICE_NAME}    - Restart service"
echo "  sudo systemctl stop ${SERVICE_NAME}       - Stop service"
echo "  sudo journalctl -u ${SERVICE_NAME} -f     - View live logs"
echo ""
