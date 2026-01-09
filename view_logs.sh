#!/bin/bash
#
# Real-time log viewer for LoRa Drone Service
# Shows live logs with timestamps and saves to a file for debugging
#
# Usage:
#   ./view_logs.sh              - View logs and save to default file
#   ./view_logs.sh custom.txt   - View logs and save to custom file
#   ./view_logs.sh --no-save    - Only view logs, don't save
#

SERVICE_NAME="lora"
LOG_DIR="/home/dart2/duplicate_scout_drone/working_quad/logs"
DATE_STAMP=$(date +"%Y-%m-%d")
TIME_STAMP=$(date +"%H-%M-%S")

# Create logs directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Determine output file
if [ "$1" == "--no-save" ]; then
    SAVE_TO_FILE=false
    LOG_FILE=""
elif [ -n "$1" ]; then
    SAVE_TO_FILE=true
    LOG_FILE="$LOG_DIR/$1"
else
    SAVE_TO_FILE=true
    LOG_FILE="$LOG_DIR/lora_${DATE_STAMP}_${TIME_STAMP}.txt"
fi

# Colors for better visibility
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================"
echo -e "  LoRa Drone Service - Live Log Viewer"
echo -e "========================================${NC}"
echo ""
echo -e "${GREEN}Service:${NC} $SERVICE_NAME"
echo -e "${GREEN}Date:${NC} $(date)"

if [ "$SAVE_TO_FILE" = true ]; then
    echo -e "${GREEN}Saving to:${NC} $LOG_FILE"
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop viewing logs${NC}"
    echo ""
    
    # Write header to log file
    {
        echo "========================================"
        echo "LoRa Drone Service Log"
        echo "Started: $(date)"
        echo "Service: $SERVICE_NAME"
        echo "========================================"
        echo ""
    } > "$LOG_FILE"
    
    echo -e "${BLUE}--- Live Logs (with timestamps) ---${NC}"
    echo ""
    
    # Stream logs to both terminal and file with timestamps
    # Using journalctl with follow mode, output timestamps, and tee to file
    sudo journalctl -u "$SERVICE_NAME" -f --output=short-iso 2>&1 | while IFS= read -r line; do
        # Print to terminal with colors for certain keywords
        if echo "$line" | grep -qiE "error|fail|exception"; then
            echo -e "${RED}${line}${NC}"
        elif echo "$line" | grep -qiE "warn"; then
            echo -e "${YELLOW}${line}${NC}"
        elif echo "$line" | grep -qiE "success|ok|armed|connected"; then
            echo -e "${GREEN}${line}${NC}"
        elif echo "$line" | grep -qiE "command|cmd|received|rx"; then
            echo -e "${CYAN}${line}${NC}"
        else
            echo "$line"
        fi
        # Also save to file (without color codes)
        echo "$line" >> "$LOG_FILE"
    done
else
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop viewing logs${NC}"
    echo ""
    echo -e "${BLUE}--- Live Logs (with timestamps) ---${NC}"
    echo ""
    
    # Just stream logs to terminal
    sudo journalctl -u "$SERVICE_NAME" -f --output=short-iso 2>&1 | while IFS= read -r line; do
        if echo "$line" | grep -qiE "error|fail|exception"; then
            echo -e "${RED}${line}${NC}"
        elif echo "$line" | grep -qiE "warn"; then
            echo -e "${YELLOW}${line}${NC}"
        elif echo "$line" | grep -qiE "success|ok|armed|connected"; then
            echo -e "${GREEN}${line}${NC}"
        elif echo "$line" | grep -qiE "command|cmd|received|rx"; then
            echo -e "${CYAN}${line}${NC}"
        else
            echo "$line"
        fi
    done
fi
