#!/bin/bash
#
# View saved LoRa service session logs
#
# Usage:
#   ./view_session_logs.sh              - List available session logs
#   ./view_session_logs.sh latest       - View the most recent session log
#   ./view_session_logs.sh today        - List today's sessions
#   ./view_session_logs.sh <filename>   - View a specific log file
#   ./view_session_logs.sh tail         - Tail the latest session log
#   ./view_session_logs.sh search <term> - Search all logs for a term
#   ./view_session_logs.sh errors       - Show only errors from latest session
#   ./view_session_logs.sh commands     - Show only RX/TX commands from latest session
#

LOG_DIR="/home/dart2/duplicate_scout_drone/working_quad/logs/sessions"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

print_header() {
    echo -e "${CYAN}========================================"
    echo -e "  LoRa Service - Session Log Viewer"
    echo -e "========================================${NC}"
    echo ""
}

list_all_sessions() {
    print_header
    echo -e "${GREEN}Available session logs:${NC}"
    echo ""
    
    if [ ! -d "$LOG_DIR" ]; then
        echo -e "${RED}No session logs found. Log directory doesn't exist.${NC}"
        echo -e "Logs will be created at: $LOG_DIR"
        exit 1
    fi
    
    # List all log files grouped by date
    for date_dir in $(ls -1r "$LOG_DIR" 2>/dev/null); do
        if [ -d "$LOG_DIR/$date_dir" ]; then
            echo -e "${BOLD}${BLUE}$date_dir:${NC}"
            for log_file in $(ls -1r "$LOG_DIR/$date_dir"/*.log 2>/dev/null); do
                filename=$(basename "$log_file")
                size=$(du -h "$log_file" | cut -f1)
                lines=$(wc -l < "$log_file")
                echo -e "  ${GREEN}$filename${NC} ($size, $lines lines)"
            done
            echo ""
        fi
    done
    
    total=$(find "$LOG_DIR" -name "*.log" 2>/dev/null | wc -l)
    echo -e "${YELLOW}Total sessions: $total${NC}"
}

view_latest() {
    latest=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -f2- -d" ")
    
    if [ -z "$latest" ]; then
        echo -e "${RED}No session logs found.${NC}"
        exit 1
    fi
    
    print_header
    echo -e "${GREEN}Viewing latest session:${NC} $latest"
    echo -e "${YELLOW}Press 'q' to exit${NC}"
    echo ""
    
    less -R "$latest"
}

tail_latest() {
    latest=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -f2- -d" ")
    
    if [ -z "$latest" ]; then
        echo -e "${RED}No session logs found.${NC}"
        exit 1
    fi
    
    print_header
    echo -e "${GREEN}Tailing latest session:${NC} $latest"
    echo -e "${YELLOW}Press Ctrl+C to exit${NC}"
    echo ""
    
    tail -f "$latest"
}

list_today() {
    today=$(date +"%Y-%m-%d")
    today_dir="$LOG_DIR/$today"
    
    print_header
    echo -e "${GREEN}Today's sessions ($today):${NC}"
    echo ""
    
    if [ ! -d "$today_dir" ]; then
        echo -e "${YELLOW}No sessions today.${NC}"
        exit 0
    fi
    
    for log_file in $(ls -1r "$today_dir"/*.log 2>/dev/null); do
        filename=$(basename "$log_file")
        size=$(du -h "$log_file" | cut -f1)
        lines=$(wc -l < "$log_file")
        # Extract time from filename (session_HH-MM-SS.log)
        time_part=$(echo "$filename" | sed 's/session_//' | sed 's/.log//' | tr '-' ':')
        echo -e "  ${GREEN}$time_part${NC} - $filename ($size, $lines lines)"
    done
}

view_file() {
    file_path="$1"
    
    # Check if it's a full path or just a filename
    if [ ! -f "$file_path" ]; then
        # Try to find it in the log directory
        found=$(find "$LOG_DIR" -name "$file_path" -type f 2>/dev/null | head -1)
        if [ -n "$found" ]; then
            file_path="$found"
        else
            echo -e "${RED}File not found: $file_path${NC}"
            exit 1
        fi
    fi
    
    print_header
    echo -e "${GREEN}Viewing:${NC} $file_path"
    echo -e "${YELLOW}Press 'q' to exit${NC}"
    echo ""
    
    less -R "$file_path"
}

search_logs() {
    term="$1"
    
    if [ -z "$term" ]; then
        echo -e "${RED}Usage: $0 search <term>${NC}"
        exit 1
    fi
    
    print_header
    echo -e "${GREEN}Searching for:${NC} '$term'"
    echo ""
    
    grep -rn --color=always "$term" "$LOG_DIR" 2>/dev/null || echo -e "${YELLOW}No matches found.${NC}"
}

show_errors() {
    latest=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -f2- -d" ")
    
    if [ -z "$latest" ]; then
        echo -e "${RED}No session logs found.${NC}"
        exit 1
    fi
    
    print_header
    echo -e "${GREEN}Errors from:${NC} $latest"
    echo ""
    
    grep -E "\[ERROR|\[CRITICAL|\[EXCEPTION|\[X\]|\[XX\]|ERROR:" "$latest" --color=always || echo -e "${YELLOW}No errors found in this session.${NC}"
}

show_commands() {
    latest=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -f2- -d" ")
    
    if [ -z "$latest" ]; then
        echo -e "${RED}No session logs found.${NC}"
        exit 1
    fi
    
    print_header
    echo -e "${GREEN}Commands (RX/TX) from:${NC} $latest"
    echo -e "${YELLOW}Format: [timestamp] << RX (received) | >> TX (sent)${NC}"
    echo ""
    
    # Extract RX and TX commands, showing them clearly
    grep -E "\[RX\] Command:|\[TX\]|>> \[TX|<< \[RX" "$latest" | \
        sed 's/.*\(\[.*\]\).*STATE: \[RX\] Command: \(.*\)/\1 << RX: \2/' | \
        sed 's/.*\(\[.*\]\).*INFO: \[TX\] \(.*\)/\1 >> TX: \2/' | \
        grep -v "OUTPUT" || echo -e "${YELLOW}No commands found in this session.${NC}"
}

# Main
case "$1" in
    "")
        list_all_sessions
        ;;
    "latest")
        view_latest
        ;;
    "today")
        list_today
        ;;
    "tail")
        tail_latest
        ;;
    "search")
        search_logs "$2"
        ;;
    "errors")
        show_errors
        ;;
    "commands")
        show_commands
        ;;
    *)
        view_file "$1"
        ;;
esac
