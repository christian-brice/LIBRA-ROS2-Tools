#!/usr/bin/env bash

# Measures the average bandwidth and publishing frequency for all active ROS 2
# topics and displays the results in a formatted table.
#
# Usage:
#   ./ros2-network-summary.sh [--csv [filename]] [--debug]
#
# Disclaimer:
#   This script was vibe-coded using Gemini and manually tweaked by Christian Brice.

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

# How long to wait for a ros2 command output to print something
# (useful if a topic has and especially low publishing rate)
sampling_time_sec=5

output_temp_file=$(mktemp)  # capture command output in a temporary file
csv_file="network_summary.csv"  # default filename for "--csv" flag

color_echo() {
    local level=$1
    shift
    local msg="$*"

    case "$level" in
        INFO)
            echo -e "\033[1;36m[INFO]\033[0;36m $msg\033[0m"
            ;;
        WARN)
            echo -e "\033[1;33m[WARN]\033[0;33m $msg\033[0m"
            ;;
        ERROR)
            echo -e "\033[1;31m[ERROR]\033[0;31m $msg\033[0m"
            ;;
        DEBUG)
            echo -e "\033[1;90m[DEBUG]\033[0;90m $msg\033[0m"
            ;;
        *)
            echo "[UNKNOWN] $msg"
            ;;
    esac
}

# ==================== SCRIPT START ====================

color_echo INFO "Sampling active ROS2 topics..."

# ==================== (0) ARG PARSING AND PRELIMINARY ACTIONS ====================

#-------------------------
csv_mode=false
debug_mode=false
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --debug)
            debug_mode=true
            shift  # consume the flag
            ;;
        --csv)
            csv_mode=true
            # Check the next argument, if any, for a filename
            if [[ -n "$2" && ! "$2" =~ ^-- ]]; then
                csv_file="$2"
                # If it doesn't end in ".csv", append it
                if [[ ! "$csv_file" =~ \.csv$ ]]; then
                    csv_file+=".csv"
                fi
                shift  # consume the flag
                shift  # consume the filename
            else
                shift  # only consume the flag
            fi
            ;;
        *)
            color_echo WARN "Unrecognized option: '$1'"
            shift
            ;;
    esac
done
#-------------------------
if [ "$debug_mode" = true ]; then
    color_echo INFO "Debug mode enabled"
fi
if [ "$csv_mode" = true ]; then
    color_echo INFO "CSV mode enabled. Output file: $csv_file"
fi
#-------------------------
# Get a list of all ROS 2 topics (used mainly in Section 1)
topics=$(ros2 topic list)
#-------------------------
# Estimate time to completion
num_topics=$(echo "$topics" | wc -l)
color_echo INFO "Estimated time to complete: $((num_topics * 2 * sampling_time_sec)) s"
sleep 1;  # let user see this message

# ==================== (1) LOOP THROUGH AVAILABLE TOPICS ====================

#-------------------------
START_TIME=$(date +%s.%N)
SLEEP_TIME=0
#-------------------------
if [ "$csv_mode" = true ]; then
    echo "\"Topic Name\",\"Bandwidth (KiB/s)\",\"Frequency (Hz)\"" > "$csv_file"
fi
echo "----------------------------------------------------------------------------------------"
echo "  Topic Name                                         Bandwidth        Frequency"
echo "----------------------------------------------------------------------------------------"
for topic in $topics
do
    #-------------------------
    if [ "$debug_mode" = true ]; then
        color_echo DEBUG "Processing topic: $topic"
    fi

    # Check if the topic has any active publishers
    publisher_count=$(ros2 topic info "$topic" 2>/dev/null | grep 'Publisher count' | awk '{print $3}')
    #-------------------------
    if [ "$publisher_count" -gt 0 ]; then
        # ==================== (1a) PARSE OUTPUT FROM ROS COMMANDS ====================

        #-------------------------
        # Poll bandwidth and extract the value w/ unit
        ( ros2 topic bw "$topic" 2>&1 & sleep "$sampling_time_sec"; kill $! ) > "$output_temp_file" 2>/dev/null
        
        bw_summary_line=$(grep -a 'messages' "$output_temp_file" 2>/dev/null | tail -n 1)
        bw_val=$(echo "$bw_summary_line" | awk '{print $1}')
        bw_unit=$(echo "$bw_summary_line" | awk '{print $2}')
        #-------------------------
        # Poll publishing frequency and extract the value w/ unit
        ( ros2 topic hz "$topic" 2>&1 & sleep "$sampling_time_sec"; kill $! ) > "$output_temp_file" 2>/dev/null
        
        hz_summary_line=$(grep -a 'average rate:' "$output_temp_file" 2>/dev/null | tail -n 1)
        hz_val=$(echo "$hz_summary_line" | awk '{print $3}')
        #-------------------------

        # ==================== (1b) SANITIZE AND PRINT DATA ====================

        #-------------------------
        # Normalize bandwidth units to KiB/s
        bw_kib=0.00
        if [ ! -z "$bw_val" ]; then
            case "$bw_unit" in
                "MiB/s")
                    bw_kib=$(echo "$bw_val * 1024" | bc)
                    ;;
                "GiB/s")
                    bw_kib=$(echo "$bw_val * 1024 * 1024" | bc)
                    ;;
                *)
                    bw_kib="$bw_val"
                    ;;
            esac
        fi
        # Remove trailing 's' from the frequency value to prevent printf errors.
        hz_val_clean=$(echo "$hz_val" | sed 's/s$//')
        #-------------------------
        # Print the data in a single, formatted line
        printf "%-50s %10.2f KiB/s %10.2f Hz\n" "$topic" "$bw_kib" "$hz_val_clean"
        if [ "$csv_mode" = true ]; then
            printf "%s,%.2f,%.2f\n" "$topic" "$bw_kib" "$hz_val_clean" >> "$csv_file"
        fi
    else
        # Also print a line for topics that aren't active
        printf "%-50s %10s KiB/s %10s Hz\n" "$topic" "--" "--"
        if [ "$csv_mode" = true ]; then
            printf "%s,%.2f,%.2f\n" "$topic" "0.00" "0.00" >> "$csv_file"
        fi
        #-------------------------
    fi
done

echo "----------------------------------------------------------------------------------------"

color_echo INFO "Sampling active ROS2 topics... SUCCESS"
#-------------------------

# ==================== SCRIPT END ====================

END_TIME=$(date +%s.%N)
ELAPSED_TIME=$(echo "$END_TIME - $START_TIME - $SLEEP_TIME" | bc -l)
F_ELAPSED_TIME=$(printf "Script finished in %.2f s" "$ELAPSED_TIME")
color_echo INFO "$F_ELAPSED_TIME"

# Cleanup
rm -f "$output_temp_file"
if [ "$csv_mode" = true ]; then
    color_echo INFO "CSV file saved to: $csv_file"
fi
