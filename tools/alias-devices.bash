#!/usr/bin/env bash

set -e  # exit on any error

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

declare -A device_map=(
    ["SerialWater"]="Arduino SA"
    ["SerialServo"]="Seeed Technology Co., Ltd."
    ["LIDAR"]="Prolific Technology, Inc."
)

declare -A device_descriptions=(
    ["SerialWater"]="Arduino_Nano_Every"
    ["SerialServo"]="Seeed_XIAO_M0"
    ["LIDAR"]="USB-Serial_Controller"
)

declare -A symlinks=(
    ["SerialWater"]="/dev/water"
    ["SerialServo"]="/dev/manip"
    ["LIDAR"]="/dev/lidar"
)

RULES_FILE="/etc/udev/rules.d/99-libra.rules"

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
        *)
            echo "[UNKNOWN] $msg"
            ;;
    esac
}

prompt_yes_no() {
    local prompt_msg="$1"
    local answer
    while true; do
        read -rp "$prompt_msg [y/n]: " answer
        case "$answer" in
            [Yy]*) return 0 ;;
            [Nn]*) return 1 ;;
            *) echo "Please answer 'y' or 'n'" ;;
        esac
    done
}

check_device_match() {
    local manufacturer="$1"
    local model="$2"
    
    for device_key in "${!device_map[@]}"; do
        if [[ "$manufacturer" == "${device_map[$device_key]}" ]] && [[ "$model" == "${device_descriptions[$device_key]}" ]]; then
            echo "$device_key"
            return 0
        fi
    done
    return 1
}

get_device_ids() {
    local dev_path="$1"
    local props="$2"
    
    # Extract vendor and product IDs
    local vendor_id=$(echo "$props" | grep -i '^ID_VENDOR_ID=' | head -n1 | cut -d= -f2-)
    local product_id=$(echo "$props" | grep -i '^ID_MODEL_ID=' | head -n1 | cut -d= -f2-)
    
    if [[ -z "$vendor_id" || -z "$product_id" ]]; then
        color_echo ERROR "Missing vendor or product ID for $dev_path"
        return 1
    fi
    
    echo "$vendor_id:$product_id"
}

# ==================== SCRIPT START ====================

color_echo INFO "Starting scan of /dev/ttyACM* and /dev/ttyUSB* devices..."

declare -A found_devices
declare -A device_ids

# Iterate over all serial devices
for dev_path in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$dev_path" ] || continue

    echo ""  # empty newline between entries
    color_echo INFO "Checking device $dev_path..."

    # Get udev properties with error capture
    if ! props=$(udevadm info -q property -n "$dev_path" 2>&1); then
        color_echo ERROR "Failed to get udev properties:"
        echo "$props"
        exit 1
    fi

    # Extract manufacturer and model info
    manufacturer=$(echo "$props" | grep -i '^ID_VENDOR_FROM_DATABASE=' | head -n1 | cut -d= -f2-)
    model=$(echo "$props" | grep -i '^ID_MODEL_FROM_DATABASE=' | head -n1 | cut -d= -f2-)

    # Fallback if *_FROM_DATABASE not found
    if [[ -z "$manufacturer" ]]; then
        manufacturer=$(echo "$props" | grep -i '^ID_VENDOR=' | head -n1 | cut -d= -f2-)
    fi
    if [[ -z "$model" ]]; then
        model=$(echo "$props" | grep -i '^ID_MODEL=' | head -n1 | cut -d= -f2-)
    fi

    if [[ -z "$manufacturer" || -z "$model" ]]; then
        color_echo ERROR "Missing manufacturer or model info! (bad port? invalid/corrupted device?)"
        exit 1
    fi

    color_echo INFO "  Manufacturer: $manufacturer"
    color_echo INFO "  Model: $model"

    # Get vendor and product IDs for rules file
    if ! ids=$(get_device_ids "$dev_path" "$props"); then
        exit 1
    fi

    color_echo INFO "  IDs: $ids"

    # Match device by manufacturer & description (model)
    if ! matched_device=$(check_device_match "$manufacturer" "$model"); then
        color_echo WARN "No matching device type found; skipping"
        continue
    fi

    # Make sure we only map one device per device type (in case multiple found)
    if [[ -n "${found_devices[$matched_device]}" ]]; then
        color_echo WARN "Already found a device; skipping additional device"
        continue
    fi

    color_echo INFO "Matched device type: $matched_device"

    found_devices[$matched_device]="$dev_path"
    device_ids[$matched_device]="$ids"
done

# Generate udev rules file if any devices were found
rules_created=false
if [[ ${#found_devices[@]} -gt 0 ]]; then
    echo ""
    color_echo INFO "Creating udev rules file: $RULES_FILE"
    
    # Check if rules file already exists
    if [[ -f "$RULES_FILE" ]]; then
        color_echo WARN "Rules file already exists"
        if ! prompt_yes_no "Overwrite existing rules file?"; then
            color_echo INFO "Skipping rules file creation"
            exit 0
        fi
    fi
    
    # Create rules file header
    cat > /tmp/libra_rules << 'EOF'
# /etc/udev/rules.d/99-libra.rules
# Auto-generated udev rules for LIBRA project serial (USB) devices

EOF

    # Generate rules for each found device
    for device_key in "${!found_devices[@]}"; do
        vendor_id=$(echo "${device_ids[$device_key]}" | cut -d: -f1)
        product_id=$(echo "${device_ids[$device_key]}" | cut -d: -f2)
        symlink_name=$(basename "${symlinks[$device_key]}")
        
        cat >> /tmp/libra_rules << EOF
# ${device_key} - ${device_map[$device_key]} ${device_descriptions[$device_key]}
SUBSYSTEM=="tty", ATTRS{idVendor}=="$vendor_id", ATTRS{idProduct}=="$product_id", SYMLINK+="$symlink_name", MODE="0666", GROUP="dialout"

EOF
    done
    
    # Install the rules file
    if ! sudo cp /tmp/libra_rules "$RULES_FILE"; then
        color_echo ERROR "Failed to create rules file! (did you run the script with sudo?)"
        rm -f /tmp/libra_rules
        exit 1
    fi
    
    rm -f /tmp/libra_rules
    color_echo INFO "Rules file created successfully"
    
    # Reload and trigger udev
    color_echo INFO "Reloading udev rules..."
    if ! sudo udevadm control --reload-rules; then
        color_echo ERROR "Failed to reload udev rules!"
        exit 1
    fi

    color_echo INFO "Triggering udev..."
    if ! sudo udevadm trigger; then
        color_echo ERROR "Failed to trigger udev!"
        exit 1
    fi

    rules_created=true
fi

# ==================== SCRIPT END ====================

if [[ "$rules_created" == true ]]; then
    color_echo INFO "Done! Created persistent rules for the following devices:"
    for dev_key in "${!found_devices[@]}"; do
        color_echo INFO "  $dev_key: ${symlinks[$dev_key]} -> ${found_devices[$dev_key]} (${device_ids[$dev_key]})"
    done
    
    echo "Symlinks will be automatically created on device plug-in"
else
    color_echo WARN "No matching devices found"
fi