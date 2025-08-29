#!/bin/bash

# Network Configuration Script for Fairino Robot Connection
# This script helps resolve connection issues and ROS node crashes due to reconnection problems
# Reference: https://www.youtube.com/watch?v=TmgsO-ZCiwU

set -e  # Exit on any error

echo "=== Fairino Robot Network Configuration ==="
echo "This script will configure your network interface for stable robot connection."
echo

# Function to print section headers
print_section() {
    echo
    echo "==== $1 ===="
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check required tools
print_section "Checking required tools"
for tool in ip nmcli ethtool; do
    if command_exists "$tool"; then
        echo "✓ $tool is available"
    else
        echo "✗ $tool is not available. Please install network-manager and ethtool."
        exit 1
    fi
done

print_section "Identifying Network Interfaces"

echo "Available network interfaces:"
ip link show

echo
echo "Network device status:"
nmcli device status

echo
echo "Detecting Ethernet interfaces..."

# Find all Ethernet interfaces (excluding loopback, wireless, docker, etc.)
ETH_INTERFACES=$(ip link show | grep -E '^[0-9]+:' | grep -v 'lo:' | grep -v 'wl' | grep -v 'docker' | grep -v 'br-' | grep -v 'veth' | awk -F': ' '{print $2}' | grep -E '^(en|eth)')

if [ -z "$ETH_INTERFACES" ]; then
    echo "No Ethernet interfaces found. Available interfaces:"
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print "  " $2}'
    echo
    echo "Please manually specify the interface name:"
    read -p "Enter interface name: " INTERFACE
    if [ -z "$INTERFACE" ]; then
        echo "No interface specified. Exiting."
        exit 1
    fi
else
    # If multiple interfaces found, let user choose
    ETH_ARRAY=($ETH_INTERFACES)
    if [ ${#ETH_ARRAY[@]} -eq 1 ]; then
        INTERFACE="${ETH_ARRAY[0]}"
        echo "Found Ethernet interface: $INTERFACE"
    else
        echo "Multiple Ethernet interfaces found:"
        for i in "${!ETH_ARRAY[@]}"; do
            echo "  $((i+1)). ${ETH_ARRAY[$i]}"
        done
        echo
        read -p "Select interface (1-${#ETH_ARRAY[@]}): " choice
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#ETH_ARRAY[@]}" ]; then
            INTERFACE="${ETH_ARRAY[$((choice-1))]}"
        else
            echo "Invalid selection. Using first interface: ${ETH_ARRAY[0]}"
            INTERFACE="${ETH_ARRAY[0]}"
        fi
    fi
fi

echo "Using interface: $INTERFACE"

print_section "Current Interface Information"

echo "Interface details:"
ip link show "$INTERFACE"

echo
echo "Current ethtool settings:"
if ethtool "$INTERFACE" 2>/dev/null; then
    echo "Interface is UP and accessible"
else
    echo "Warning: Interface may be DOWN or inaccessible"
fi

print_section "Driver and Firmware Information"

echo "Driver/firmware information for $INTERFACE:"
if sudo ethtool -i "$INTERFACE" 2>/dev/null; then
    echo "Driver information retrieved successfully"
else
    echo "Warning: Could not retrieve driver information"
fi

print_section "Network Configuration"

echo "This will configure the interface for stable robot connection:"
echo "1. Force 100 Mb/s Full Duplex (disable autonegotiation)"
echo "2. Disable Energy-Efficient Ethernet (EEE)"
echo
read -p "Continue with configuration? (y/N): " confirm

if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "Configuration cancelled."
    exit 0
fi

print_section "Forcing 100 Mb/s Full Duplex"

echo "Setting $INTERFACE to 100 Mb/s Full Duplex with autonegotiation off..."
if sudo ethtool -s "$INTERFACE" speed 100 duplex full autoneg off 2>/dev/null; then
    echo "✓ Speed/duplex configuration applied"
    
    # Wait a moment for settings to take effect
    sleep 2
    
    echo "Verifying configuration:"
    ethtool "$INTERFACE" | grep -E "(Speed|Duplex|Auto-negotiation)"
else
    echo "⚠ Warning: Could not set speed/duplex. This may not be supported by your NIC or switch."
    echo "This is not necessarily a problem - continuing with EEE configuration..."
fi

print_section "Disabling Energy-Efficient Ethernet (EEE)"

echo "Disabling EEE on $INTERFACE..."
if sudo ethtool --set-eee "$INTERFACE" eee off 2>/dev/null; then
    echo "✓ EEE disabled successfully"
    
    echo "Verifying EEE configuration:"
    sudo ethtool --show-eee "$INTERFACE" 2>/dev/null || echo "EEE status query not supported (this is normal for some NICs)"
else
    echo "⚠ Note: EEE configuration not supported by this NIC (this is normal for many network cards)"
fi

print_section "Configuration Complete"

echo "Network configuration completed for interface: $INTERFACE"
echo
echo "Summary of changes:"
echo "- Attempted to set 100 Mb/s Full Duplex with autonegotiation off"
echo "- Attempted to disable Energy-Efficient Ethernet (EEE)"
echo
echo "Current interface status:"
ethtool "$INTERFACE" | grep -E "(Speed|Duplex|Auto-negotiation)" || echo "Status unavailable"

echo
echo "Tips:"
echo "- If connection issues persist, try a different network cable"
echo "- Check your switch/router supports these settings"
echo "- Monitor robot connection stability after these changes"
echo "- These settings may not persist after reboot - consider making them permanent"

echo
echo "To make these settings permanent, add them to /etc/network/interfaces or use NetworkManager"
echo "Configuration script completed successfully!"
