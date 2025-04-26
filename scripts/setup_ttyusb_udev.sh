#!/bin/bash

# Script to set up udev rules for ttyUSB0 and ttyUSB1 devices
# Gives chmod 777 permissions to these devices when plugged in
# Usage: 
#   sudo ./setup_ttyusb_udev.sh enable   - Enable udev rules
#   sudo ./setup_ttyusb_udev.sh disable  - Disable udev rules

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "This script requires root privileges. Please run with sudo."
  exit 1
fi

# Define the udev rule file path
UDEV_RULE_FILE="/etc/udev/rules.d/99-ttyusb-permissions.rules"

# Check for command line arguments
if [ $# -ne 1 ]; then
  echo "Usage: $0 [enable|disable]"
  exit 1
fi

case "$1" in
  enable)
    # Create the udev rule file
    echo "Creating udev rule file..."
    cat > "$UDEV_RULE_FILE" << 'EOF'
# Set permissions for ttyUSB0 and ttyUSB1 devices
KERNEL=="ttyUSB[0-1]", MODE="0777"

# U2D2 specific rule (FT583QPG) with dialout group
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT583QPG", GROUP="dialout", MODE="0660"
EOF

    # Check if the file was created successfully
    if [ ! -f "$UDEV_RULE_FILE" ]; then
      echo "Failed to create udev rule file."
      exit 1
    fi

    echo "Udev rule file created successfully."

    # Reload udev rules and trigger them
    echo "Applying udev rules..."
    udevadm control --reload-rules
    udevadm trigger

    echo "Done! ttyUSB0 and ttyUSB1 devices will now have 777 permissions when connected."
    echo "The U2D2 device will have dialout group permissions."
    ;;

  disable)
    # Remove the udev rule file if it exists
    if [ -f "$UDEV_RULE_FILE" ]; then
      echo "Removing udev rule file..."
      rm "$UDEV_RULE_FILE"
      
      # Reload udev rules
      echo "Reloading udev rules..."
      udevadm control --reload-rules
      
      echo "Udev rules disabled successfully."
    else
      echo "Udev rule file not found. Nothing to disable."
    fi
    ;;

  *)
    echo "Invalid option: $1"
    echo "Usage: $0 [enable|disable]"
    exit 1
    ;;
esac 