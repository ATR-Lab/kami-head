#!/bin/bash
# configure_serial_device_access.sh - Configure automatic permissions for USB serial devices
#
# DESCRIPTION:
#   This script configures udev rules to automatically set permissions on USB serial devices
#   when they are plugged in. This is needed for the Coffee Buddy robot project to access
#   Dynamixel servo controllers and other USB-to-serial devices without requiring sudo.
#
#   The script handles two types of devices:
#   1. Generic USB serial devices (ttyUSB0, ttyUSB1) - for general serial communication
#   2. U2D2 device (Dynamixel interface) - specific hardware for servo control
#
# ⚠️  SECURITY WARNING:
#   This script sets 777 permissions (read/write for everyone) on ttyUSB devices.
#   This is a SECURITY RISK as it allows any user on the system to access these devices.
#   
#   For better security, consider adding your user to the 'dialout' group instead:
#   sudo usermod -a -G dialout $USER
#   (requires logout/login to take effect)
#
# HARDWARE SUPPORTED:
#   - Generic USB-to-Serial converters (ttyUSB0, ttyUSB1)
#   - U2D2 Dynamixel Interface (ROBOTIS U2D2 servo controller interface)
#   - FTDI-based serial devices (vendor ID 0403, product ID 6014)
#
# USAGE:
#   sudo ./configure_serial_device_access.sh enable    - Enable automatic permissions
#   sudo ./configure_serial_device_access.sh disable   - Remove automatic permissions
#
# REQUIREMENTS:
#   - Root privileges (uses sudo)
#   - udev system (standard on most Linux distributions)
#
# WHAT IT CREATES:
#   Creates /etc/udev/rules.d/99-ttyusb-permissions.rules with device permission rules
#
# WHEN YOU NEED THIS:
#   - You get "Permission denied" errors when accessing /dev/ttyUSB* devices
#   - Your ROS2 nodes can't communicate with Dynamixel servos
#   - You want to avoid using sudo for every serial device access
#
# ALTERNATIVES (More Secure):
#   Instead of 777 permissions, you can:
#   1. Add user to dialout group: sudo usermod -a -G dialout $USER
#   2. Use specific device permissions with group access
#   3. Use udev rules with GROUP="dialout", MODE="0660"
#
# VERIFICATION:
#   After enabling, check: ls -l /dev/ttyUSB*
#   You should see permissions like: crw-rw-rw- (for 777) or crw-rw---- (for 660)

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "❌ Error: This script requires root privileges."
  echo "   Please run with: sudo $0 [enable|disable]"
  exit 1
fi

# Define the udev rule file path
UDEV_RULE_FILE="/etc/udev/rules.d/99-coffee-buddy-serial-permissions.rules"

# Check for command line arguments
if [ $# -ne 1 ]; then
  echo "Usage: $0 [enable|disable]"
  echo ""
  echo "Examples:"
  echo "  sudo $0 enable     # Enable automatic USB serial device permissions"
  echo "  sudo $0 disable    # Remove automatic permissions"
  exit 1
fi

case "$1" in
  enable)
    echo "=== Configuring USB Serial Device Access ==="
    echo ""
    echo "⚠️  Security Warning:"
    echo "   This will set 777 permissions on ttyUSB devices (accessible by everyone)"
    echo "   For better security, consider adding users to 'dialout' group instead"
    echo ""
    
    # Create the udev rule file
    echo "Creating udev rules for USB serial devices..."
    cat > "$UDEV_RULE_FILE" << 'EOF'
# Coffee Buddy Robot - USB Serial Device Permissions
# 
# Generic USB-to-Serial devices (ttyUSB0, ttyUSB1)
# WARNING: 777 permissions allow access by any user (security risk)
KERNEL=="ttyUSB[0-1]", MODE="0777"

# ROBOTIS U2D2 Dynamixel Interface (More secure with dialout group)
# This device is specifically used for Dynamixel servo communication
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT583QPG", GROUP="dialout", MODE="0660"

# Alternative secure rule for all USB serial devices (commented out)
# Uncomment this and comment out the 777 rule above for better security
# KERNEL=="ttyUSB*", GROUP="dialout", MODE="0660"
EOF

    # Check if the file was created successfully
    if [ ! -f "$UDEV_RULE_FILE" ]; then
      echo "❌ Failed to create udev rule file at $UDEV_RULE_FILE"
      exit 1
    fi

    echo "✓ Udev rule file created: $UDEV_RULE_FILE"

    # Reload udev rules and trigger them
    echo ""
    echo "Applying udev rules to system..."
    udevadm control --reload-rules
    udevadm trigger

    echo ""
    echo "✓ USB serial device access configured!"
    echo ""
    echo "Configured devices:"
    echo "  • ttyUSB0, ttyUSB1: 777 permissions (⚠️  less secure)"
    echo "  • U2D2 Dynamixel interface: dialout group, 660 permissions (✓ secure)"
    echo ""
    echo "To verify, plug in a USB serial device and check:"
    echo "  ls -l /dev/ttyUSB*"
    echo ""
    echo "For better security in the future, consider:"
    echo "  sudo usermod -a -G dialout \$USER"
    echo "  (then logout/login and use the secure udev rules)"
    ;;

  disable)
    echo "=== Removing USB Serial Device Rules ==="
    echo ""
    
    # Remove the udev rule file if it exists
    if [ -f "$UDEV_RULE_FILE" ]; then
      echo "Removing udev rule file..."
      rm "$UDEV_RULE_FILE"
      
      # Reload udev rules
      echo "Reloading udev rules..."
      udevadm control --reload-rules
      
      echo "✓ USB serial device rules removed successfully"
      echo ""
      echo "USB serial devices will now use default system permissions."
      echo "You may need to use sudo or add users to dialout group for access."
    else
      echo "ℹ️  No udev rule file found at $UDEV_RULE_FILE"
      echo "   USB serial device rules are not currently enabled."
    fi
    ;;

  *)
    echo "❌ Invalid option: $1"
    echo ""
    echo "Usage: $0 [enable|disable]"
    echo ""
    echo "Examples:"
    echo "  sudo $0 enable     # Enable automatic USB serial device permissions"  
    echo "  sudo $0 disable    # Remove automatic permissions"
    exit 1
    ;;
esac 