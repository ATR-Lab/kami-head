#!/bin/bash
# configure_cyclonedds.sh - Configure CycloneDDS middleware for ROS2
#
# DESCRIPTION:
#   This script configures Eclipse CycloneDDS as the ROS2 middleware implementation.
#   CycloneDDS is a high-performance, open-source DDS implementation that provides
#   better performance and reliability compared to the default FastDDS in many scenarios.
#
#   The script permanently adds environment variables to ~/.bashrc to ensure
#   CycloneDDS is used as the RMW (ROS Middleware) implementation across all
#   ROS2 sessions.
#
# USAGE:
#   ./configure_cyclonedds.sh
#
# WHAT IT CONFIGURES:
#   1. RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#      - Sets CycloneDDS as the ROS2 middleware implementation
#      - Alternative to default FastDDS (rmw_fastrtps_cpp)
#
#   2. CYCLONEDDS_URI with discovery settings:
#      - AllowMulticast=true: Enables multicast for node discovery
#      - ParticipantIndex=auto: Automatic participant indexing
#      - MaxAutoParticipantIndex=32: Supports up to 32 participants
#
# REQUIREMENTS:
#   - CycloneDDS must be installed: sudo apt install ros-$ROS_DISTRO-rmw-cyclonedx-cpp
#   - Write permissions to ~/.bashrc
#
# PERMANENT CHANGES:
#   ⚠️  WARNING: This script modifies ~/.bashrc permanently
#   The environment variables will persist across all future shell sessions
#
# WHY CYCLONEDDS:
#   - Better performance for large message throughput
#   - More reliable discovery in complex network environments  
#   - Lower CPU usage compared to FastDDS in many scenarios
#   - Better multicast support
#
# USAGE:
#   chmod +x scripts/configure_cyclonedds.sh
#   ./scripts/configure_cyclonedds.sh
#
# TO VERIFY CONFIGURATION:
#   echo $RMW_IMPLEMENTATION  # Should show: rmw_cyclonedds_cpp
#   ros2 doctor --report       # Should show CycloneDDS as middleware

echo "=== CycloneDDS Configuration Script ==="
echo "This script will configure Eclipse CycloneDDS as your ROS2 middleware."
echo ""

# Check if CycloneDDS is installed
if ! dpkg -l | grep -q "rmw-cyclonedx-cpp"; then
    echo "⚠️  Warning: CycloneDDS may not be installed."
    echo "   Install with: sudo apt install ros-\$ROS_DISTRO-rmw-cyclonedx-cpp"
    echo "   Continuing anyway..."
    echo ""
fi

echo "Configuring ROS2 middleware settings..."

# Configure RMW Implementation (CycloneDDS)
echo ""
echo "[1/2] Setting RMW_IMPLEMENTATION to CycloneDDS..."
if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    echo "✓ Added RMW_IMPLEMENTATION=rmw_cyclonedds_cpp to ~/.bashrc"
else
    echo "✓ RMW_IMPLEMENTATION already configured in ~/.bashrc"
fi

# Configure CycloneDDS Discovery Settings
echo ""
echo "[2/2] Configuring CycloneDDS discovery settings..."
if ! grep -q "export CYCLONEDDS_URI=" ~/.bashrc; then
    # Add CycloneDDS configuration for optimal discovery
    echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'" >> ~/.bashrc
    echo "✓ Added CycloneDDS discovery configuration to ~/.bashrc"
    echo "  - Multicast discovery: enabled"
    echo "  - Participant indexing: automatic"  
    echo "  - Max participants: 32"
else
    echo "✓ CYCLONEDDS_URI already configured in ~/.bashrc"
fi

# Apply changes to current session
echo ""
echo "Applying configuration to current session..."
source ~/.bashrc
echo "✓ Configuration applied"

echo ""
echo "=== Configuration Complete ==="
echo "CycloneDDS is now configured as your ROS2 middleware."
echo ""
echo "To verify the configuration:"
echo "  echo \$RMW_IMPLEMENTATION    # Should show: rmw_cyclonedds_cpp"
echo "  ros2 doctor --report         # Should show CycloneDDS info"
echo ""
echo "⚠️  Note: Changes are permanent in ~/.bashrc"
echo "   To revert, manually remove the added lines from ~/.bashrc"
echo ""
