#!/bin/bash
"""
Manual micro-ROS Agent Starter Script

This script provides a simple way to start the micro-ROS agent Docker container
manually with configurable parameters.
"""

# Default values
DEVICE_PATH="/dev/ttyUSB1"
VERBOSITY=6
DOCKER_IMAGE="microros/micro-ros-agent:jazzy"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--device)
            DEVICE_PATH="$2"
            shift 2
            ;;
        -v|--verbosity)
            VERBOSITY="$2"
            shift 2
            ;;
        -i|--image)
            DOCKER_IMAGE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Start micro-ROS agent Docker container for ESP32 ear control"
            echo ""
            echo "Options:"
            echo "  -d, --device PATH      Serial device path (default: /dev/ttyUSB1)"
            echo "  -v, --verbosity LEVEL  Verbosity level 0-6 (default: 6)"
            echo "  -i, --image IMAGE      Docker image to use (default: microros/micro-ros-agent:jazzy)"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use defaults"
            echo "  $0 -d /dev/ttyUSB0 -v 4              # Custom device and verbosity"
            echo "  $0 --device /dev/ttyACM0             # Different device type"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if device exists
if [ ! -e "$DEVICE_PATH" ]; then
    echo "Error: Device $DEVICE_PATH does not exist"
    echo "Available devices:"
    ls -la /dev/tty* 2>/dev/null | grep -E "(USB|ACM)" || echo "  No USB/ACM devices found"
    exit 1
fi

# Check if docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Check if running as root or if user is in docker group
if [ "$EUID" -ne 0 ] && ! groups | grep -q docker; then
    echo "Warning: You may need to run this script with sudo or add your user to the docker group"
fi

echo "Starting micro-ROS agent with:"
echo "  Device: $DEVICE_PATH"
echo "  Verbosity: $VERBOSITY"
echo "  Docker image: $DOCKER_IMAGE"
echo ""

# Build and execute docker command
set -x  # Show the command being executed
sudo docker run -it --rm \
    -v /dev:/dev \
    --privileged \
    --net=host \
    "$DOCKER_IMAGE" \
    serial \
    --dev "$DEVICE_PATH" \
    -v"$VERBOSITY" 