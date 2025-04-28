#!/bin/bash
# setup_env.sh - Script to set up the ROS2 environment with a specified virtual environment
#
# Usage:
#   source setup_env.sh [venv_name]
#
# Example:
#   source setup_env.sh coffee_buddy_venv

# Check if script is being sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: This script must be sourced, not executed."
    echo "Usage: source $(basename ${BASH_SOURCE[0]}) [venv_name]"
    exit 1
fi

# Default venv name
DEFAULT_VENV_NAME="coffee_buddy_venv"
VENV_NAME="${1:-$DEFAULT_VENV_NAME}"

# Find the repository root
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# Define paths
VENV_PATH="$REPO_ROOT/$VENV_NAME"
SCRIPTS_PATH="$REPO_ROOT/scripts"
COFFEE_WS_PATH="$REPO_ROOT/coffee_ws"

# Check if the venv exists
if [ ! -d "$VENV_PATH" ]; then
    echo "Error: Virtual environment not found at $VENV_PATH"
    return 1
fi

echo "Setting up environment with virtual environment: $VENV_NAME"

# Step 1: Activate the virtual environment
echo "Activating virtual environment..."
source "$VENV_PATH/bin/activate"

# Step 2: Enable ROS2 with the virtual environment
echo "Enabling ROS2 with virtual environment..."
source "$SCRIPTS_PATH/ros2_venv.sh" enable "$VENV_PATH"

# Step 3: Change to coffee_ws directory and source ros-source.sh
echo "Changing to coffee workspace and sourcing ROS..."
pushd "$COFFEE_WS_PATH" > /dev/null
source "$SCRIPTS_PATH/ros-source.sh"

# Step 4: Run ros-source function
echo "Running ros-source..."
ros-source

# Return to original directory
popd > /dev/null

echo "Environment setup complete!"