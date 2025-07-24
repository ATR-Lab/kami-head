#!/bin/bash
# activate_workspace.sh - Activate Python virtual environment and ROS2 workspace
#
# DESCRIPTION:
#   This script sets up the complete development environment for the Coffee Buddy project.
#   It activates the specified Python virtual environment, sources the ROS2 installation,
#   and configures the workspace overlay. This provides a one-command setup for development.
#
#   The script handles all path resolution automatically and can be run from any directory.
#   It uses intelligent workspace detection to source the appropriate ROS2 environment
#   and workspace overlay files.
#
# USAGE:
#   source activate_workspace.sh [venv_name]
#
# PARAMETERS:
#   venv_name    Optional. Name of the virtual environment to activate.
#                Default: coffee_buddy_venv
#
# EXAMPLES:
#   # Use default virtual environment
#   source activate_workspace.sh
#
#   # Use specific virtual environment  
#   source activate_workspace.sh my_custom_venv
#
#   # Run from any directory
#   cd coffee_ws
#   source ../scripts/activate_workspace.sh
#
# REQUIREMENTS:
#   - Virtual environment must exist in project root directory
#   - ROS2 must be installed (/opt/ros/*)
#   - Workspace must be built (install/setup.bash exists)
#
# WHAT IT DOES:
#   1. Activates the specified Python virtual environment
#   2. Sources ROS2 base installation 
#   3. Sources the workspace overlay (if built)
#   4. Provides confirmation of successful setup
#
# NOTE:
#   This script must be SOURCED, not executed, to modify the current shell environment.

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
    echo ""
    echo "It looks like you haven't run the initial setup yet."
    echo "Please run the setup script first:"
    echo "  ./scripts/setup_workspace.sh"
    echo ""
    echo "This will install system dependencies and create the virtual environment."
    return 1
fi

echo "Activating workspace environment..."
echo "Virtual environment: $VENV_NAME"
echo "Workspace: $COFFEE_WS_PATH"

# Step 1: Activate the virtual environment
echo "  [1/3] Activating virtual environment..."
source "$VENV_PATH/bin/activate"

# Step 2: Change to coffee_ws directory and source ROS environment
echo "  [2/3] Sourcing ROS2 environment..."
pushd "$COFFEE_WS_PATH" > /dev/null
source "$SCRIPTS_PATH/ros_source.sh"

# Step 3: Run ros-source function to set up ROS2 and workspace
echo "  [3/3] Configuring workspace overlay..."
ros-source

# Return to original directory
popd > /dev/null

# Quick verification that critical packages are available
echo "  [Verification] Checking critical dependencies..."
if ! python -c "import pyaudio" 2>/dev/null; then
    echo ""
    echo "⚠️  Warning: PyAudio not available in virtual environment"
    echo "   This is required for TTS functionality."
    echo "   Please run the setup script to install dependencies:"
    echo "   ./scripts/setup_workspace.sh"
    echo ""
    return 1
fi

echo ""
echo "✓ Workspace activation complete!"
echo "  Virtual environment: $VENV_NAME"
echo "  Python: $(which python)"
echo "  PyAudio: Available ✓"
echo "  ROS2 workspace ready for development"
echo ""