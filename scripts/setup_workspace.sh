#!/bin/bash
# setup_workspace.sh - One-time setup for Coffee Buddy development environment
#
# DESCRIPTION:
#   This script performs the initial setup required for Coffee Buddy development.
#   It installs system dependencies, creates the Python virtual environment,
#   and installs all required Python packages. Run this script once before
#   using the activate_workspace.sh script for daily development.
#
#   Currently supports Ubuntu/Debian systems. Future versions will support macOS.
#
# USAGE:
#   ./scripts/setup_workspace.sh [venv_name]
#
# PARAMETERS:
#   venv_name    Optional. Name of the virtual environment to create.
#                Default: coffee_buddy_venv
#
# EXAMPLES:
#   # Use default virtual environment
#   ./scripts/setup_workspace.sh
#
#   # Use specific virtual environment  
#   ./scripts/setup_workspace.sh my_custom_venv
#
# REQUIREMENTS:
#   - Ubuntu/Debian system (apt package manager)
#   - sudo privileges (for system package installation)
#   - Internet connection
#
# WHAT IT DOES:
#   1. Installs system dependencies for audio processing (PyAudio)
#   2. Creates Python virtual environment if it doesn't exist
#   3. Installs Python packages from requirements.txt
#   4. Calls activate_workspace.sh to verify setup
#
# NOTE:
#   This script requires sudo privileges for system package installation.

# TODO: Add macOS support using homebrew
# TODO: Add Windows support using chocolatey or vcpkg
# TODO: Add automatic platform detection

set -e  # Exit on any error

# Default venv name
DEFAULT_VENV_NAME="coffee_buddy_venv"
VENV_NAME="${1:-$DEFAULT_VENV_NAME}"

# Find the repository root
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# Define paths
VENV_PATH="$REPO_ROOT/$VENV_NAME"
REQUIREMENTS_FILE="$REPO_ROOT/requirements.txt"

echo "Setting up Coffee Buddy development environment..."
echo "Virtual environment: $VENV_NAME"
echo "Repository root: $REPO_ROOT"
echo ""

# Step 1: Install system dependencies for Ubuntu
echo "[1/4] Installing system dependencies..."

# TODO: Add platform detection here
# if [[ "$OSTYPE" == "darwin"* ]]; then
#     # macOS
#     echo "Installing macOS dependencies with homebrew..."
#     brew install portaudio python3
# elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
#     # Linux (Ubuntu/Debian)

# Check if we're on Ubuntu/Debian
if ! command -v apt &> /dev/null; then
    echo "Error: This script currently only supports Ubuntu/Debian systems (apt package manager)"
    echo "TODO: Add support for other Linux distributions and macOS"
    exit 1
fi

# Install system dependencies required for PyAudio
echo "Installing Ubuntu system dependencies for audio processing..."
sudo apt update
sudo apt install -y \
    portaudio19-dev \
    python3-dev \
    python3-venv \
    python3-pip \
    build-essential \
    libasound2-dev \
    pkg-config

echo "✓ System dependencies installed"

# TODO: Add elif for other platforms here
# else
#     echo "Error: Unsupported platform: $OSTYPE"
#     echo "TODO: Add support for this platform"
#     exit 1
# fi

# Step 2: Create virtual environment if it doesn't exist
echo ""
echo "[2/4] Setting up Python virtual environment..."

if [ -d "$VENV_PATH" ]; then
    echo "Virtual environment already exists at: $VENV_PATH"
else
    echo "Creating virtual environment at: $VENV_PATH"
    python3 -m venv "$VENV_PATH"
    echo "✓ Virtual environment created"
fi

# Step 3: Activate virtual environment and install packages
echo ""
echo "[3/4] Installing Python packages..."

if [ ! -f "$REQUIREMENTS_FILE" ]; then
    echo "Error: requirements.txt not found at: $REQUIREMENTS_FILE"
    exit 1
fi

# Activate virtual environment
source "$VENV_PATH/bin/activate"

# Upgrade pip to latest version
echo "Upgrading pip..."
pip install --upgrade pip

# Install packages from requirements.txt
echo "Installing packages from requirements.txt..."
pip install -r "$REQUIREMENTS_FILE"

echo "✓ Python packages installed"

# Step 4: Test the setup by activating the workspace
echo ""
echo "[4/4] Testing workspace activation..."

# Source the activation script to verify everything works
source "$REPO_ROOT/scripts/activate_workspace.sh" "$VENV_NAME"

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "  1. To activate your development environment daily, run:"
echo "     source scripts/activate_workspace.sh"
echo ""
echo "  2. Test your TTS node:"
echo "     ros2 run coffee_voice_service tts_node"
echo ""
echo "  3. If you encounter issues, check the logs or re-run this setup script."
echo ""

# TODO: Add verification steps
# TODO: Test PyAudio import specifically
# TODO: Check ROS2 package build status 