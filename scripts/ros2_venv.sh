#!/bin/bash
# ros2_venv.sh - Script to switch ROS2 between Python venv and system Python
# 
# Usage:
#   source ros2_venv.sh enable [venv_path]    - Enable Python venv for ROS2
#   source ros2_venv.sh disable               - Revert to system Python

# Default venv path
DEFAULT_VENV_PATH="$HOME/Github/coffee-budy/coffee_buddy_venv"
ROS_DISTRO="jazzy"  # Change this if using a different ROS2 distribution

# Check if script is being sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: This script must be sourced, not executed."
    echo "Usage: source $(basename ${BASH_SOURCE[0]}) enable|disable [venv_path]"
    exit 1
fi

# Save original PYTHONPATH if not already saved
if [ -z "$ORIGINAL_PYTHONPATH" ]; then
    export ORIGINAL_PYTHONPATH="$PYTHONPATH"
fi

# Function to enable venv for ROS2
enable_venv() {
    local venv_path="${1:-$DEFAULT_VENV_PATH}"
    
    if [ ! -d "$venv_path" ]; then
        echo "Error: Virtual environment not found at $venv_path"
        return 1
    fi
    
    # Activate the venv
    source "$venv_path/bin/activate"
    
    # Get venv's site-packages path
    VENV_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
    
    # Source ROS2 setup file
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    
    # Add venv site-packages to PYTHONPATH
    export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"
    
    # Source workspace setup if it exists
    if [ -f "$HOME/Github/coffee-budy/coffee_ws/install/setup.bash" ]; then
        source "$HOME/Github/coffee-budy/coffee_ws/install/setup.bash"
    fi
    
    echo "ROS2 is now using Python virtual environment at $venv_path"
    echo "PYTHONPATH now includes: $VENV_SITE_PACKAGES"
    
    # Set a flag to indicate venv is enabled
    export ROS2_VENV_ENABLED=1
    export ROS2_VENV_PATH="$venv_path"
}

# Function to disable venv and revert to system Python
disable_venv() {
    if [ -n "$ROS2_VENV_ENABLED" ]; then
        # Deactivate venv if active
        if type deactivate > /dev/null 2>&1; then
            deactivate
        fi
        
        # Restore original PYTHONPATH
        export PYTHONPATH="$ORIGINAL_PYTHONPATH"
        
        # Source ROS2 setup file to reset environment
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        
        # Source workspace setup if it exists
        if [ -f "$HOME/Github/coffee-budy/coffee_ws/install/setup.bash" ]; then
            source "$HOME/Github/coffee-budy/coffee_ws/install/setup.bash"
        fi
        
        # Clear the flag
        unset ROS2_VENV_ENABLED
        unset ROS2_VENV_PATH
        
        echo "ROS2 is now using system Python"
    else
        echo "ROS2 is already using system Python"
    fi
}

# Parse command
case "$1" in
    enable)
        enable_venv "${2:-$DEFAULT_VENV_PATH}"
        ;;
    disable)
        disable_venv
        ;;
    *)
        echo "Usage: source $(basename ${BASH_SOURCE[0]}) enable|disable [venv_path]"
        ;;
esac 