#!/bin/bash
# ros_source.sh - Intelligent ROS2 environment sourcing utility
#
# DESCRIPTION:
#   This script provides an intelligent way to source ROS2 environments without
#   hardcoding paths or ROS versions. It automatically detects the installed ROS
#   distribution and finds the nearest workspace by searching upward from the
#   current directory.
#
#   This eliminates the need to manually source multiple setup files and makes
#   the environment setup portable across different ROS installations and
#   workspace structures.
#
# FEATURES:
#   - Auto-detects ROS version (jazzy, humble, iron, etc.)
#   - Searches upward for workspace (no need to be in specific directory)
#   - Handles missing ROS installations gracefully
#   - Provides clear feedback about what's being sourced
#   - Returns appropriate exit codes for error handling
#
# USAGE:
#   # Load the function (usually done by other scripts)
#   source ros_source.sh
#   
#   # Call the function to set up environment
#   ros-source
#
# EXAMPLES:
#   # Manual usage
#   cd ~/my_project/coffee_ws/src/my_package
#   source ../../../scripts/ros_source.sh
#   ros-source  # Will find and source coffee_ws workspace
#
#   # Used by other scripts
#   source "$SCRIPTS_PATH/ros_source.sh"
#   ros-source
#
# RETURN CODES:
#   0 - Success (ROS and workspace sourced)
#   1 - Error (ROS not found or workspace not found)
#
# WHAT IT SOURCES:
#   1. Global ROS setup: /opt/ros/{version}/setup.bash
#   2. Workspace setup: {workspace}/install/setup.bash (if found)
#
# DIRECTORY SEARCH:
#   Searches upward from current directory until it finds install/setup.bash
#   Example: /home/user/project/workspace/src/package
#            ↑ searches here, then parent, then parent...
#            ↑ finds /home/user/project/workspace/install/setup.bash

# Define the intelligent ROS sourcing function
ros-source() {
    echo "🤖 Starting intelligent ROS environment setup..."
    
    # Step 1: Auto-detect ROS version from /opt/ros/
    echo ""
    echo "[1/2] Detecting ROS installation..."
    local ros_version=""
    
    # Search through all directories in /opt/ros/ to find installed versions
    for dir in /opt/ros/*/; do
        if [[ -f "$dir/setup.bash" ]]; then
            ros_version=$(basename "$dir")
            echo "  ✓ Found ROS $ros_version at $dir"
            break
        fi
    done

    # Check if any ROS installation was found
    if [[ -z "$ros_version" ]]; then
        echo "  ❌ No ROS installation found in /opt/ros/"
        echo "     Please install ROS2 first: https://docs.ros.org/en/jazzy/Installation.html"
        return 1
    fi

    # Step 2: Source the global ROS setup file
    local ros_setup="/opt/ros/$ros_version/setup.bash"
    if [[ -f "$ros_setup" ]]; then
        echo "  📦 Sourcing global ROS setup: $ros_setup"
        source "$ros_setup"
    else
        echo "  ❌ Could not find ROS setup file: $ros_setup"
        return 1
    fi

    # Step 3: Search upward for a workspace with install/setup.bash
    echo ""
    echo "[2/2] Searching for ROS workspace..."
    local dir="$PWD"
    local search_depth=0
    local max_depth=10  # Prevent infinite search
    
    # Search upward through parent directories
    while [[ "$dir" != "/" && $search_depth -lt $max_depth ]]; do
        echo "  🔍 Checking: $dir"
        
        # Check if this directory contains a built workspace
        if [[ -f "$dir/install/setup.bash" ]]; then
            echo "  ✓ Found workspace: $dir"
            echo "  🏗️  Sourcing workspace setup: $dir/install/setup.bash"
            source "$dir/install/setup.bash"
            
            echo ""
            echo "🎉 ROS environment ready!"
            echo "   ROS Version: $ros_version"
            echo "   Workspace: $dir"
            echo "   You can now run ROS2 commands and launch nodes."
            return 0
        fi
        
        # Move to parent directory and increment search depth
        dir=$(dirname "$dir")
        ((search_depth++))
    done

    # No workspace found - this is not necessarily an error
    echo "  ℹ️  No ROS workspace found in parent directories"
    echo "     (This is OK if you're not working in a workspace)"
    
    echo ""
    echo "🎯 ROS base environment ready!"
    echo "   ROS Version: $ros_version"
    echo "   No workspace overlay (using base ROS packages only)"
    echo ""
    echo "💡 To create a workspace:"
    echo "   mkdir -p ~/my_workspace/src"
    echo "   cd ~/my_workspace"
    echo "   colcon build"
    return 1
}
