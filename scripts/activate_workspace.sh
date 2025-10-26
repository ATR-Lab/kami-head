#!/bin/bash
# activate_workspace.sh - Cross-platform workspace activation for Coffee Buddy
#
# DESCRIPTION:
#   This script sets up the complete development environment for the Coffee Buddy project.
#   It supports both Ubuntu (venv + native ROS2) and macOS (mamba + RoboStack).
#   The script handles platform detection and activates the appropriate environment type.
#
#   The script handles all path resolution automatically and can be run from any directory.
#   It uses intelligent workspace detection to source the appropriate ROS2 environment
#   and workspace overlay files.
#
# USAGE:
#   source activate_workspace.sh [env_name] [ros_distro]
#
# PARAMETERS:
#   env_name     Optional. Name of the environment to activate.
#                Default: ros_env (for both venv and conda environments)
#   ros_distro   Optional. ROS2 distribution to use.
#                Default: humble
#
# EXAMPLES:
#   # Use defaults (ros_env, humble)
#   source activate_workspace.sh
#
#   # Use specific environment name
#   source activate_workspace.sh my_ros_env
#
#   # Use specific ROS distribution
#   source activate_workspace.sh ros_env humble
#
#   # Run from any directory
#   cd coffee_ws
#   source ../scripts/activate_workspace.sh
#
# REQUIREMENTS:
#   Ubuntu: Virtual environment must exist, ROS2 installed via apt
#   macOS: Conda environment must exist, RoboStack installed via mamba
#   All platforms: Workspace must be built (install/setup.bash exists)
#
# WHAT IT DOES:
#   1. Detects platform (Ubuntu vs macOS)
#   2. Activates appropriate environment (venv vs conda)
#   3. Sources ROS2 base installation
#   4. Sources the workspace overlay (if built)
#   5. Provides confirmation of successful setup
#
# NOTE:
#   This script must be SOURCED, not executed, to modify the current shell environment.

# Check if script is being sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: This script must be sourced, not executed."
    echo "Usage: source $(basename ${BASH_SOURCE[0]}) [env_name] [ros_distro]"
    exit 1
fi

# Default values
DEFAULT_ENV_NAME="ros_env"
DEFAULT_ROS_DISTRO="humble"
ENV_NAME="${1:-$DEFAULT_ENV_NAME}"
ROS_DISTRO="${2:-$DEFAULT_ROS_DISTRO}"

# Find the repository root
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# Define paths
SCRIPTS_PATH="$REPO_ROOT/scripts"
COFFEE_WS_PATH="$REPO_ROOT/coffee_ws"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Platform detection
detect_platform() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        PLATFORM="macos"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if command -v apt &> /dev/null; then
            PLATFORM="ubuntu"
        else
            log_error "Linux platform detected but apt not found. Only Ubuntu/Debian supported."
            return 1
        fi
    else
        log_error "Unsupported platform: $OSTYPE"
        log_error "Supported platforms: macOS, Ubuntu/Debian"
        return 1
    fi
}

# Check environment exists for Ubuntu (venv)
check_ubuntu_env() {
    local venv_path="$REPO_ROOT/$ENV_NAME"
    if [ ! -d "$venv_path" ]; then
        log_error "Virtual environment not found at $venv_path"
        echo ""
        echo "It looks like you haven't run the initial setup yet."
        echo "Please run the setup script first:"
        echo "  ./scripts/setup_workspace.sh --env-name $ENV_NAME --ros-distro $ROS_DISTRO"
        echo ""
        echo "This will install system dependencies and create the virtual environment."
        return 1
    fi
}

# Check environment exists for macOS (conda)
check_macos_env() {
    # First check if already in the target environment
    if [[ -n "$CONDA_DEFAULT_ENV" && "$CONDA_DEFAULT_ENV" == "$ENV_NAME" ]]; then
        log_info "Already in target conda environment '$ENV_NAME'"
        return 0
    fi
    
    if ! command -v mamba &> /dev/null && ! command -v conda &> /dev/null; then
        log_error "Neither mamba nor conda found!"
        echo ""
        echo "Please install miniforge first:"
        echo "  curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
        echo "  bash Miniforge3-MacOSX-$(uname -m).sh"
        echo ""
        return 1
    fi
    
    local manager="mamba"
    if ! command -v mamba &> /dev/null; then
        manager="conda"
    fi
    
    if ! $manager env list | grep -q "\s$ENV_NAME\s"; then
        log_error "Conda environment '$ENV_NAME' not found!"
        echo ""
        echo "It looks like you haven't run the initial setup yet."
        echo "Please run the setup script first:"
        echo "  ./scripts/setup_workspace.sh --env-name $ENV_NAME --ros-distro $ROS_DISTRO"
        echo ""
        echo "This will create the mamba environment with RoboStack."
        return 1
    fi
}

# Check if already in conda environment
check_conda_env_status() {
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        if [[ "$CONDA_DEFAULT_ENV" == "$ENV_NAME" ]]; then
            log_info "Already in target environment '$ENV_NAME'"
            return 0  # Already in correct environment
        else
            log_warning "Currently in environment '$CONDA_DEFAULT_ENV', will switch to '$ENV_NAME'"
            return 1  # Need to switch
        fi
    fi
    return 1  # Not in any conda environment
}

# Activate Ubuntu environment
activate_ubuntu() {
    local venv_path="$REPO_ROOT/$ENV_NAME"
    
    log_info "[1/4] Activating virtual environment..."
    source "$venv_path/bin/activate"
    
    log_info "[2/4] Configuring PYTHONPATH for ROS2 entry points..."
    VENV_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
    export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"
    
    log_info "[3/4] Sourcing ROS2 environment..."
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        log_error "ROS2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
        return 1
    fi
    
    log_info "[4/4] Sourcing workspace overlay..."
    if [ -f "$COFFEE_WS_PATH/install/setup.bash" ]; then
        cd "$COFFEE_WS_PATH"
        cd install
        source setup.bash
        cd ..
    else
        log_warning "Workspace not built yet. Run 'colcon build' in $COFFEE_WS_PATH"
    fi
}

# Activate macOS environment
activate_macos() {
    local manager="mamba"
    if ! command -v mamba &> /dev/null; then
        manager="conda"
    fi
    
    # Check if already in correct environment
    if check_conda_env_status; then
        log_info "[1/3] Already in conda environment '$ENV_NAME', skipping activation..."
    else
        log_info "[1/3] Activating conda environment '$ENV_NAME'..."
        eval "$($manager shell hook)"
        $manager activate "$ENV_NAME"
    fi
    
    log_info "[2/3] ROS2 environment configured via RoboStack..."
    # RoboStack automatically sets up ROS2 environment when activating
    # Verify ROS2 is available
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 not available in environment. Check RoboStack installation."
        return 1
    fi
    
    log_info "[3/3] Sourcing workspace overlay..."
    if [ -f "$COFFEE_WS_PATH/install/setup.bash" ]; then
        cd "$COFFEE_WS_PATH"
        cd install
        source setup.bash
        cd ..
        
        # Set macOS-specific environment variables for Qt applications
        # Set macOS Qt environment variables for GUI applications
        if [[ "$PLATFORM" == "macos" ]]; then
            export QT_QPA_PLATFORM=cocoa
            export QT_MAC_WANTS_LAYER=1
            log_info "macOS Qt environment variables set"
        fi
    else
        log_warning "Workspace not built yet. Run 'colcon build' in $COFFEE_WS_PATH"
    fi
}

# Verify setup
verify_setup() {
    log_info "Verifying environment setup..."
    
    # Check ROS2 command availability
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 command not available"
        return 1
    fi
    
    # Check Python environment
    local python_path=$(which python)
    log_info "Python: $python_path"
    
    # Platform-specific verification
    if [[ "$PLATFORM" == "ubuntu" ]]; then
        # Verify virtual environment packages are accessible
        if ! /usr/bin/python3 -c "import sys; sys.path.insert(0, '$VENV_SITE_PACKAGES')" 2>/dev/null; then
            log_warning "PYTHONPATH configuration may have issues"
        fi
    fi
    
    # Check if workspace packages are available
    if ros2 pkg list | grep -q "coffee_voice_agent_ui"; then
        log_success "Coffee Voice Agent UI package found"
    else
        log_warning "Coffee Voice Agent UI package not found. Workspace may need to be built."
    fi
    
    return 0
}

# Main execution
main() {
    echo "======================================"
    echo "Coffee Buddy Workspace Activation"
    echo "======================================"
    echo ""
    
    # Detect platform
    if ! detect_platform; then
        return 1
    fi
    
    log_info "Platform detected: $PLATFORM"
    log_info "Environment name: $ENV_NAME"
    log_info "ROS2 distribution: $ROS_DISTRO"
    log_info "Repository root: $REPO_ROOT"
    echo ""
    
    # Platform-specific environment check and activation
    case $PLATFORM in
        ubuntu)
            if ! check_ubuntu_env; then
                return 1
            fi
            if ! activate_ubuntu; then
                return 1
            fi
            ;;
        macos)
            if ! check_macos_env; then
                return 1
            fi
            if ! activate_macos; then
                return 1
            fi
            ;;
        *)
            log_error "Unsupported platform: $PLATFORM"
            return 1
            ;;
    esac
    
    echo ""
    
    # Verify setup
    if verify_setup; then
        log_success "Environment verification passed!"
    else
        log_warning "Environment verification had issues, but activation completed"
    fi
    
    echo ""
    log_success "Workspace activation complete!"
    echo ""
    echo "Environment Details:"
    echo "  Platform: $PLATFORM"
    echo "  Environment: $ENV_NAME"
    echo "  ROS2 Distribution: $ROS_DISTRO"
    echo "  Python: $(which python)"
    echo "  Workspace: $COFFEE_WS_PATH"
    echo ""
    echo "Ready for development! You can now:"
    echo "  - Build packages: cd coffee_ws && colcon build"
    echo "  - Run nodes: ros2 run coffee_voice_agent_ui voice_agent_monitor_simple"
    echo "  - Run full monitor: ros2 run coffee_voice_agent_ui voice_agent_monitor"
    echo "  - Launch systems: ros2 launch coffee_voice_agent_ui monitor.launch.py"
    echo ""
}

# Execute main function
main