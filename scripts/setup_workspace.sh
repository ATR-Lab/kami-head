#!/bin/bash
# setup_workspace.sh - Cross-platform setup for Coffee Buddy development environment
#
# DESCRIPTION:
#   This script performs the initial setup required for Coffee Buddy development.
#   It supports both Ubuntu (native ROS2) and macOS (RoboStack with mamba).
#   The script installs dependencies, creates the appropriate environment,
#   and builds all ROS2 packages. Run this script once before using the
#   activate_workspace.sh script for daily development.
#
# USAGE:
#   ./scripts/setup_workspace.sh [options]
#
# OPTIONS:
#   -e, --env-name NAME     Name of the environment to create (default: ros_env)
#   -d, --ros-distro DISTRO ROS2 distribution (default: humble)
#   -h, --help              Show this help message
#
# EXAMPLES:
#   # Use defaults (ros_env, jazzy)
#   ./scripts/setup_workspace.sh
#
#   # Custom environment name
#   ./scripts/setup_workspace.sh --env-name my_ros_env
#
#   # Different ROS2 distro
#   ./scripts/setup_workspace.sh --ros-distro humble
#
# PLATFORMS SUPPORTED:
#   - Ubuntu/Debian: Native ROS2 installation with apt + venv
#   - macOS: RoboStack with mamba/conda environment
#
# REQUIREMENTS:
#   Ubuntu: sudo privileges, apt package manager
#   macOS: miniforge/mamba installed (script will guide if missing)
#
# WHAT IT DOES:
#   1. Detects platform (Ubuntu vs macOS)
#   2. Installs platform-specific dependencies
#   3. Creates appropriate environment (venv vs conda)
#   4. Installs ROS2 and development tools
#   5. Builds all ROS2 packages in workspace
#   6. Validates the setup

set -e  # Exit on any error

# Default values
DEFAULT_ENV_NAME="ros_env"
DEFAULT_ROS_DISTRO="jazzy"
ENV_NAME=""
ROS_DISTRO=""

# Find the repository root
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
REQUIREMENTS_FILE="$REPO_ROOT/requirements.txt"

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

# Help function
show_help() {
    cat << EOF
Coffee Buddy Development Environment Setup

USAGE:
    $0 [options]

OPTIONS:
    -e, --env-name NAME     Name of the environment to create (default: $DEFAULT_ENV_NAME)
    -d, --ros-distro DISTRO ROS2 distribution (default: $DEFAULT_ROS_DISTRO)
    -h, --help              Show this help message

EXAMPLES:
    # Use defaults
    $0

    # Custom environment name  
    $0 --env-name my_ros_env

    # Different ROS2 distribution
    $0 --ros-distro humble

SUPPORTED PLATFORMS:
    - Ubuntu/Debian: Native ROS2 with apt + Python venv
    - macOS: RoboStack with mamba/conda environment

SUPPORTED ROS2 DISTRIBUTIONS:
    - jazzy (default)
    - humble
    - kilted

NOTES:
    - If already in a mamba environment, the script will handle switching automatically
    - RoboStack channels are configured automatically for the selected ROS2 distribution
    - For humble: ./scripts/setup_workspace.sh --ros-distro humble

For more information, see: https://github.com/your-repo/coffee-buddy
EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -e|--env-name)
                ENV_NAME="$2"
                shift 2
                ;;
            -d|--ros-distro)
                ROS_DISTRO="$2"
                shift 2
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # Set defaults if not specified
    ENV_NAME="${ENV_NAME:-$DEFAULT_ENV_NAME}"
    ROS_DISTRO="${ROS_DISTRO:-$DEFAULT_ROS_DISTRO}"
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
            exit 1
        fi
    else
        log_error "Unsupported platform: $OSTYPE"
        log_error "Supported platforms: macOS, Ubuntu/Debian"
        exit 1
    fi
}

# Check if mamba/conda is available (for macOS)
check_mamba() {
    if ! command -v mamba &> /dev/null; then
        if ! command -v conda &> /dev/null; then
            log_error "Neither mamba nor conda found!"
            log_error "Please install miniforge first:"
            log_error "  curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
            log_error "  bash Miniforge3-MacOSX-$(uname -m).sh"
            log_error "Then restart your terminal and run this script again."
            exit 1
        else
            log_warning "conda found but mamba not available. Installing mamba..."
            conda install mamba -c conda-forge -y
        fi
    fi
}

# Validate ROS2 distribution support
validate_ros_distro() {
    local supported_distros=("humble" "jazzy" "kilted")
    local distro_supported=false
    
    for supported in "${supported_distros[@]}"; do
        if [[ "$ROS_DISTRO" == "$supported" ]]; then
            distro_supported=true
            break
        fi
    done
    
    if [[ "$distro_supported" == false ]]; then
        log_error "ROS2 distribution '$ROS_DISTRO' is not supported by RoboStack"
        log_error "Supported distributions: ${supported_distros[*]}"
        log_error "Please choose a supported distribution:"
        log_error "  ./scripts/setup_workspace.sh --ros-distro humble"
        exit 1
    fi
}

# Check if already in conda environment
check_conda_env_active() {
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        log_warning "Already in conda environment: $CONDA_DEFAULT_ENV"
        if [[ "$CONDA_DEFAULT_ENV" == "$ENV_NAME" ]]; then
            log_info "Already in target environment '$ENV_NAME', continuing setup..."
            return 0
        else
            log_warning "In different environment. Will deactivate and switch to '$ENV_NAME'"
            conda deactivate 2>/dev/null || true
            mamba deactivate 2>/dev/null || true
        fi
    fi
    return 1
}

# Ubuntu setup function
setup_ubuntu() {
    log_info "Setting up Ubuntu environment..."
    
    # Step 1: Install system dependencies
    log_info "[1/5] Installing system dependencies..."
    sudo apt update
    sudo apt install -y \
        portaudio19-dev \
        python3-dev \
        python3-venv \
        python3-pip \
        build-essential \
        libasound2-dev \
        pkg-config \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common

    # Step 2: Install ROS2 if not already installed
    log_info "[2/5] Setting up ROS2 $ROS_DISTRO..."
    if ! command -v ros2 &> /dev/null; then
        log_info "Installing ROS2 $ROS_DISTRO..."
        
        # Add ROS2 apt repository
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        # Install ROS2
        sudo apt update
        sudo apt install -y ros-$ROS_DISTRO-desktop python3-colcon-common-extensions
        
        log_success "ROS2 $ROS_DISTRO installed"
    else
        log_success "ROS2 already installed"
    fi

    # Step 3: Create Python virtual environment
    log_info "[3/5] Setting up Python virtual environment..."
    VENV_PATH="$REPO_ROOT/$ENV_NAME"
    
    if [ -d "$VENV_PATH" ]; then
        log_warning "Virtual environment already exists at: $VENV_PATH"
    else
        log_info "Creating virtual environment at: $VENV_PATH"
        python3 -m venv "$VENV_PATH"
        log_success "Virtual environment created"
    fi

    # Step 4: Install Python packages
    log_info "[4/5] Installing Python packages..."
    source "$VENV_PATH/bin/activate"
    pip install --upgrade pip
    
    if [ -f "$REQUIREMENTS_FILE" ]; then
        pip install -r "$REQUIREMENTS_FILE"
        log_success "Python packages installed from requirements.txt"
    else
        log_warning "requirements.txt not found, skipping Python package installation"
    fi

    # Step 5: Build ROS2 packages
    log_info "[5/5] Building ROS2 packages..."
    cd "$REPO_ROOT/coffee_ws"
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
    log_success "ROS2 packages built successfully"
}

# macOS setup function  
setup_macos() {
    log_info "Setting up macOS environment with RoboStack..."
    
    # Step 1: Check mamba installation
    log_info "[1/5] Checking mamba installation..."
    check_mamba
    log_success "Mamba is available"

    # Step 2: Create conda environment
    log_info "[2/5] Setting up conda environment '$ENV_NAME'..."
    
    # Check if already in target environment
    local already_in_target_env=false
    if check_conda_env_active; then
        already_in_target_env=true
    fi
    
    # Check if environment exists
    if mamba env list | grep -q "^$ENV_NAME "; then
        log_warning "Environment '$ENV_NAME' already exists"
        if [[ "$already_in_target_env" == false ]]; then
            log_info "Activating existing environment..."
            eval "$(mamba shell hook)"
            mamba activate "$ENV_NAME"
        fi
    else
        log_info "Creating new environment '$ENV_NAME'..."
        mamba create -n "$ENV_NAME" -y
        eval "$(mamba shell hook)"
        mamba activate "$ENV_NAME"
    fi
    
    # Configure channels for the environment
    conda config --env --add channels conda-forge
    conda config --env --remove channels defaults 2>/dev/null || true
    conda config --env --add channels robostack-$ROS_DISTRO

    # Step 3: Install ROS2 and development tools
    log_info "[3/5] Installing ROS2 $ROS_DISTRO and development tools..."
    mamba install -y \
        ros-$ROS_DISTRO-desktop \
        compilers \
        cmake \
        pkg-config \
        make \
        ninja \
        colcon-common-extensions \
        rosdep \
        python \
        pip

    log_success "ROS2 and development tools installed"

    # Step 4: Install additional Python packages
    log_info "[4/5] Installing additional Python packages..."
    if [ -f "$REQUIREMENTS_FILE" ]; then
        # Try to install via conda first, fallback to pip
        while IFS= read -r package || [[ -n "$package" ]]; do
            # Skip empty lines and comments
            [[ -z "$package" || "$package" =~ ^#.*$ ]] && continue
            
            package_name=$(echo "$package" | cut -d'=' -f1 | cut -d'>' -f1 | cut -d'<' -f1)
            
            # Try conda-forge first
            if mamba install -c conda-forge "$package_name" -y 2>/dev/null; then
                log_info "Installed $package_name via conda"
            else
                # Fallback to pip
                log_info "Installing $package_name via pip..."
                pip install "$package"
            fi
        done < "$REQUIREMENTS_FILE"
        log_success "Additional Python packages installed"
    else
        log_warning "requirements.txt not found, skipping additional Python packages"
    fi

    # Deactivate and reactivate to ensure proper ROS setup
    mamba deactivate
    mamba activate "$ENV_NAME"

    # Step 5: Build ROS2 packages
    log_info "[5/5] Building ROS2 packages..."
    cd "$REPO_ROOT/coffee_ws"
    colcon build --symlink-install
    log_success "ROS2 packages built successfully"
}

# Validation function
validate_setup() {
    log_info "Validating setup..."
    
    cd "$REPO_ROOT/coffee_ws"
    
    if [[ "$PLATFORM" == "macos" ]]; then
        # For macOS, activate conda environment
        eval "$(mamba shell hook)"
        mamba activate "$ENV_NAME"
    else
        # For Ubuntu, source ROS and activate venv
        source /opt/ros/$ROS_DISTRO/setup.bash
        source "$REPO_ROOT/$ENV_NAME/bin/activate"
    fi
    
    # Source workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        log_success "Workspace sourced successfully"
    else
        log_error "Workspace build failed - install/setup.bash not found"
        return 1
    fi
    
    # Test ROS2 functionality
    if command -v ros2 &> /dev/null; then
        log_success "ROS2 command available"
        
        # Test if our packages are available
        if ros2 pkg list | grep -q "coffee_voice_agent_ui"; then
            log_success "Coffee Voice Agent UI package found"
        else
            log_warning "Coffee Voice Agent UI package not found in ROS2 package list"
        fi
    else
        log_error "ROS2 command not available"
        return 1
    fi
    
    return 0
}

# Main execution
main() {
    echo "======================================"
    echo "Coffee Buddy Development Setup"
    echo "======================================"
    echo ""
    
    # Parse arguments
    parse_args "$@"
    
    # Detect platform
    detect_platform
    
    # Validate ROS2 distribution for macOS (RoboStack)
    if [[ "$PLATFORM" == "macos" ]]; then
        validate_ros_distro
    fi
    
    log_info "Platform detected: $PLATFORM"
    log_info "Environment name: $ENV_NAME"
    log_info "ROS2 distribution: $ROS_DISTRO"
    log_info "Repository root: $REPO_ROOT"
    
    # Show current environment status for macOS
    if [[ "$PLATFORM" == "macos" && -n "$CONDA_DEFAULT_ENV" ]]; then
        log_info "Current conda environment: $CONDA_DEFAULT_ENV"
    fi
    echo ""
    
    # Platform-specific setup
    case $PLATFORM in
        ubuntu)
            setup_ubuntu
            ;;
        macos)
            setup_macos
            ;;
        *)
            log_error "Unsupported platform: $PLATFORM"
            exit 1
            ;;
    esac
    
    echo ""
    
    # Validate setup
    if validate_setup; then
        log_success "Setup validation passed!"
    else
        log_error "Setup validation failed!"
        exit 1
    fi
    
    echo ""
    echo "🎉 Setup complete!"
    echo ""
    echo "Next steps:"
    if [[ "$PLATFORM" == "macos" ]]; then
        echo "  1. To activate your development environment:"
        echo "     mamba activate $ENV_NAME"
        echo "     cd $REPO_ROOT/coffee_ws"
        echo "     source install/setup.bash"
    else
        echo "  1. To activate your development environment:"
        echo "     source $REPO_ROOT/scripts/activate_workspace.sh $ENV_NAME"
    fi
    echo ""
    echo "  2. Test your setup:"
    echo "     ros2 run coffee_voice_agent_ui voice_agent_monitor"
    echo ""
    echo "  3. If you encounter issues:"
    echo "     - Check the build logs in coffee_ws/log/"
    echo "     - Re-run this setup script"
    echo "     - Check the troubleshooting guide in README.md"
    echo ""
}

# Run main function with all arguments
main "$@" 