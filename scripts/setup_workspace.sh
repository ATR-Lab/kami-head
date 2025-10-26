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
# MACOS-SPECIFIC HANDLING:
#   This script includes special handling for common macOS development environment
#   conflicts, particularly the Homebrew/conda library conflict that affects git
#   operations within conda environments.
#
#   Git Package Conflict Resolution:
#   - Problem: Homebrew's git + conda's libiconv = symbol conflicts
#   - Solution: Pre-download git-based packages using system libraries, 
#             then install from local copies within conda environment
#   - Packages affected: Any using git+https:// URLs in requirements.txt
#   - Fallback: If conflicts persist, install git via conda
#
# REQUIREMENTS:
#   Ubuntu: sudo privileges, apt package manager
#   macOS: internet connection (script installs miniforge automatically if needed)
#         Homebrew (optional - required only for voice functionality via PyAudio)
#
# WHAT IT DOES:
#   1. Detects platform (Ubuntu vs macOS)
#   2. Installs platform-specific infrastructure (miniforge, RoboStack, etc.)
#   3. Creates appropriate environment (venv vs conda)
#   4. Installs ROS2 and all required development tools
#   5. Initializes git submodules (DynamixelSDK, hardware interfaces)
#   6. Builds all ROS2 packages in workspace
#   7. Validates the complete setup

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

MACOS-SPECIFIC NOTES:
    - The script handles Homebrew/conda git conflicts automatically
    - Git-based packages (git+https://) are pre-downloaded to avoid library conflicts
    - PyAudio requires Homebrew PortAudio for compilation (installed automatically)
    - If git issues persist, try: mamba install git -c conda-forge

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

# Check and install mamba/conda if needed (for macOS)
check_mamba() {
    if ! command -v mamba &> /dev/null; then
        if ! command -v conda &> /dev/null; then
            log_info "Neither mamba nor conda found. Installing miniforge automatically..."
            
            # Download and install miniforge
            local installer="Miniforge3-MacOSX-$(uname -m).sh"
            local url="https://github.com/conda-forge/miniforge/releases/latest/download/$installer"
            
            log_info "Downloading miniforge installer..."
            if ! curl -L -O "$url"; then
                log_error "Failed to download miniforge installer"
                exit 1
            fi
            
            log_info "Installing miniforge..."
            if ! bash "$installer" -b -p "$HOME/miniforge3"; then
                log_error "Failed to install miniforge"
                exit 1
            fi
            
            # Clean up installer
            rm -f "$installer"
            
            # Initialize conda for the current shell
            eval "$($HOME/miniforge3/bin/conda shell.$(basename $SHELL) hook)"
            
            log_success "Miniforge installed successfully"
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
    log_info "[1/6] Installing system dependencies..."
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

    # Step 2: Install ROS2 and required packages
    log_info "[2/6] Setting up ROS2 $ROS_DISTRO..."
    
    # Check if ROS2 base is installed, if not install it
    if ! command -v ros2 &> /dev/null; then
        log_info "Installing ROS2 $ROS_DISTRO..."
        
        # Add ROS2 apt repository
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        sudo apt update
    else
        log_success "ROS2 base already installed"
    fi
    
    # Always ensure all required packages are installed (whether ROS2 was already present or not)
    log_info "Ensuring all required ROS2 packages are installed..."
    sudo apt install -y \
        ros-$ROS_DISTRO-desktop \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-hardware-interface \
        ros-$ROS_DISTRO-controller-interface \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-ros2-control \
        ros-$ROS_DISTRO-ros2-controllers \
        python3-colcon-common-extensions
    
    log_success "All required ROS2 packages installed"

    # Step 3: Create Python virtual environment
    log_info "[3/6] Setting up Python virtual environment..."
    VENV_PATH="$REPO_ROOT/$ENV_NAME"
    
    if [ -d "$VENV_PATH" ]; then
        log_warning "Virtual environment already exists at: $VENV_PATH"
    else
        log_info "Creating virtual environment at: $VENV_PATH"
        python3 -m venv "$VENV_PATH"
        log_success "Virtual environment created"
    fi

    # Step 4: Install Python packages
    log_info "[4/6] Installing Python packages..."
    source "$VENV_PATH/bin/activate"
    pip install --upgrade pip
    
    if [ -f "$REQUIREMENTS_FILE" ]; then
        pip install -r "$REQUIREMENTS_FILE"
        log_success "Python packages installed from requirements.txt"
    else
        log_warning "requirements.txt not found, skipping Python package installation"
    fi

    # Step 5: Initialize git submodules and build ROS2 packages
    log_info "[5/6] Initializing git submodules..."
    cd "$REPO_ROOT"
    if ! git submodule update --init --recursive; then
        log_error "Failed to initialize git submodules"
        return 1
    fi
    log_success "Git submodules initialized successfully"
    
    log_info "[6/6] Building ROS2 packages..."
    cd "$REPO_ROOT/coffee_ws"
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
    log_success "ROS2 packages built successfully"
}

# Helper function: Handle git-based packages for macOS (Homebrew/conda conflict workaround)
handle_git_packages_macos() {
    log_info "Checking for git-based packages that need special handling..."
    
    # Create temporary directory for git package downloads
    local temp_dir=$(mktemp -d)
    local git_packages_found=false
    
    # Extract git-based packages from requirements.txt
    while IFS= read -r package || [[ -n "$package" ]]; do
        # Skip empty lines and comments
        [[ -z "$package" || "$package" =~ ^#.*$ ]] && continue
        
        # Check if this is a git-based package
        if [[ "$package" =~ ^git\+ ]]; then
            git_packages_found=true
            log_info "Found git-based package: $package"
            
            # Extract the git URL and package name
            local git_url=$(echo "$package" | sed 's/^git+//')
            local package_name=$(basename "$git_url" .git)
            
            log_info "Pre-downloading $package_name to avoid Homebrew/conda git conflicts..."
            
            # Download using system git (outside conda environment library conflicts)
            # We temporarily unset library search paths to use system libraries
            local orig_dyld_library_path="$DYLD_LIBRARY_PATH"
            unset DYLD_LIBRARY_PATH
            
            if git clone "$git_url" "$temp_dir/$package_name"; then
                log_success "Successfully downloaded $package_name"
                
                # Install from local directory using pip
                export DYLD_LIBRARY_PATH="$orig_dyld_library_path"
                log_info "Installing $package_name from local copy..."
                pip install "$temp_dir/$package_name"
                log_success "Installed $package_name"
            else
                # Restore library path and report error
                export DYLD_LIBRARY_PATH="$orig_dyld_library_path"
                log_error "Failed to download $package_name"
                log_error "You may need to install git in conda environment:"
                log_error "  mamba install git -c conda-forge"
                
                # Clean up and exit
                rm -rf "$temp_dir"
                return 1
            fi
            
            # Restore library path
            export DYLD_LIBRARY_PATH="$orig_dyld_library_path"
        fi
    done < "$REQUIREMENTS_FILE"
    
    # Clean up temporary directory
    rm -rf "$temp_dir"
    
    if [[ "$git_packages_found" == true ]]; then
        log_success "All git-based packages installed successfully"
    else
        log_info "No git-based packages found in requirements.txt"
    fi
}

# Helper function: Install regular (non-git) packages for macOS
install_regular_packages_macos() {
    log_info "Installing regular packages via conda-forge and pip..."
    
    # Process non-git packages from requirements.txt
    while IFS= read -r package || [[ -n "$package" ]]; do
        # Skip empty lines and comments
        [[ -z "$package" || "$package" =~ ^#.*$ ]] && continue
        
        # Skip git-based packages (already handled)
        if [[ "$package" =~ ^git\+ ]]; then
            continue
        fi
        
        package_name=$(echo "$package" | cut -d'=' -f1 | cut -d'>' -f1 | cut -d'<' -f1)
        
        # Special handling for PyAudio (avoid Homebrew/conda conflicts)
        if [[ "$package_name" == "PyAudio" ]]; then
            log_info "Installing PyAudio via conda (avoiding Homebrew conflicts)..."
            if mamba install -c conda-forge pyaudio -y 2>/dev/null; then
                log_success "PyAudio installed via conda"
                continue
            elif [[ "$HOMEBREW_AVAILABLE" == "true" ]]; then
                log_warning "PyAudio conda installation failed, trying pip with Homebrew PortAudio..."
                if pip install "$package"; then
                    log_success "PyAudio installed via pip with Homebrew PortAudio"
                else
                    log_error "PyAudio installation failed even with Homebrew PortAudio"
                    PYAUDIO_FAILED=true
                fi
            else
                log_warning "PyAudio installation skipped (Homebrew not available)"
                PYAUDIO_SKIPPED=true
            fi
        else
            # Try conda-forge first for other packages
            if mamba install -c conda-forge "$package_name" -y 2>/dev/null; then
                log_info "Installed $package_name via conda"
            else
                # Fallback to pip
                log_info "Installing $package_name via pip..."
                pip install "$package"
            fi
        fi
    done < "$REQUIREMENTS_FILE"
}

# macOS setup function  
setup_macos() {
    log_info "Setting up macOS environment with RoboStack..."
    
    # Step 1: Check mamba installation
    log_info "[1/6] Checking mamba installation..."
    check_mamba
    log_success "Mamba is available"

    # Step 1.5: Check Homebrew availability for PyAudio compilation
    HOMEBREW_AVAILABLE=false
    PYAUDIO_SKIPPED=false
    PYAUDIO_FAILED=false
    if command -v brew &> /dev/null; then
        log_info "Homebrew found. Ensuring PortAudio for PyAudio compilation..."
        if ! brew list portaudio &>/dev/null; then
            log_info "Installing PortAudio via Homebrew..."
            brew install portaudio
            log_success "PortAudio installed via Homebrew"
        else
            log_success "PortAudio already installed via Homebrew"
        fi
        HOMEBREW_AVAILABLE=true
    else
        log_warning "Homebrew not found. Voice functionality (PyAudio) will be skipped."
        log_info "You can install voice functionality later by installing Homebrew first."
    fi

    # Step 2: Create conda environment
    log_info "[2/6] Setting up conda environment '$ENV_NAME'..."
    
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
    log_info "[3/6] Installing ROS2 $ROS_DISTRO and development tools..."
    mamba install -y \
        ros-$ROS_DISTRO-desktop \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-hardware-interface \
        ros-$ROS_DISTRO-controller-interface \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-ros2-control \
        ros-$ROS_DISTRO-ros2-controllers \
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
    log_info "[4/6] Installing additional Python packages..."
    if [ -f "$REQUIREMENTS_FILE" ]; then
        # Step 4a: Handle git-based packages separately (macOS Homebrew/conda conflict workaround)
        handle_git_packages_macos
        
        # Step 4b: Install regular packages
        install_regular_packages_macos
        
        log_success "Additional Python packages installed"
    else
        log_warning "requirements.txt not found, skipping additional Python packages"
    fi

    # Deactivate and reactivate to ensure proper ROS setup
    mamba deactivate
    mamba activate "$ENV_NAME"

    # Step 5: Initialize git submodules and build ROS2 packages
    log_info "[5/6] Initializing git submodules..."
    cd "$REPO_ROOT"
    if ! git submodule update --init --recursive; then
        log_error "Failed to initialize git submodules"
        return 1
    fi
    log_success "Git submodules initialized successfully"
    
    log_info "[6/6] Building ROS2 packages..."
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
    
    # Test PyAudio functionality
    if python -c "import pyaudio; pa = pyaudio.PyAudio(); device_count = pa.get_device_count(); pa.terminate(); print(f'PyAudio devices: {device_count}')" 2>/dev/null; then
        log_success "PyAudio is working correctly"
    else
        log_warning "PyAudio import or initialization failed"
        if [[ "$PLATFORM" == "ubuntu" ]]; then
            log_warning "Try: sudo apt install portaudio19-dev"
        elif [[ "$PLATFORM" == "macos" ]]; then
            log_warning "Try: mamba install pyaudio -c conda-forge"
        fi
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
    echo "  1. To activate your development environment for daily use:"
    echo "     source $REPO_ROOT/scripts/activate_workspace.sh"
    echo ""
    echo "  2. Test your setup:"
    echo "     ros2 run coffee_voice_agent_ui voice_agent_monitor"
    echo ""
    
    # Check for PyAudio installation issues
    if [[ "$PYAUDIO_SKIPPED" == "true" ]]; then
        echo "⚠️  IMPORTANT: Voice functionality is not available"
        echo "   PyAudio was skipped because Homebrew is not installed."
        echo ""
        echo "   To enable voice functionality:"
        echo "   1. Install Homebrew:"
        echo "      /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        echo "   2. Re-run this setup script:"
        echo "      ./scripts/setup_workspace.sh"
        echo ""
    elif [[ "$PYAUDIO_FAILED" == "true" ]]; then
        echo "⚠️  WARNING: PyAudio installation failed"
        echo "   Voice functionality may not work properly."
        echo "   Try re-running this setup script or install PyAudio manually."
        echo ""
    fi
    
    echo "  3. If you encounter issues:"
    echo "     - Check the build logs in coffee_ws/log/"
    echo "     - Re-run this setup script"
    echo "     - Check the troubleshooting guide in README.md"
    echo ""
}

# Run main function with all arguments
main "$@" 