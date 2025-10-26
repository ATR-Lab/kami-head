# Coffee Buddy Robot

![Platform Support](https://img.shields.io/badge/Platform-Ubuntu%20%7C%20macOS-brightgreen)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.12-blue)

## Overview

Coffee Buddy is an interactive service robot designed for coffee shop environments. It features voice interaction, facial expressions, head tracking, and autonomous coffee machine operation through a comprehensive ROS2-based architecture.

## Platform Support

### Supported Platforms
- **Ubuntu 24.04** (Native ROS2 installation)
- **macOS** (RoboStack via mamba/conda)

### Cross-Platform Features
- **Automated Setup**: One-command environment setup for both platforms
- **Qt Application Compatibility**: Optimized PyQt UIs with platform-specific fixes
- **Cross-Platform Emoji System**: Ubuntu displays emojis, macOS uses text fallbacks
- **Audio Processing**: PyAudio with platform-specific installation handling

## Quick Start

### Prerequisites

**Ubuntu:**
- Ubuntu 24.04 (Noble Numbat)
- `sudo` privileges
- Internet connection

**macOS:**
- macOS 10.15+ 
- [Homebrew](https://brew.sh) installed
- [Miniforge](https://github.com/conda-forge/miniforge) or [Mambaforge](https://github.com/conda-forge/miniforge) installed
- Internet connection

### One-Command Setup

```bash
# Clone the repository
git clone <repository-url>
cd coffee-buddy

# Run automated setup (works on both Ubuntu and macOS)
bash scripts/setup_workspace.sh

# For specific ROS2 distribution (if needed)
bash scripts/setup_workspace.sh --ros-distro humble

# Activate the development environment
source scripts/activate_workspace.sh
```

### Quick Test

```bash
# Test the voice agent UI
ros2 run coffee_voice_agent_ui voice_agent_monitor

# Test head tracking (if hardware available)
ros2 launch coffee_head all_nodes.launch.py

# Test facial expressions
ros2 launch coffee_face coffee_eyes.launch.py
```

## Architecture

### Core Components

**Head Control System:**
- Multi-servo head tracking with Dynamixel motors
- Real-time face detection and tracking
- Smooth motion interpolation

**Voice Interaction:**
- Speech-to-text processing
- LLM-powered conversation system
- Text-to-speech with emotional expression

**Visual Expression:**
- Animated eye display system
- Emotion-based facial expressions
- Cross-platform emoji compatibility

**Coffee Machine Interface:**
- Automated brewing control
- Order processing and queue management
- Safety monitoring and error handling

### Platform-Specific Features

**macOS Optimizations:**
- RoboStack integration for native ROS2 support
- Qt application fixes for external terminal compatibility  
- Emoji fallback system for stable rendering
- Automatic Qt environment configuration

**Ubuntu Optimizations:**
- Native ROS2 package installation
- Full emoji support in Qt applications
- Optimized audio pipeline with ALSA/PulseAudio

## Installation

### Automated Installation (Recommended)

The setup script handles all platform differences automatically:

```bash
# Setup everything
bash scripts/setup_workspace.sh --help  # See all options

# Default setup (ROS2 Humble)
bash scripts/setup_workspace.sh

# With specific ROS2 distribution (e.g., for Jazzy)
bash scripts/setup_workspace.sh --ros-distro jazzy

# Clean installation (removes existing environment)
bash scripts/setup_workspace.sh --clean
```

### Daily Usage

```bash
# Activate development environment (default: ros_env, humble)
source scripts/activate_workspace.sh

# Activate with specific environment name
source scripts/activate_workspace.sh my_ros_env

# Activate with specific ROS2 distribution
source scripts/activate_workspace.sh ros_env humble

# Build packages (if needed)
colcon build --symlink-install

# Run applications
ros2 run <package> <executable>
```

### Manual Installation

<details>
<summary>Click to expand manual installation steps</summary>

**Ubuntu 24.04:**
```bash
# Install ROS2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-pip

# Audio dependencies
sudo apt install portaudio19-dev

# Python virtual environment
python3 -m venv coffee_venv
source coffee_venv/bin/activate
pip install -r requirements.txt

# Build workspace
cd coffee_ws
colcon build --symlink-install
```

**macOS:**
```bash
# Install RoboStack environment
mamba create -n ros_env
mamba activate ros_env
mamba install ros-humble-desktop compilers cmake pkg-config make ninja colcon-common-extensions rosdep python pip

# Audio dependencies  
brew install portaudio
pip install PyAudio

# Python dependencies
pip install -r requirements.txt

# Build workspace
cd coffee_ws
colcon build --symlink-install
```

</details>

## Usage

### Voice Agent Interface

```bash
# Launch the voice agent monitor UI
ros2 run coffee_voice_agent_ui voice_agent_monitor

# Launch voice processing services
ros2 launch coffee_voice_service voice_service.launch.py
```

### Head Control

```bash
# Launch all head control nodes
ros2 launch coffee_head all_nodes.launch.py

# Manual head control interface
ros2 run coffee_head_manual_control manual_control_ui

# Record head motion sequences
ros2 run coffee_head_motion_recorder motion_recorder
```

### Facial Expressions

```bash
# Launch animated eyes
ros2 launch coffee_face coffee_eyes.launch.py

# Test expression states
ros2 run coffee_expressions expression_test_ui
```

### Coffee Machine Control

```bash
# Launch coffee machine interface
ros2 launch coffee_machine_control coffee_control.launch.py

# Order processing interface
ros2 run coffee_machine_control order_ui
```

## Development

### Cross-Platform Considerations

**Qt Applications on macOS:**
- Environment variables are automatically configured
- Emoji rendering uses text fallbacks to prevent crashes
- External terminal compatibility is handled automatically

**Audio Processing:**
- PyAudio installation is handled platform-specifically
- Ubuntu uses system PortAudio, macOS uses Homebrew PortAudio

**Package Management:**
- Ubuntu: apt + pip in venv
- macOS: mamba + pip in conda environment

### Adding New Packages

```bash
# Create new ROS2 package
cd coffee_ws/src
ros2 pkg create --build-type ament_python my_package

# Add dependencies to package.xml
# Add Python dependencies to setup.py
# Build and test
colcon build --packages-select my_package
```

### Cross-Platform Testing

```bash
# Test on current platform
colcon test --packages-select my_package

# Platform-specific testing
source scripts/activate_workspace.sh
ros2 run my_package my_executable

# Test with different ROS2 distributions
source scripts/activate_workspace.sh ros_env jazzy
ros2 run my_package my_executable
```

## Troubleshooting

### Common Issues

**RoboStack Environment Issues (macOS):**
```bash
# Check environment
mamba info
mamba list | grep ros

# Recreate environment
mamba remove -n ros_env --all
bash scripts/setup_workspace.sh --clean
```

**Git/Homebrew Conflicts During Setup (macOS):**
- **Problem**: `git version did not run successfully` with `Symbol not found: _iconv`
- **Solution**: Automatically handled by `setup_workspace.sh` (pre-downloads git packages)
- **Manual fix if needed**: `mamba install git -c conda-forge`

**PyAudio Installation Issues:**
- **Ubuntu**: Automatically installs via pip after installing `portaudio19-dev`
- **macOS**: Automatically installs via pip using Homebrew PortAudio
- **Manual fix if needed**: 
  - Ubuntu: `sudo apt install portaudio19-dev && pip install PyAudio`
  - macOS: `brew install portaudio && pip install PyAudio`

**macOS Qt Application Crashes:**
- **Problem**: `Bus error: 10` in external terminals
- **Solution**: Emoji characters cause Qt crashes; system uses text fallbacks automatically
- **Manual fix**: Ensure you're using the proper activation script

**Package Not Found After Build:**
- **Problem**: `Package 'package_name' not found`
- **Solution**: Ensure you're sourcing from the correct directory
```bash
cd coffee_ws/install
source setup.bash
cd ..
```

**Terminal Compatibility (macOS):**
- ✅ **VS Code integrated terminal**: Full support
- ✅ **Terminal.app with activation script**: Full support  
- ❌ **Raw terminal without activation**: Qt environment not configured

### Environment Validation

```bash
# Check ROS2 installation
ros2 doctor

# Verify workspace packages
ros2 pkg list | grep coffee

# Test PyAudio functionality
python -c "import pyaudio; print('PyAudio OK')"

# Check Qt environment (macOS)
echo $QT_QPA_PLATFORM
```

### Getting Help

1. **Check logs**: `ros2 doctor` and individual node logs
2. **Validate environment**: Run environment validation commands above
3. **Clean rebuild**: `rm -rf coffee_ws/build coffee_ws/install && colcon build`
4. **Reset environment**: Use `--clean` flag with setup script

## Contributing

### Development Setup

```bash
# Fork and clone your fork
git clone <your-fork-url>
cd coffee-buddy

# Setup development environment  
bash scripts/setup_workspace.sh
source scripts/activate_workspace.sh

# Or with specific ROS2 distribution
bash scripts/setup_workspace.sh --ros-distro humble
source scripts/activate_workspace.sh ros_env humble

# Create feature branch
git checkout -b feature/your-feature

# Make changes and test
colcon build --symlink-install
colcon test

# Submit pull request
```

### Code Standards

- Follow [ROS2 Coding Conventions](ROS2_CODING_CONVENTIONS.md)
- Use type hints in Python code
- Include docstrings for all public functions
- Test on both Ubuntu and macOS when possible

## Repository Structure

```
coffee-buddy/
├── coffee_ws/                  # ROS2 workspace
│   └── src/                   # Source packages
│       ├── coffee_voice_agent/         # Voice interaction system
│       ├── coffee_head_control/        # Head tracking and control
│       ├── coffee_face/               # Facial expression system
│       ├── coffee_machine_control/    # Coffee machine interface
│       ├── coffee_vision/             # Computer vision
│       └── ...                        # Additional packages
├── scripts/                   # Setup and utility scripts
│   ├── setup_workspace.sh     # One-time environment setup
│   └── activate_workspace.sh  # Daily environment activation  
├── requirements.txt           # Python dependencies
└── README.md                 # This file
```

## License

[License information to be added]