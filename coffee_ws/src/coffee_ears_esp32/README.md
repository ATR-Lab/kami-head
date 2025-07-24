# Coffee Ears ESP32 Package

ROS2 package for controlling Coffee Buddy robot ears via ESP32 and micro-ROS agent. This package manages the Docker container lifecycle for micro-ROS communication with the ESP32 ear motion controller.

## Overview

This package replaces the manual Docker command with a proper ROS2 integration that provides:

- **Automated micro-ROS agent management** via Docker containers
- **Health monitoring and auto-restart** capabilities  
- **Configurable parameters** for device paths and settings
- **ROS2 launch file integration** with the Coffee Buddy system
- **Status monitoring** and diagnostics

## Current vs New Approach

**Before (Manual):**
```bash
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB1 -v6
```

**After (ROS2 Integrated):**
```bash
ros2 launch coffee_ears_esp32 ears_system.launch.py
```

## Installation

### Build the Package

```bash
cd coffee_ws
colcon build --packages-select coffee_ears_esp32
source install/setup.bash
```

## Usage

### Launch Options

#### 1. Complete Ear System (Recommended)
```bash
# Start complete system with defaults
ros2 launch coffee_ears_esp32 ears_system.launch.py

# With custom device
ros2 launch coffee_ears_esp32 ears_system.launch.py device_path:=/dev/ttyUSB0

# With custom settings
ros2 launch coffee_ears_esp32 ears_system.launch.py device_path:=/dev/ttyUSB0 verbosity:=4
```

#### 2. Micro-ROS Agent Only
```bash
# Start just the micro-ROS agent manager
ros2 launch coffee_ears_esp32 microros_agent.launch.py

# With custom parameters
ros2 launch coffee_ears_esp32 microros_agent.launch.py \
  device_path:=/dev/ttyUSB0 \
  verbosity:=4 \
  auto_start:=false
```

#### 3. Manual Script (Alternative)
```bash
# Use the bash script directly
./src/coffee_ears_esp32/scripts/start_microros_agent.sh

# With options
./src/coffee_ears_esp32/scripts/start_microros_agent.sh --device /dev/ttyUSB0 --verbosity 4
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_path` | `/dev/ttyUSB1` | Serial device path for ESP32 |
| `verbosity` | `6` | micro-ROS agent verbosity (0-6) |
| `auto_start` | `true` | Start agent automatically |
| `restart_on_failure` | `true` | Auto-restart on failures |
| `health_check_interval` | `5.0` | Health check frequency (seconds) |

## ROS2 Interface

### Topics

#### Published by micro-ROS Manager:
- `/microros_agent/status` (std_msgs/Bool) - Agent running status
- `/microros_agent/diagnostics` (std_msgs/String) - Diagnostic messages

#### Subscribed by micro-ROS Manager:
- `/microros_agent/control` (std_msgs/Bool) - Start/stop agent commands

#### ESP32 Interface (from Arduino code):
- `/ear_motion_command` (std_msgs/Int32) - Send motion commands to ESP32

### Commands

#### Control Ear Motions
```bash
# Send predefined motion commands (1-12)
ros2 topic pub /ear_motion_command std_msgs/Int32 "data: 1"   # Ear twitch
ros2 topic pub /ear_motion_command std_msgs/Int32 "data: 4"   # Alert ears
ros2 topic pub /ear_motion_command std_msgs/Int32 "data: 8"   # Happy ears
```

#### Monitor System Status
```bash
# Check if micro-ROS agent is running
ros2 topic echo /microros_agent/status

# Monitor diagnostics
ros2 topic echo /microros_agent/diagnostics

# Check if ESP32 node is connected
ros2 node list | grep ear_motion_node
```

#### Control Agent
```bash
# Start the agent
ros2 topic pub /microros_agent/control std_msgs/Bool "data: true"

# Stop the agent  
ros2 topic pub /microros_agent/control std_msgs/Bool "data: false"
```

## ESP32 Motion Library

The ESP32 implements 12 predefined ear motions:

1. **Ear twitch** - Quick ear movement
2. **Ear wiggle** - Alternating ear movement  
3. **Ear sweep** - Full range motion
4. **Alert ears** - Focused attention position
5. **Relaxed ears** - Calm position
6. **Curious tilt** - Questioning position
7. **Startled ears** - Quick surprised motion
8. **Happy ears** - Joyful wiggling
9. **Annoyed flick** - Quick dismissive motion
10. **Scanning ears** - Listening position
11. **Sleepy ears** - Tired position
12. **Greeting ears** - Welcoming motion

## Integration with Coffee Buddy System

### Expression System Integration
```python
# Include in system-wide launch files
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('coffee_ears_esp32'),
        '/launch/ears_system.launch.py'
    ]),
    launch_arguments={'device_path': '/dev/ttyUSB1'}.items()
)
```

### Emotion Mapping
The ear motions can be mapped to Coffee Buddy emotions:
- **Alert** → Wake word detection, focused attention
- **Happy** → Positive interactions, successful operations
- **Startled** → Unexpected events, errors
- **Sleepy** → Idle state, low power mode
- **Curious** → Processing user input, thinking

## Troubleshooting

### Common Issues

#### Device Not Found
```bash
# Check available devices
ls -la /dev/tty* | grep -E "(USB|ACM)"

# Try different device path
ros2 launch coffee_ears_esp32 ears_system.launch.py device_path:=/dev/ttyUSB0
```

#### Permission Denied
```bash
# Add user to dialout group (logout/login required)
sudo usermod -a -G dialout $USER

# Or temporary fix
sudo chmod 666 /dev/ttyUSB1
```

#### Docker Issues
```bash
# Check if Docker is running
sudo systemctl status docker

# Add user to docker group (logout/login required)  
sudo usermod -a -G docker $USER

# Pull the image manually
sudo docker pull microros/micro-ros-agent:jazzy
```

#### ESP32 Connection Issues
```bash
# Monitor serial connection
sudo dmesg | tail -f

# Check micro-ROS agent logs
ros2 topic echo /microros_agent/diagnostics

# Restart the ESP32
ros2 topic pub /microros_agent/control std_msgs/Bool "data: false"
ros2 topic pub /microros_agent/control std_msgs/Bool "data: true"
```

### Monitoring

#### System Health
```bash
# Check all ear-related nodes
ros2 node list | grep -E "(ear|microros)"

# Monitor connection status
ros2 topic hz /microros_agent/status

# View all ear topics
ros2 topic list | grep -E "(ear|microros)"
```

## Development

### Package Structure
```
coffee_ears_esp32/
├── coffee_ears_esp32/
│   ├── __init__.py
│   └── microros_manager.py      # Main manager node
├── launch/
│   ├── microros_agent.launch.py # Agent only
│   └── ears_system.launch.py    # Complete system
├── scripts/
│   └── start_microros_agent.sh  # Manual script
├── package.xml
├── setup.py
└── README.md
```

### Adding New Features

To extend the package:
1. Modify `microros_manager.py` for new container management features
2. Add new launch files for different configurations
3. Update parameters in launch files for new settings

## License

Apache-2.0 