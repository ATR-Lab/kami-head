# Coffee Machine Control

A ROS2 package for controlling Delonghi Prima Donna coffee machines via Bluetooth Low Energy (BLE).

## Overview

The `coffee_machine_control` package provides a ROS2 interface to remotely control Delonghi coffee machines using Bluetooth connectivity. It supports brewing various beverages, controlling machine settings, and monitoring machine status through standardized ROS2 services and messages.

## Features

### Beverage Control
- **Espresso** - Single and double shots
- **Long Coffee** - Extended brew
- **Americano** - Espresso with hot water
- **Doppio** - Double espresso
- **Hot Water** - For tea or manual coffee preparation
- **Steam** - For milk frothing

### Machine Settings
- **Cup Light** - Toggle illumination on/off
- **Sound Alerts** - Enable/disable audio feedback
- **Energy Save Mode** - Power management settings
- **Machine Power** - Turn on/off

### Status Monitoring
- Real-time machine status
- Current beverage being prepared
- Settings state (lights, sounds, energy mode)
- Device information (name, model)

## Architecture

```
┌─────────────────────────────────────────┐
│              ROS2 Client                │
│         (Your Application)              │
└─────────────────┬───────────────────────┘
                  │ ROS2 Services/Messages
┌─────────────────▼───────────────────────┐
│       CoffeeMachineControlNode          │
│    - Service handlers                   │
│    - Async command execution            │
│    - Status monitoring                  │
└─────────────────┬───────────────────────┘
                  │ Bluetooth Commands
┌─────────────────▼───────────────────────┐
│       DelonghiController                │
│    - BLE communication                  │
│    - Command encoding                   │
│    - Device management                  │
└─────────────────┬───────────────────────┘
                  │ Bluetooth LE
┌─────────────────▼───────────────────────┐
│       Delonghi Coffee Machine           │
│          (Prima Donna)                  │
└─────────────────────────────────────────┘
```

## Installation

### Prerequisites
- ROS2 (tested with appropriate distribution)
- Python 3.8+
- Bluetooth Low Energy support
- `bleak` Python library for BLE communication

### Dependencies
```bash
# Install Python dependencies
pip install bleak

# Build the workspace (from coffee_ws directory)
colcon build --packages-select coffee_machine_control coffee_machine_control_msgs
source install/setup.bash
```

## Usage

### Launch the Node

#### With Physical Coffee Machine
```bash
# Launch with default MAC address
ros2 launch coffee_machine_control coffee_machine_control.launch.py

# Launch with specific MAC address
ros2 launch coffee_machine_control coffee_machine_control.launch.py mac_address:="YOUR_MACHINE_MAC"
```

#### Mock Mode (for Testing)
The node automatically falls back to mock mode if no physical machine is detected.

### Service Interfaces

#### Coffee Commands
```bash
# Make an espresso
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand \
  "{action: 'make', parameter: 'espresso'}"

# Cancel current operation
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand \
  "{action: 'cancel', parameter: ''}"

# Turn cup light on
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand \
  "{action: 'cuplight', parameter: 'on'}"

# Enable energy save mode
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand \
  "{action: 'energy_save', parameter: 'on'}"
```

#### Status Requests
```bash
# Get machine status
ros2 service call /coffee_machine/get_status coffee_machine_control_msgs/srv/MachineStatusRequest
```

### Available Commands

| Action | Parameter | Description |
|--------|-----------|-------------|
| `make` | `espresso`, `americano`, `long`, `coffee`, `doppio`, `hot_water`, `steam` | Brew specified beverage |
| `cancel` | - | Cancel current operation |
| `cuplight` | `on`, `off` | Control cup illumination |
| `sound` | `on`, `off` | Enable/disable sound alerts |
| `energy_save` | `on`, `off` | Toggle energy saving mode |
| `power` | `on` | Turn machine on |

## Configuration

### Launch Parameters
- `mac_address`: Bluetooth MAC address of your coffee machine (string)
  - Default: `9C:95:6E:61:B6:2C`
  - Find your machine's MAC using: `bluetoothctl` or `hcitool lescan`

### Finding Your Machine's MAC Address
1. Enable Bluetooth on your system
2. Put your coffee machine in pairing mode
3. Scan for devices:
   ```bash
   bluetoothctl
   scan on
   # Look for "Delonghi" or similar device name
   ```

## Message Types

### FunctionCall.msg
```
string name            # Function name (e.g., "make_coffee")
string parameter       # Optional parameter (e.g., "espresso")
```

### Service Types

#### CoffeeCommand.srv
```
# Request
string action        # Action to perform
string parameter     # Action parameter
---
# Response
bool success         # Command success status
string message       # Response message
```

#### MachineStatusRequest.srv
```
# Request (empty)
---
# Response
string device_name     # Machine device name
string model          # Machine model
string status         # Current status
string steam_nozzle   # Steam nozzle state
string current_beverage # Currently brewing beverage
bool cup_light        # Cup light state
bool energy_save      # Energy save mode state
bool sound_enabled    # Sound alerts state
```

## Development

### Mock Mode
For development and testing without a physical machine, the node includes a mock implementation that simulates:
- Brewing operations with realistic timing
- Status responses
- Setting changes
- Error conditions

### Extending Functionality
To add new beverages or commands:
1. Add entries to `AvailableBeverage` enum in `delonghi_controller.py`
2. Define command bytes in the same file
3. Update the command handler in `coffee_machine_control_node.py`
4. Add documentation to this README

## Troubleshooting

### Common Issues

#### Bluetooth Connection Failed
- Ensure machine is powered on and in pairing mode
- Verify MAC address is correct
- Check Bluetooth permissions: `sudo usermod -a -G bluetooth $USER`
- Restart Bluetooth service: `sudo systemctl restart bluetooth`

#### Service Call Timeouts
- Machine may be busy with another operation
- Check physical machine status
- Ensure machine is connected to power

#### Mock Mode Always Active
- Verify MAC address parameter
- Check Bluetooth adapter is working: `hciconfig`
- Ensure machine is discoverable

### Logs
```bash
# View node logs
ros2 topic echo /rosout

# Debug Bluetooth issues
journalctl -u bluetooth
```
