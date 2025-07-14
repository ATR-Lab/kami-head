# coffee_machine_control_msgs

A ROS2 package providing message and service definitions for coffee machine control and monitoring.

## Overview

This package contains the interface definitions (messages and services) used to communicate with and control coffee machines in the Coffee Buddy robot system. It provides standardized message types for machine status reporting and command execution.

## Package Information

- **Version**: 0.0.1
- **License**: TODO: License declaration
- **Build Type**: ament_cmake
- **Dependencies**: rosidl_default_generators, rosidl_default_runtime

## Messages

### MachineStatus.msg

Represents the current status and configuration of a coffee machine.

```msg
string device_name
string model
string status
string steam_nozzle
string current_beverage
bool cup_light
bool energy_save
bool sound_enabled
```

**Fields:**
- `device_name`: Name/identifier of the coffee machine device
- `model`: Model name of the coffee machine
- `status`: Current operational status (e.g., "ready", "brewing", "error")
- `steam_nozzle`: Status of the steam nozzle
- `current_beverage`: Currently selected or brewing beverage type
- `cup_light`: Whether the cup light is enabled
- `energy_save`: Whether energy saving mode is active
- `sound_enabled`: Whether sound notifications are enabled

### FunctionCall.msg

Represents a function call or action to be performed on the coffee machine.

```msg
string name            # e.g., "make_coffee", "turn_cuplight_on", "cancel_brewing"
string parameter       # Optional parameter like "espresso" or "on"
```

**Fields:**
- `name`: Name of the function/action to execute
- `parameter`: Optional parameter for the function (e.g., beverage type, on/off state)

**Example function names:**
- `make_coffee`: Brew coffee with optional beverage type parameter
- `turn_cuplight_on`/`turn_cuplight_off`: Control cup light
- `cancel_brewing`: Cancel current brewing operation

## Services

### MachineStatusRequest.srv

Service to request the current status of the coffee machine.

```srv
# Empty request
---
string device_name
string model
string status
string steam_nozzle
string current_beverage
bool cup_light
bool energy_save
bool sound_enabled
```

**Request**: Empty (no parameters required)

**Response**: Returns a complete `MachineStatus` with all current machine information.

### CoffeeCommand.srv

Service to send commands to the coffee machine and receive execution results.

```srv
# Request
string action        # "make", "cancel", "cuplight", "sound", "energy_save"
string parameter     # Parameter depending on action (e.g., "espresso", "on", "off")
---
# Response
bool success
string message
```

**Request:**
- `action`: The action to perform on the machine
- `parameter`: Action-specific parameter

**Response:**
- `success`: Whether the command was executed successfully
- `message`: Human-readable result or error message

**Supported Actions:**
- `make`: Start brewing with parameter specifying beverage type
- `cancel`: Cancel current operation
- `cuplight`: Control cup light (parameter: "on"/"off")
- `sound`: Control sound notifications (parameter: "on"/"off")
- `energy_save`: Control energy saving mode (parameter: "on"/"off")

## Usage Examples

### Python Examples

#### Requesting Machine Status

```python
import rclpy
from rclpy.node import Node
from coffee_machine_control_msgs.srv import MachineStatusRequest

class StatusClient(Node):
    def __init__(self):
        super().__init__('status_client')
        self.client = self.create_client(MachineStatusRequest, 'coffee_machine_status')
        
    def get_status(self):
        request = MachineStatusRequest.Request()
        future = self.client.call_async(request)
        # Handle response...
```

#### Sending Coffee Commands

```python
import rclpy
from rclpy.node import Node
from coffee_machine_control_msgs.srv import CoffeeCommand

class CommandClient(Node):
    def __init__(self):
        super().__init__('command_client')
        self.client = self.create_client(CoffeeCommand, 'coffee_machine_command')
        
    def make_espresso(self):
        request = CoffeeCommand.Request()
        request.action = "make"
        request.parameter = "espresso"
        future = self.client.call_async(request)
        # Handle response...
```

#### Publishing Function Calls

```python
import rclpy
from rclpy.node import Node
from coffee_machine_control_msgs.msg import FunctionCall

class FunctionPublisher(Node):
    def __init__(self):
        super().__init__('function_publisher')
        self.publisher = self.create_publisher(FunctionCall, 'coffee_function_calls', 10)
        
    def publish_function_call(self, name, parameter=""):
        msg = FunctionCall()
        msg.name = name
        msg.parameter = parameter
        self.publisher.publish(msg)
```

## Building

This package uses the standard ROS2 build system:

```bash
# From your workspace root
colcon build --packages-select coffee_machine_control_msgs
```

## Dependencies

- ROS2 (tested with Humble/Iron)
- rosidl_default_generators (build dependency)
- rosidl_default_runtime (runtime dependency)

## Integration

This package is designed to work with:
- `coffee_machine_control`: Main coffee machine control node
- Other coffee buddy system packages that need to interact with coffee machines