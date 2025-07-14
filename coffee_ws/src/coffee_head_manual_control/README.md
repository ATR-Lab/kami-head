# Coffee Head Manual Control Package

A ROS2 package for manual control of the Coffee Buddy robot head using various input methods including joystick, keyboard, and other manual interfaces.

## Overview

The `coffee_head_manual_control` package provides manual control capabilities for the Coffee Buddy robot head system. It serves as the counterpart to the autonomous `coffee_head_control` package, allowing operators to take direct control of head movements.

**Current Features:**
- **Joystick Control**: Direct head control using gaming controllers
- **Smooth Motion**: Configurable smoothing and deadzone handling
- **Multi-axis Control**: Independent yaw, pitch, and roll control
- **Real-time Response**: 50Hz update rate for responsive control

**Planned Features:**
- **Keyboard Control**: Arrow keys and WASD control schemes
- **Gamepad Support**: Extended controller compatibility
- **Mobile App**: Remote control via smartphone/tablet
- **Voice Commands**: Simple directional voice control

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                Manual Control Package                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ JoystickControl │  │ KeyboardControl │  │ Other Inputs    │  │
│  │ - Pygame input  │  │ - Key mapping   │  │ - Future expand │  │
│  │ - Axis mapping  │  │ - Rate control  │  │ - Voice/Mobile  │  │
│  │ - Smoothing     │  │ - Combinations  │  │ - Custom HID    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                          Motor Control Topics:
                          • set_position_yaw
                          • set_position_pitch  
                          • set_position_roll
```

## Components

### JoystickControlNode

Main node for joystick-based head control using gaming controllers.

**Publishers:**
- `set_position_yaw` (dynamixel_sdk_custom_interfaces/SetPosition): Horizontal rotation commands
- `set_position_pitch` (dynamixel_sdk_custom_interfaces/SetPosition): Vertical tilt commands  
- `set_position_roll` (dynamixel_sdk_custom_interfaces/SetPosition): Roll rotation commands

**Input Mapping:**
- **Left Stick X-axis**: Yaw (horizontal rotation)
- **Right Stick Y-axis**: Pitch (vertical tilt)
- **Left Trigger**: Roll left
- **Right Trigger**: Roll right
- **Combined Triggers**: Roll control (LT = left, RT = right)

**Features:**
- **Deadzone Handling**: Configurable deadzone to prevent drift (default: 0.1)
- **Motion Smoothing**: Adjustable smoothing factor for fluid movement (default: 0.3)
- **Real-time Processing**: 50Hz update rate for responsive control
- **Error Handling**: Graceful handling of joystick disconnection

## Installation

### Dependencies

- **ROS2**: rclpy, std_msgs, geometry_msgs
- **Motor Control**: dynamixel_sdk_custom_interfaces
- **Input Handling**: python3-pygame
- **Math**: numpy (included with ROS2)

### Hardware Requirements

- **Joystick/Gamepad**: Any pygame-compatible gaming controller
  - Xbox controllers (wired/wireless)
  - PlayStation controllers (DS4, DualSense)
  - Generic USB gamepads
  - Logitech controllers

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd coffee_ws

# Build the package
colcon build --packages-select coffee_head_manual_control

# Source the workspace
source install/setup.bash
```

## Usage

### Joystick Control

```bash
# Connect your joystick/gamepad first
# Launch joystick control node
ros2 run coffee_head_manual_control joystick_control

# Verify joystick is detected
# Check the terminal output for "Initialized joystick: [Controller Name]"
```

### Manual Testing

```bash
# Monitor the motor commands being sent
ros2 topic echo /set_position_yaw
ros2 topic echo /set_position_pitch  
ros2 topic echo /set_position_roll

# Check joystick detection
ros2 node info /joystick_control_node
```

## Configuration

### Joystick Parameters

The joystick control can be customized by modifying the following parameters in the code:

```python
# Control sensitivity and response
self.deadzone = 0.1          # Ignore movements below this threshold (0.0-1.0)
self.max_angle = 1.0         # Maximum angle in radians
self.smooth_factor = 0.3     # Motion smoothing (0.0=none, 1.0=maximum)

# Update rate
create_timer(0.02, ...)      # 50Hz = 0.02s interval
```

### Input Mapping

**Standard Gamepad Layout:**
```
Left Stick:
  X-axis → Yaw (head rotation left/right)
  Y-axis → (unused)

Right Stick:  
  X-axis → (unused)
  Y-axis → Pitch (head tilt up/down)

Triggers:
  Left Trigger → Roll left
  Right Trigger → Roll right
```

**Axis Values:**
- Range: -1.0 to +1.0
- Deadzone applied automatically
- Converted to motor position units (x1000)

## Integration

### Motor Service Integration

Requires compatible motor control service to be running:
```bash
# Start motor service (if using dynamixel_sdk_examples)
ros2 run dynamixel_sdk_examples read_write_node

# Or start your custom motor service
```

### Coordination with Autonomous Control

**Important**: Manual control and autonomous head tracking should not run simultaneously as they will conflict. Consider implementing:

1. **Mode Switching**: Toggle between manual and autonomous modes
2. **Priority System**: Manual input overrides autonomous tracking
3. **Timeout Handling**: Return to autonomous mode after manual inactivity

### Topic Compatibility

Current topic structure (`set_position_yaw`, `set_position_pitch`, `set_position_roll`) differs from the autonomous control package (`set_position`). Future updates may standardize this interface.

## Troubleshooting

### Joystick Issues

**No joystick detected:**
```bash
# Check if joystick is connected
ls /dev/input/js*

# Test with pygame directly
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print(f'Found {pygame.joystick.get_count()} joysticks')"

# Check permissions
sudo chmod 666 /dev/input/js0
```

**Joystick not responding:**
- Verify the controller is in the correct mode (if applicable)
- Check battery level for wireless controllers
- Try unplugging and reconnecting
- Restart the node: `Ctrl+C` and rerun

**Erratic movement:**
- Increase deadzone value if controller has stick drift
- Check for interference with wireless controllers
- Calibrate controller using system tools

### Motor Issues

**No motor response:**
```bash
# Check if motor service is running
ros2 service list | grep position

# Verify topics are being published
ros2 topic hz /set_position_yaw

# Check motor connection and power
```

**Jerky movement:**
- Increase smooth_factor for smoother motion
- Reduce update rate if system is overloaded
- Check for network latency if using remote control

## Development

### Adding New Input Methods

To add keyboard control or other input methods:

1. **Create new node file**:
   ```bash
   touch coffee_head_manual_control/keyboard_control_node.py
   ```

2. **Update setup.py entry points**:
   ```python
   'console_scripts': [
       'joystick_control = coffee_head_manual_control.joystick_control_node:main',
       'keyboard_control = coffee_head_manual_control.keyboard_control_node:main',
   ],
   ```

3. **Follow existing patterns**:
   - Use same topic structure
   - Implement similar smoothing and deadzone handling
   - Add proper error handling and cleanup

### Extending Joystick Support

**Custom Controller Mapping:**
```python
# Add controller-specific mappings
CONTROLLER_MAPPINGS = {
    'Xbox Controller': {
        'yaw_axis': 0,
        'pitch_axis': 3,
        'roll_left': 2,
        'roll_right': 5
    },
    'PS4 Controller': {
        'yaw_axis': 0,
        'pitch_axis': 3,
        'roll_left': 2,
        'roll_right': 5
    }
}
```

**Advanced Features:**
- Button mapping for preset positions
- Gesture recognition
- Force feedback support
- Multi-controller support

## Future Enhancements

### Planned Input Methods

1. **Keyboard Control**:
   - Arrow keys for basic movement
   - WASD for alternative layout
   - Modifier keys for different speeds
   - Number keys for preset positions

2. **Mobile App**:
   - Touch joystick interface
   - Gesture-based control
   - Preset position buttons
   - Real-time video feedback

3. **Voice Commands**:
   - "Look left", "Look up", "Center head"
   - Speed modifiers: "Slowly look right"
   - Integration with existing voice agent

### Advanced Features

- **Motion Recording**: Record and playback manual movements
- **Macro Support**: Define complex movement sequences
- **Collaborative Control**: Multiple operators sharing control
- **Safety Zones**: Configurable movement limits
- **Haptic Feedback**: Force feedback for boundaries

## Safety Considerations

- **Movement Limits**: Respect motor angle limits to prevent damage
- **Emergency Stop**: Implement immediate stop capability
- **Smooth Transitions**: Avoid sudden jerky movements
- **Power Management**: Consider motor heating during extended use
- **Conflict Prevention**: Ensure only one control method is active

## License

TODO: License declaration

## Contributing

When adding new manual control methods:
1. Follow the existing code structure and patterns
2. Add comprehensive error handling
3. Update this README with new features
4. Include example usage and troubleshooting
5. Test with various input devices
6. Consider accessibility requirements
