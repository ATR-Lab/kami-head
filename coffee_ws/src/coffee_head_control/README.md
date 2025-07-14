# Coffee Head Control Package

A ROS2 package for controlling the physical head movements of the Coffee Buddy robot, including face tracking, motor control, and interactive GUI interface.

## Overview

The `coffee_head_control` package provides comprehensive head movement control for the Coffee Buddy robot system. It includes:

- **Face Tracking**: Automatically tracks detected faces using camera input
- **Motor Control**: Controls pan/tilt Dynamixel motors for head movement
- **PID Control**: Smooth, precise motor movements with configurable PID controllers
- **Interactive GUI**: Qt-based interface for manual control and parameter tuning
- **Scanning Mode**: Automatic scanning when no faces are detected
- **Coordinate Transformation**: Integrates with eye tracking systems

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Head Tracking Node                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ HeadTrackingUI  │  │HeadTrackingSystem│  │  PIDController  │  │
│  │ - Qt Controls   │  │ - Face tracking  │  │ - Smooth motion │  │
│  │ - Parameter UI  │  │ - Motor commands │  │ - Anti-windup   │  │
│  │ - Status        │  │ - Scanning mode  │  │ - Rate limiting │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                          Dynamixel Motors:
                          • Pan Motor (ID: 1)
                          • Tilt Motor (ID: 9)
```

## Components

### HeadTrackingNode

Main ROS2 node that coordinates head tracking functionality.

**Subscribers:**
- `/robot/affective_state` (coffee_expressions_msgs/AffectiveState): Face detection data

**Publishers:**
- `set_position` (dynamixel_sdk_custom_interfaces/SetPosition): Motor position commands
- `face_velocity` (geometry_msgs/Vector3): Face velocity information
- `head_pan_angle` (std_msgs/Float32): Current pan angle for eye coordination
- `head_tilt_angle` (std_msgs/Float32): Current tilt angle for eye coordination

**Services:**
- Uses `get_position` (dynamixel_sdk_custom_interfaces/GetPosition): Read motor positions

### HeadTrackingSystem

Core tracking logic with multiple operating modes:

**States:**
- `INITIALIZING`: Reading initial motor positions
- `IDLE`: No tracking, motors at rest
- `TRACKING`: Actively following faces
- `SCANNING`: Searching for faces with sweep motion
- `MOVING`: Executing smooth movements to target positions

**Features:**
- **Face Selection**: Automatically selects largest/closest face to track
- **Velocity Calculation**: Tracks face movement speed and direction
- **Deadzone Management**: Prevents jittery movements near center
- **Coordinate Movement**: Synchronized pan/tilt for smooth tracking

### PIDController

Advanced PID controller with anti-windup and rate limiting:
- **Proportional**: Immediate response to error
- **Integral**: Eliminates steady-state error with anti-windup
- **Derivative**: Reduces overshoot and oscillation
- **Output Limiting**: Prevents excessive motor commands

## Installation

### Dependencies

- **ROS2**: rclpy, std_msgs, geometry_msgs, sensor_msgs
- **Dynamixel**: dynamixel_sdk_custom_interfaces
- **Coffee Buddy**: coffee_expressions_msgs
- **GUI**: python_qt_binding, PyQt5
- **Math/Vision**: numpy, opencv-python

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd coffee_ws

# Build the package
colcon build --packages-select coffee_head_control

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch head tracking node with GUI
ros2 launch coffee_head_control head_tracking.launch.py

# Launch with custom parameters
ros2 launch coffee_head_control head_tracking.launch.py \
  enable_tracking:=true \
  update_rate:=25.0 \
  baud_rate:=1000000
```

### GUI Controls

The head tracking node launches with an interactive GUI providing:

1. **Tracking Controls**
   - Enable/disable head tracking
   - Reset head to default position
   - Toggle PID smoothing

2. **Communication Settings**
   - Motor update rate (1-100 Hz)
   - Dynamixel baud rate selection
   - Real-time parameter updates

3. **Movement Thresholds**
   - Pan threshold (10-100 pixels)
   - Tilt threshold (5-80 pixels)
   - Adjustable deadzones

4. **Speed Controls**
   - Min/max pan speed (deg/s)
   - Min/max tilt speed (deg/s)
   - Adaptive speed based on distance

5. **PID Tuning**
   - Separate pan and tilt PID parameters
   - Real-time tuning with sliders
   - Smoothing factor adjustment
   - Scanning frequency control

### Manual Control

```bash
# Run the node directly
ros2 run coffee_head_control head_tracking

# Monitor head angles
ros2 topic echo /head_pan_angle
ros2 topic echo /head_tilt_angle

# Monitor face velocities
ros2 topic echo /face_velocity
```

## Configuration

### Motor Settings

The system is configured for specific Dynamixel motor IDs:
- **Pan Motor**: ID 1 (horizontal movement)
- **Tilt Motor**: ID 9 (vertical movement)

**Angle Limits:**
- Pan: 143° to 210° (center: 180°)
- Tilt: 169° to 206° (center: 180°)

### Performance Parameters

**Default Tracking Thresholds:**
- Pan threshold: 80 pixels
- Tilt threshold: 80 pixels
- Update rate: 30 Hz

**PID Default Values:**
- Pan PID: P=0.1, I=0.005, D=0.08
- Tilt PID: P=0.15, I=0.01, D=0.05
- Smoothing factor: 0.8

**Speed Limits:**
- Pan: 1.0-80.0 deg/s
- Tilt: 1.0-15.0 deg/s

## Integration

### Face Detection Integration

Subscribes to face detection data from camera/vision systems:
```json
{
  "timestamp": 1672531200.123,
  "frame_width": 640,
  "frame_height": 480,
  "faces": [
    {
      "x1": 100, "y1": 120, "x2": 200, "y2": 220,
      "center_x": 150, "center_y": 170,
      "confidence": 0.85,
      "id": "person_1"
    }
  ]
}
```

### Eye Tracking Coordination

Publishes head angles for eye tracking coordination:
- Head handles large movements (outside thresholds)
- Eyes handle fine movements (within thresholds)
- Prevents conflicting movements

### Dynamixel Motor Service

Requires `dynamixel_sdk_examples` service for motor communication:
```bash
# Start motor service
ros2 run dynamixel_sdk_examples read_write_node
```

## Behavior Modes

### Tracking Mode

When faces are detected:
1. **Face Selection**: Chooses largest face or closest to center
2. **Error Calculation**: Computes pixel offset from frame center
3. **Threshold Check**: Only moves if error exceeds thresholds
4. **PID Control**: Calculates smooth motor adjustments
5. **Coordinate Movement**: Sends synchronized pan/tilt commands

### Scanning Mode

When no faces detected:
1. **Sine Wave Motion**: Smooth left-right scanning
2. **Configurable Frequency**: Adjustable scan speed
3. **Position Continuity**: Starts from current position
4. **Automatic Transition**: Switches to tracking when face found

### Idle Mode

When tracking disabled:
- Motors remain at current position
- No automatic movements
- Manual reset available

## Troubleshooting

### Motor Issues

```bash
# Check if motors are connected
ros2 service list | grep position

# Test motor service
ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 1}"

# Check motor positions
ros2 topic echo /head_pan_angle
```

### Face Detection Issues

```bash
# Check face detection data
ros2 topic echo /robot/affective_state

# Monitor face velocities
ros2 topic echo /face_velocity

# Check timing (should be recent)
ros2 topic hz /robot/affective_state
```

### GUI Issues

```bash
# Ensure Qt5 is installed
sudo apt install python3-pyqt5

# Check X11 forwarding (if using SSH)
echo $DISPLAY
```

### Performance Issues

1. **Slow Response**: Increase update rate in GUI
2. **Jittery Movement**: Increase smoothing factor
3. **Overshoot**: Reduce PID gains
4. **Missed Faces**: Decrease thresholds

## Advanced Features

### Velocity Tracking

Calculates and displays face movement velocities:
- **Real-time Calculation**: Based on position history
- **Smoothing**: Temporal filtering for stable readings
- **Direction Display**: Cardinal direction indicators
- **Vector Visualization**: GUI displays velocity vectors

### Coordinated Movement

Synchronizes pan and tilt for natural head motion:
- **Simultaneous Commands**: Both motors move together
- **Speed Adaptation**: Faster movement for distant targets
- **Smooth Interpolation**: Sine-wave based acceleration

### State Management

Robust state machine prevents conflicts:
- **Initialization**: Safe startup sequence
- **Movement Coordination**: Prevents simultaneous operations
- **Error Recovery**: Automatic recovery from failures

## Development

### Extending Functionality

Key extension points:
- **New Tracking Algorithms**: Modify `calculate_coordinated_movement()`
- **Custom Behaviors**: Add new states to `HeadState` enum
- **GUI Enhancements**: Extend `HeadTrackingUI` class
- **Motor Integration**: Update motor IDs and limits

### Parameter Tuning

For optimal performance:
1. **Tune PID Values**: Start with P, then add D, finally I
2. **Adjust Thresholds**: Balance responsiveness vs stability
3. **Set Speed Limits**: Consider mechanical constraints
4. **Test Scanning**: Ensure smooth operation without faces

## License

TODO: License declaration 