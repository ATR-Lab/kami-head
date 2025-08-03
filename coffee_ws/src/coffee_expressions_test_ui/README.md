# Coffee Expressions Test UI

A ROS2 package providing a Qt-based graphical user interface for testing and debugging coffee robot expressions. This tool allows developers and operators to manually trigger different emotional expressions and monitor the robot's affective state system.

## Features

- **Expression Testing**: Select from predefined expressions (Happy, Angry, Loving, Sad, Surprised)
- **Trigger Source Control**: Specify the source of expression triggers (vision, audio, event, mock)
- **Real-time Adjustment**: Live sliders for intensity and confidence levels (0-100%)
- **Gaze Control**: Interactive gaze target positioning with visual preview
- **Publishing Modes**: Manual publishing or real-time automatic publishing
- **Visual Feedback**: Live preview widget showing gaze target position
- **Status Monitoring**: Real-time publishing status and timestamps

## Requirements

- ROS2 (Jazzy or compatible)
- Python 3
- python_qt_binding (included with ROS2)
- coffee_expressions_msgs (interface package)

## Installation

1. Ensure the package is in your ROS2 workspace:
   ```bash
   cd /path/to/your/workspace/src
   # coffee_expressions_test_ui/
   ```

2. Build the package:
   ```bash
   cd /path/to/your/workspace
   # Build interface package first
   colcon build --packages-select coffee_expressions_msgs
   # Build test UI
   colcon build --packages-select coffee_expressions_test_ui
   ```

3. Source the workspace:
   ```bash
   source /path/to/your/workspace/install/setup.bash
   ```

## Usage

### Launch the Test UI

```bash
ros2 run coffee_expressions_test_ui expressions_test_ui
```

### Interface Controls

#### Expression Selection
- **Expression Dropdown**: Choose from available emotional expressions
  - Happy
  - Angry  
  - Loving
  - Sad
  - Surprised

#### Trigger Configuration
- **Trigger Source**: Radio buttons to specify the source of the expression
  - `vision`: Triggered by computer vision system
  - `audio`: Triggered by audio processing
  - `event`: Triggered by system events
  - `mock`: Manual testing trigger (default)

#### Intensity & Confidence
- **Intensity Slider**: Control expression intensity (0-100%)
- **Confidence Slider**: Set confidence level of the expression (0-100%)

#### Gaze Control
- **Gaze X/Y Sliders**: Set gaze target coordinates (-100 to +100)
- **Gaze Preview**: Visual widget showing current gaze target position
- **Interactive Preview**: Click and drag in the preview to set gaze target

#### Publishing Options
- **Is Idle Checkbox**: Mark the robot as idle/active
- **Real-time Publishing**: Automatically publish changes as you adjust controls
- **Publish Button**: Manually publish current settings when real-time is disabled

### ROS2 Integration

#### Published Topics

- **`/robot/affective_state`** (`coffee_expressions_msgs/AffectiveState`)
  - Complete affective state information including expression, intensity, confidence, gaze target, and trigger source

#### Message Structure

The published `AffectiveState` message contains:
```
string expression          # Selected expression name
float32 intensity         # Expression intensity (0.0-1.0)
float32 confidence        # Confidence level (0.0-1.0)
string trigger_source     # Source of the trigger
geometry_msgs/Point gaze_target  # Gaze target coordinates
bool is_idle             # Robot idle state
```

### Testing Workflows

#### Expression Development
1. Launch the test UI
2. Select "Real-time Publishing" for immediate feedback
3. Choose an expression and adjust intensity/confidence
4. Monitor the robot's response in real-time

#### Gaze System Testing
1. Use the gaze sliders or click in the preview widget
2. Observe robot head/eye movement responses
3. Test different gaze positions and combinations

#### Integration Testing
1. Set trigger source to match your system component
2. Test different expressions with varying intensities
3. Verify message reception in your expression processing nodes

## Development

### Adding New Expressions

To add new expressions, modify the expression list in `expressions_test_ui.py`:

```python
self.expression_combo.addItems(['Happy', 'Angry', 'Loving', 'Sad', 'Surprised', 'NewExpression'])
```

### Customizing Gaze Range

Adjust the slider ranges in the gaze control section:

```python
slider.setRange(-100, 100)  # Modify range as needed
```

## Troubleshooting

### UI Not Starting
- Ensure `python_qt_binding` is available (comes with ROS2)
- Check that all dependencies are built and sourced
- Verify display is available for GUI applications

### Messages Not Publishing
- Check topic name matches your expression processing nodes
- Verify `coffee_expressions_msgs` package is built and sourced
- Monitor with `ros2 topic echo /robot/affective_state`

### Gaze Preview Not Responding
- Ensure mouse tracking is enabled in the widget
- Check coordinate system matches your robot's expectations

## License

Apache License 2.0
