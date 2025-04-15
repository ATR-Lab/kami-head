# Coffee Face Package

This package provides animated eyes for the Coffee Robot. It uses PyGame to render animated eyes that track faces.

## Components

- **coffee_eyes**: A ROS2 node that displays animated eyes that track faces using camera input

## Dependencies

- ROS2
- Python 3
- PyGame
- numpy

## Building

To build the package:

```bash
cd coffee_ws
colcon build --packages-select coffee_face
source install/setup.bash
```

## Running

To run the coffee_eyes node:

```bash
ros2 run coffee_face coffee_eyes
```

Or using the launch file:

```bash
ros2 launch coffee_face coffee_eyes.launch.py
```

## Configuration

The node can be configured with the following parameters:

### Display Parameters
- `screen_width`: Width of the eye display window (default: 1080)
- `screen_height`: Height of the eye display window (default: 600)
- `movement_speed`: Speed factor for eye movements (default: 1.0)

### Face Tracking Parameters
- `face_tracking_enabled`: Enable/disable face tracking (default: true)
- `frame_width`: Camera frame width in pixels (default: 640)
- `frame_height`: Camera frame height in pixels (default: 480)
- `invert_x`: Invert x-axis mapping (default: false)
- `invert_y`: Invert y-axis mapping (default: false)
- `eye_range`: Maximum range for eye movement (-eye_range to +eye_range) (default: 3.0)

Example:

```bash
ros2 run coffee_face coffee_eyes --ros-args -p screen_width:=800 -p screen_height:=480 -p face_tracking_enabled:=true
```

## Features

- Smooth eye animations with bezier curves
- Automatic blinking
- Configurable eye appearance
- Face tracking integration

## Face Tracking

The coffee_eyes node subscribes to the following topics:

- `face_detection_data`: JSON string message containing face positions
- `face_velocity`: Vector3 message containing face velocity information

When faces are detected, the eyes will track the largest/closest face. When no faces are detected, the eyes remain stationary.

## Customizing Expressions

The eye expressions are loaded from `resource/expressions.json`. This file contains predefined eye shapes and animation parameters. You can add your own expressions by adding new entries to this file.

## Troubleshooting

If you encounter the error "No available video device" when running headless, you may need to set up a virtual frame buffer:

```bash
sudo apt-get install xvfb
Xvfb :1 -screen 0 1080x600x24 &
export DISPLAY=:1
```

Then run the node as usual.