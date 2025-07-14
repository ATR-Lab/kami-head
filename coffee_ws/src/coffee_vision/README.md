# Coffee Vision Package

A ROS2 package providing computer vision capabilities for camera capture and face detection in the Coffee Buddy robot system.

## Overview

The `coffee_vision` package is a focused computer vision module that handles:
- Camera capture and image streaming
- Real-time face detection using OpenCV DNN
- Face position tracking and data publishing

This package follows the single responsibility principle, focusing only on low-level computer vision tasks. Face recognition and memory management are handled by separate packages.

## Architecture

```
┌─────────────────┐    /camera/image_raw    ┌──────────────────────┐
│   Camera Node   ├────────────────────────►│ Face Detection Node  │
│                 │                         │                      │
├─────────────────┤                         ├──────────────────────┤
│ - Camera setup  │                         │ - OpenCV DNN         │
│ - Frame capture │                         │ - Face detection     │
│ - Image publish │                         │ - Position tracking  │
└─────────────────┘                         └──────────────────────┘
                                                        │
                                                        ▼
                                            /vision/face_detection_data
                                            /vision/face_position
```

## Nodes

### Camera Node (`camera_node`)

Handles camera initialization, frame capture, and image publishing.

**Publishers:**
- `/camera/image_raw` (sensor_msgs/Image): Raw camera frames
- `/camera/status` (std_msgs/String): Camera status

**Parameters:**
- `camera_index` (int, default: 0): Camera device index
- `frame_width` (int, default: 640): Frame width in pixels
- `frame_height` (int, default: 480): Frame height in pixels
- `target_fps` (int, default: 30): Target frames per second

### Face Detection Node (`face_detection_node`)

Subscribes to camera images and performs face detection using OpenCV DNN.

**Subscribers:**
- `/camera/image_raw` (sensor_msgs/Image): Camera frames

**Publishers:**
- `/vision/face_detection_data` (std_msgs/String): JSON-formatted face detection results
- `/vision/face_position` (geometry_msgs/Vector3): Position of largest detected face
- `/vision/face_detection_status` (std_msgs/String): Detection status

**Parameters:**
- `confidence_threshold` (double, default: 0.5): Minimum confidence for face detection
- `nms_threshold` (double, default: 0.4): Non-maximum suppression threshold
- `enable_smoothing` (bool, default: true): Enable temporal smoothing of detections

## Installation

### Dependencies

The package requires the following dependencies:
- OpenCV with DNN support
- NumPy
- Standard ROS2 packages (rclpy, sensor_msgs, etc.)

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd coffee_ws

# Build the package
colcon build --packages-select coffee_vision

# Source the workspace
source install/setup.bash
```

## Usage

### Launch All Vision Nodes

```bash
# Launch camera and face detection nodes together
ros2 launch coffee_vision vision_nodes.launch.py

# Launch with custom parameters
ros2 launch coffee_vision vision_nodes.launch.py camera_index:=1 confidence_threshold:=0.7
```

### Run Individual Nodes

```bash
# Camera node only
ros2 run coffee_vision camera_node

# Face detection node only (requires camera feed)
ros2 run coffee_vision face_detection_node
```

### Monitor Topics

```bash
# View camera feed
ros2 topic echo /camera/image_raw

# Monitor face detection results
ros2 topic echo /vision/face_detection_data

# Watch face positions
ros2 topic echo /vision/face_position
```

## Data Formats

### Face Detection Data

Published on `/vision/face_detection_data` as JSON:

```json
{
  "timestamp": 1672531200,
  "faces": [
    {
      "x1": 100,
      "y1": 120,
      "x2": 200,
      "y2": 220,
      "center_x": 150,
      "center_y": 170,
      "confidence": 0.85
    }
  ],
  "count": 1
}
```

### Face Position

Published on `/vision/face_position` as Vector3:
- `x`: Center X coordinate of largest face
- `y`: Center Y coordinate of largest face  
- `z`: Confidence score

## Configuration

### Camera Settings

The camera node automatically configures optimal settings but can be customized:

```bash
ros2 run coffee_vision camera_node --ros-args \
  -p camera_index:=0 \
  -p frame_width:=1280 \
  -p frame_height:=720 \
  -p target_fps:=30
```

### Face Detection Tuning

Adjust detection sensitivity and performance:

```bash
ros2 run coffee_vision face_detection_node --ros-args \
  -p confidence_threshold:=0.7 \
  -p nms_threshold:=0.3 \
  -p enable_smoothing:=true
```

## Models

The face detection node automatically downloads OpenCV's pre-trained face detection models:
- `opencv_face_detector_uint8.pb`
- `opencv_face_detector.pbtxt`

Models are stored in `coffee_vision/models/` directory.

## Performance

### GPU Acceleration

The face detection node automatically attempts to use CUDA acceleration if available, falling back to CPU processing otherwise.

### Optimization Tips

1. **Lower resolution** for better performance: Set smaller `frame_width` and `frame_height`
2. **Adjust confidence threshold**: Higher values reduce false positives but may miss faces
3. **Disable smoothing** for faster processing: Set `enable_smoothing:=false`

## Integration

This package is designed to integrate with other Coffee Buddy packages:

- **coffee_recognition**: Subscribes to face detection data for recognition tasks
- **coffee_head_control**: Uses face positions for head tracking
- **coffee_expressions**: Responds to face detection for emotional expressions

## Troubleshooting

### Camera Issues

```bash
# List available cameras
ls /dev/video*

# Test camera directly
ros2 run coffee_vision camera_node --ros-args -p camera_index:=1
```

### Detection Issues

```bash
# Lower confidence threshold for more detections
ros2 run coffee_vision face_detection_node --ros-args -p confidence_threshold:=0.3

# Check model download
ls coffee_ws/src/coffee_vision/coffee_vision/models/
```

## Development

### Adding New Detection Models

1. Place model files in `coffee_vision/models/`
2. Update `initialize_face_detector()` in `face_detection_node.py`
3. Test with various lighting conditions

### Extending Functionality

This package is designed to be minimal and focused. For additional computer vision features:
- Create separate specialized packages
- Use the same topic interfaces for compatibility
- Follow the modular architecture pattern

## License

TODO: License declaration 