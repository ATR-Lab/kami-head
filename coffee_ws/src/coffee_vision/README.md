# Coffee Vision Package

A ROS2 package providing comprehensive computer vision capabilities including camera capture, face detection, and coordinate transformation for the Coffee Buddy robot system.

## Overview

The `coffee_vision` package provides a modular computer vision solution that combines:
- Camera capture and streaming
- Real-time face detection using OpenCV DNN
- Face position tracking and coordinate transformation
- Multi-threaded performance optimization
- Camera diagnostics and quality controls
- Configurable parameters via ROS2 parameter system

This package features a modular architecture with separate components for face detection, coordinate transformation, and camera management, all coordinated by the main `camera_node`.

**Modular Benefits:**
- **Testability**: Face detection and coordinate transformation can be tested independently
- **Reusability**: Components can be used by other packages
- **Maintainability**: Clear separation of concerns makes code easier to understand and modify
- **Configurability**: ROS2 parameters allow runtime tuning without code changes

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Camera Node                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  FrameGrabber   │  │  FaceDetector   │  │ CoordinateUtils │  │
│  │ - Multi-thread  │  │ - OpenCV DNN    │  │ - Transform     │  │
│  │ - Frame buffer  │  │ - Face tracking │  │ - Eye coords    │  │
│  │ - Rate control  │  │ - Smoothing     │  │ - Mapping       │  │
│  │ - ROS publish   │  │ - Visualization │  │ - Validation    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│           │                     │                     │         │
│           └─────────────────────┼─────────────────────┘         │
│                                 │                               │
└─────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                        ROS2 Publishers:
                        • /coffee_bot/camera/image_raw (sensor_msgs/Image)
                        • /face_detection_data (std_msgs/String)
                        • /vision/face_position (geometry_msgs/Point)
                        • /vision/face_position_v2 (std_msgs/String)
                        • /face_images (sensor_msgs/Image)

                        ROS2 Parameters:
                        • face_confidence_threshold (float)
                        • face_smoothing_factor (float)
                        • eye_range (float)
                        • eye_sensitivity (float)
                        • invert_x/invert_y (bool)
```

## Components

### Camera Node (`camera_node`)

The main ROS2 node that coordinates camera capture, face detection, and coordinate transformation using a modular architecture.

**Key Features:**
- **Multi-threaded Architecture**: Separate threads for capture, processing, and publishing
- **Modular Design**: Separate modules for face detection and coordinate transformation
- **Configurable Parameters**: ROS2 parameters for runtime configuration
- **Performance Optimization**: Adaptive frame rates and quality controls
- **ROS Interface**: Comprehensive ROS topic interface for external control

**Publishers:**
- `/coffee_bot/camera/image_raw` (sensor_msgs/Image): Raw camera frames
- `/face_detection_data` (std_msgs/String): JSON-formatted face detection results
- `/vision/face_position` (geometry_msgs/Point): Eye-coordinate transformed face position
- `/vision/face_position_v2` (std_msgs/String): Extended face position data
- `/face_images` (sensor_msgs/Image): Extracted face image regions

### Face Detector (`face_detection.py`)

Standalone face detection module using OpenCV DNN with temporal smoothing.

**Features:**
- **OpenCV DNN Backend**: High-performance face detection
- **Automatic Model Download**: Downloads required models on first run
- **Temporal Smoothing**: Reduces detection flickering
- **CUDA Support**: Automatic GPU acceleration when available
- **Debug Visualization**: Face overlay rendering for debugging

**Key Methods:**
- `detect_faces()`: Core face detection functionality
- `smooth_detections()`: Temporal smoothing algorithm
- `draw_debug_overlay()`: Visualization for debugging

### Coordinate Utilities (`coordinate_utils.py`)

Pure utility functions for coordinate system transformations.

**Features:**
- **Camera to Eye Coordinates**: Transform pixel coordinates to robot eye movement
- **Configurable Sensitivity**: Adjustable sensitivity and inversion parameters
- **Input Validation**: Robust parameter validation
- **No Dependencies**: Pure math functions with no ROS dependencies

**Key Functions:**
- `transform_camera_to_eye_coords()`: Main coordinate transformation
- Input validation and range clamping

## Installation

### Dependencies

The package requires:
- **Qt5/PyQt5**: For GUI interface (`python_qt_binding`)
- **OpenCV**: With DNN support for face detection
- **NumPy**: For numerical operations
- **Standard ROS2**: rclpy, sensor_msgs, geometry_msgs, cv_bridge

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

### Running the Camera Node

The camera node can be run with default settings or custom parameters:

```bash
# Run with default settings
ros2 run coffee_vision camera_node

# Run with custom parameters
ros2 run coffee_vision camera_node --ros-args \
  -p face_confidence_threshold:=0.7 \
  -p eye_range:=1.5 \
  -p eye_sensitivity:=2.0

# The node will automatically:
# - Load configuration parameters
# - Scan for available cameras
# - Start publishing camera data
# - Begin face detection
```

### Configuration Parameters

The node supports the following ROS2 parameters:

```bash
# Face detection parameters
ros2 param set /camera_node face_confidence_threshold 0.7
ros2 param set /camera_node face_smoothing_factor 0.6

# Eye movement parameters
ros2 param set /camera_node eye_range 1.5
ros2 param set /camera_node eye_sensitivity 2.0
ros2 param set /camera_node invert_x true

# View current parameters
ros2 param list /camera_node
ros2 param get /camera_node face_confidence_threshold
```

### External Control

The camera node provides ROS topic interfaces for external control:

```bash
# Camera control topics
ros2 topic pub /coffee_bot/camera/cmd/select std_msgs/Int32 "data: 1"
ros2 topic pub /coffee_bot/camera/cmd/quality std_msgs/Bool "data: true"
ros2 topic pub /coffee_bot/camera/cmd/face_detection std_msgs/Bool "data: false"

# Status and diagnostics
ros2 topic echo /coffee_bot/camera/status/info
ros2 topic pub /coffee_bot/camera/cmd/diagnostics std_msgs/String "data: 'get'"
```

### ROS2 Topics

Monitor the published data:

```bash
# View camera frames
ros2 topic echo /coffee_bot/camera/image_raw

# Monitor face detection results (JSON format)
ros2 topic echo /face_detection_data

# Watch face positions (eye coordinates)
ros2 topic echo /vision/face_position

# Extended face position data
ros2 topic echo /vision/face_position_v2

# Face image extractions
ros2 topic echo /face_images
```

## Configuration

### ROS2 Parameters

The node supports comprehensive configuration through ROS2 parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `face_confidence_threshold` | float | 0.5 | Minimum confidence for face detection (0.0-1.0) |
| `face_smoothing_factor` | float | 0.4 | Temporal smoothing factor (0.0-1.0, higher = more smoothing) |
| `eye_range` | float | 1.0 | Maximum eye movement range (-eye_range to +eye_range) |
| `eye_sensitivity` | float | 1.5 | Eye movement sensitivity multiplier |
| `invert_x` | bool | false | Invert X axis for eye movement |
| `invert_y` | bool | false | Invert Y axis for eye movement |

### Camera Settings

The node automatically detects and configures cameras with optimal settings:
- **Resolution**: 640x480 (standard) or 1280x720 (high quality)
- **Frame Rate**: 30 FPS (standard) or 24 FPS (high quality)
- **Backend**: Automatically selects V4L2, GStreamer, or OpenCV backends

### Face Detection Configuration

Built-in face detection with configurable parameters:
- **Model**: OpenCV DNN face detector (auto-downloaded)
- **Confidence Threshold**: Configurable via `face_confidence_threshold` parameter
- **Detection Rate**: Adaptive (3-6 FPS) based on performance
- **Smoothing**: Configurable temporal smoothing via `face_smoothing_factor`
- **Coordinate System**: Transforms to robot eye coordinates using configurable sensitivity

### Performance Optimization

The node includes several performance optimizations:
- **Adaptive Frame Skipping**: Reduces face detection frequency under load
- **Multi-threading**: Separate threads for capture, processing, and publishing
- **Frame Buffering**: Manages memory efficiently with frame locks
- **Rate Limiting**: Controls UI updates and ROS publishing rates

## Data Formats

### Face Detection Data (`/face_detection_data`)

Published as JSON string:

```json
{
  "timestamp": 1672531200.123,
  "frame_width": 640,
  "frame_height": 480,
  "faces": [
    {
      "x1": 100,
      "y1": 120,
      "x2": 200,
      "y2": 220,
      "center_x": 150,
      "center_y": 170,
      "confidence": 0.85,
      "id": "Unknown"
    }
  ]
}
```

### Face Position (`/vision/face_position`)

Published as geometry_msgs/Point with eye coordinates:
- `x`: Horizontal eye position (-1.0 to 1.0)
- `y`: Vertical eye position (-1.0 to 1.0)
- `z`: Reserved (usually 0.0)

## Models

The face detection system automatically downloads required models:
- **opencv_face_detector_uint8.pb**: DNN model weights
- **opencv_face_detector.pbtxt**: Model configuration

Models are stored in: `coffee_vision/models/` directory

**GPU Acceleration**: Automatically uses CUDA if available, falls back to CPU

## Integration

This package integrates with other Coffee Buddy components:

- **coffee_head_control**: Subscribes to `/vision/face_position` for head tracking
- **coffee_recognition**: Uses `/face_detection_data` and `/face_images` for recognition
- **coffee_expressions**: Responds to face detection for emotional expressions
- **coffee_face**: May use face position data for eye animations

## Performance Notes

### Threading Architecture

The node uses a sophisticated multi-threading approach:
1. **Capture Thread**: Dedicated camera frame capture
2. **Process Thread**: Face detection and image processing
3. **Publish Thread**: ROS message publishing at controlled rates
4. **UI Thread**: Qt GUI updates and user interaction

### Adaptive Performance

- **Dynamic Frame Skipping**: Adjusts detection frequency based on processing time
- **Quality Scaling**: Automatic resolution adjustment for performance
- **Rate Limiting**: Prevents overwhelming ROS topics with high-frequency data

### Memory Management

- **Frame Buffering**: Efficient shared memory for multi-threading
- **GPU Memory**: Automatic cleanup and management for DNN models
- **Resource Cleanup**: Proper camera release on shutdown

## Troubleshooting

### Camera Issues

```bash
# Check available cameras
ls /dev/video*

# View camera diagnostics in GUI
# Click "Camera Diagnostics" button

# Test different camera indices
# Use GUI dropdown to select cameras
```

### Performance Issues

1. **Low Frame Rate**: Try disabling face detection or reducing quality
2. **High CPU Usage**: Face detection is CPU-intensive; consider GPU acceleration
3. **Memory Issues**: The node uses significant memory for image processing

### GUI Issues

```bash
# Ensure Qt5 is properly installed
sudo apt install python3-pyqt5

# Check X11 forwarding if using SSH
echo $DISPLAY
```

### Face Detection Issues

1. **No Faces Detected**: 
   - Check lighting conditions
   - Adjust confidence threshold in code
   - Ensure camera is properly focused

2. **Model Download Fails**:
   - Check internet connection
   - Manually download models to `coffee_vision/models/`

3. **Poor Detection Accuracy**:
   - Increase resolution (use High Quality mode)
   - Improve lighting conditions
   - Ensure face is clearly visible

## Development

### Code Structure

- **CameraNode** (`camera_node.py`): Main ROS2 node with parameter management
- **FrameGrabber** (`camera_node.py`): Core camera capture and processing logic
- **FaceDetector** (`face_detection.py`): Standalone face detection module
- **CoordinateUtils** (`coordinate_utils.py`): Pure coordinate transformation functions

### Extending Functionality

To add new features:
1. **New Publishers**: Add to FrameGrabber class in `camera_node.py`
2. **Face Detection**: Extend FaceDetector class in `face_detection.py`
3. **Coordinate Systems**: Add functions to `coordinate_utils.py`
4. **Parameters**: Add new ROS2 parameters in CameraNode
5. **Processing Pipeline**: Modify `_process_loop()` thread in FrameGrabber

### Performance Tuning

Key configurable parameters for optimization:
- `face_confidence_threshold`: Detection sensitivity (lower = more detections)
- `face_smoothing_factor`: Temporal smoothing strength (higher = smoother)
- `eye_sensitivity`: Eye movement responsiveness
- `eye_range`: Maximum eye movement range

Built-in adaptive parameters:
- `min_detection_interval`: Face detection frequency (adaptive)
- `detection_skip_frames`: Frame skipping for performance (adaptive)

## Testing UI Separation

### Separated Camera UI

The `camera_viewer_test` node provides a **full-featured camera UI** that communicates with `camera_node` via ROS topics, enabling complete UI separation from camera processing.

```bash
# Run both camera processing and separated UI
ros2 launch coffee_vision camera_viewer_test.launch.py

# Or run them separately:
# Terminal 1: Run camera node (headless processing)
ros2 run coffee_vision camera_node

# Terminal 2: Run separated UI
ros2 run coffee_vision camera_viewer_test
```

**Features:**
- **Complete camera control**: Camera selection, quality settings, face detection toggle
- **Real-time video display**: Full video streaming with performance metrics
- **Diagnostics**: System and camera diagnostic information
- **ROS communication**: All control via standardized `/coffee_bot/` topics

**ROS Topics:**
- Control: `/coffee_bot/camera/cmd/{select,quality,face_detection,refresh}`
- Status: `/coffee_bot/camera/status/{info,available,diagnostics}`
- Video: `/coffee_bot/camera/image_raw`

**Benefits:**
- **Headless operation**: Camera node can run without GUI dependencies
- **Remote monitoring**: UI can run on different machine than camera processing
- **Scalability**: Multiple UIs can monitor same camera node
- **Development flexibility**: UI and camera processing can be developed independently

**Performance Testing:**
The UI displays real-time performance metrics to evaluate ROS transport vs integrated approach:
- **Display FPS**: UI rendering smoothness
- **Frame Interval**: Time between received frames  
- **Transport Latency**: ROS communication overhead
- **Connection Status**: Real-time connection monitoring

## Known Limitations

1. **Single Camera**: Currently supports one camera at a time
2. **Memory Usage**: High memory consumption due to image processing
3. **CPU Intensive**: Face detection requires significant computational resources
4. **Synchronous Model Download**: Face detection models downloaded synchronously at startup

## Future Improvements

Potential enhancements:
- Multiple camera support
- Asynchronous model downloading
- Advanced face tracking algorithms
- Custom face detection models
- Camera reconnection and recovery
- WebRTC streaming capabilities

## License

TODO: License declaration 