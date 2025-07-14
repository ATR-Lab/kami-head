# Coffee Vision Package

A ROS2 package providing comprehensive computer vision capabilities including camera capture, face detection, and GUI visualization for the Coffee Buddy robot system.

## Overview

The `coffee_vision` package provides an integrated computer vision solution that combines:
- Camera capture and streaming
- Real-time face detection using OpenCV DNN
- Interactive Qt-based GUI for camera control
- Face position tracking and coordinate transformation
- Multi-threaded performance optimization
- Camera diagnostics and quality controls

This package contains a single, comprehensive `camera_node` that handles all computer vision tasks with an integrated GUI interface.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Camera Node                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │   Qt GUI        │  │  FrameGrabber   │  │  Face Detection │  │
│  │ - Camera select │  │ - Multi-thread  │  │ - OpenCV DNN    │  │
│  │ - Quality ctrl  │  │ - Frame buffer  │  │ - Face tracking │  │
│  │ - Diagnostics   │  │ - Rate control  │  │ - Smoothing     │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                          ROS2 Publishers:
                          • /camera_frame (sensor_msgs/Image)
                          • /face_detection_data (std_msgs/String)
                          • /vision/face_position (geometry_msgs/Point)
                          • /vision/face_position_v2 (std_msgs/String)
                          • /face_images (sensor_msgs/Image)
```

## Components

### Camera Node (`camera_node`)

A comprehensive ROS2 node that provides camera capture, face detection, and GUI control in a single integrated package.

**Key Features:**
- **Multi-threaded Architecture**: Separate threads for capture, processing, publishing, and UI
- **Interactive GUI**: Qt-based interface for camera selection and control
- **Built-in Face Detection**: OpenCV DNN-based face detection with temporal smoothing
- **Performance Optimization**: Adaptive frame rates and quality controls
- **Eye Coordinate Transformation**: Converts face positions to robot eye coordinates

**Publishers:**
- `/camera_frame` (sensor_msgs/Image): Raw camera frames
- `/face_detection_data` (std_msgs/String): JSON-formatted face detection results
- `/vision/face_position` (geometry_msgs/Point): Eye-coordinate transformed face position
- `/vision/face_position_v2` (std_msgs/String): Extended face position data
- `/face_images` (sensor_msgs/Image): Extracted face image regions

**GUI Controls:**
- Camera device selection and refresh
- Quality toggle (480p/720p)
- Face detection enable/disable
- Camera diagnostics display
- Real-time video preview with face overlays

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

The camera node provides both command-line and GUI interfaces:

```bash
# Run with GUI (default)
ros2 run coffee_vision camera_node

# The node will automatically:
# - Scan for available cameras
# - Launch the Qt GUI
# - Start publishing camera data
# - Begin face detection
```

### GUI Interface

The camera node launches with an interactive GUI that provides:

1. **Camera Selection**: Dropdown to choose between detected cameras
2. **Quality Control**: Toggle between standard (640x480) and high quality (1280x720)
3. **Face Detection**: Enable/disable real-time face detection
4. **Diagnostics**: View system information and camera capabilities
5. **Live Preview**: Real-time camera feed with face detection overlays

### ROS2 Topics

Monitor the published data:

```bash
# View camera frames
ros2 topic echo /camera_frame

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

### Camera Settings

The node automatically detects and configures cameras with optimal settings:
- **Resolution**: 640x480 (standard) or 1280x720 (high quality)
- **Frame Rate**: 30 FPS (standard) or 24 FPS (high quality)
- **Backend**: Automatically selects V4L2, GStreamer, or OpenCV backends

### Face Detection Parameters

Built-in face detection with the following characteristics:
- **Model**: OpenCV DNN face detector (auto-downloaded)
- **Confidence Threshold**: 0.5 (configurable in code)
- **Detection Rate**: Adaptive (3-6 FPS) based on performance
- **Smoothing**: Temporal smoothing to reduce detection jitter
- **Coordinate System**: Transforms to robot eye coordinates (-1.0 to 1.0 range)

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

- **FrameGrabber**: Core camera capture and processing logic
- **CameraViewer**: Qt GUI interface and controls  
- **CameraNode**: ROS2 node wrapper and coordination
- **Face Detection**: OpenCV DNN integration with smoothing

### Extending Functionality

To add new features:
1. **New Publishers**: Add to FrameGrabber class
2. **GUI Controls**: Extend CameraViewer interface
3. **Detection Models**: Update `init_face_detector()` method
4. **Processing Pipeline**: Modify `_process_loop()` thread

### Performance Tuning

Key parameters for optimization:
- `min_detection_interval`: Face detection frequency
- `detection_skip_frames`: Frame skipping for performance
- `smoothing_factor`: Temporal smoothing strength
- `face_confidence_threshold`: Detection sensitivity

## Known Limitations

1. **Single Camera**: Currently supports one camera at a time
2. **Qt Dependency**: Requires GUI environment (not headless friendly)
3. **Memory Usage**: High memory consumption due to image processing
4. **CPU Intensive**: Face detection requires significant computational resources

## Future Improvements

Potential enhancements:
- Headless mode without GUI
- Multiple camera support
- Advanced face tracking algorithms
- Custom face detection models
- WebRTC streaming capabilities

## License

TODO: License declaration 