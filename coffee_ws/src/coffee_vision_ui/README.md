# Coffee Vision UI

A separated user interface package for coffee vision camera control and monitoring. This package provides a complete camera control interface that communicates with the `camera_node` via ROS topics, enabling headless camera operation and flexible deployment scenarios.

## Overview

The `coffee_vision_ui` package implements a **separated architecture** where camera processing and user interface run as independent ROS nodes. This enables:

- **Headless operation**: Camera processing can run without GUI dependencies
- **Remote monitoring**: UI can run on different machines than camera processing
- **Multiple interfaces**: Several UI instances can monitor the same camera
- **Development flexibility**: UI and camera processing can be developed independently

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Coffee Vision UI                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │   Control Panel │  │  Camera Display │  │  ROS Interface  │  │
│  │ - Camera select │  │ - Video stream  │  │ - Frame receiver│  │
│  │ - Quality ctrl  │  │ - Performance   │  │ - Camera ctrl   │  │
│  │ - Face detection│  │ - Connection    │  │ - Status monitor│  │
│  │ - Diagnostics   │  │   status        │  │                 │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                           ROS2 Communication
                          /coffee_bot/ namespace
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Camera Node (coffee_vision)                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  FrameGrabber   │  │  Face Detection │  │  ROS Publishers │  │
│  │ - Camera capture│  │ - OpenCV DNN    │  │ - Video stream  │  │
│  │ - Multi-thread  │  │ - Face tracking │  │ - Status info   │  │
│  │ - Performance   │  │ - Smoothing     │  │ - Control resp  │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Package Components

### Core Modules

- **`camera_ui_node.py`**: Main application node that coordinates all components
- **`ros_interface/`**: ROS communication layer
  - `camera_controller.py`: Handles camera control commands and status
  - `frame_receiver.py`: Processes video frames with performance monitoring
- **`widgets/`**: Reusable Qt UI components
  - `camera_display.py`: Video display with performance metrics
  - `control_panel.py`: Camera controls interface

### Launch Files

- **`camera_ui.launch.py`**: Launch UI only (connects to existing camera_node)
- **`full_system.launch.py`**: Launch both camera processing and UI

## Installation

### Dependencies

The package requires the following ROS2 packages:
- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `python_qt_binding`

And Python packages:
- `opencv-python`
- `numpy`
- `PyQt5`

### Building

```bash
# Navigate to your ROS2 workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select coffee_vision_ui

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

1. **Launch camera processing** (from coffee_vision package):
```bash
ros2 run coffee_vision camera_node
```

2. **Launch separated UI**:
```bash
ros2 run coffee_vision_ui camera_ui
```

### Launch File Options

#### UI Only
```bash
# Connect to existing camera_node
ros2 launch coffee_vision_ui camera_ui.launch.py

# With custom parameters
ros2 launch coffee_vision_ui camera_ui.launch.py robot_namespace:=kitchen_bot
```

#### Full System
```bash
# Launch both camera processing and UI
ros2 launch coffee_vision_ui full_system.launch.py

# With custom camera
ros2 launch coffee_vision_ui full_system.launch.py camera_index:=1
```

### Parameterization

#### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_namespace` | `coffee_bot` | ROS topic namespace |
| `camera_index` | `0` | Camera device index |
| `ui_node_name` | `camera_ui` | Name for UI node |
| `log_level` | `info` | Logging level |

#### Example with Custom Parameters
```bash
ros2 launch coffee_vision_ui full_system.launch.py \
    camera_index:=1 \
    robot_namespace:=kitchen_robot \
    log_level:=debug
```

## ROS Interface

### Published Topics (UI → Camera)

| Topic | Type | Description |
|-------|------|-------------|
| `/coffee_bot/camera/cmd/select` | `std_msgs/Int32` | Camera selection command |
| `/coffee_bot/camera/cmd/quality` | `std_msgs/Bool` | Quality toggle (high/low) |
| `/coffee_bot/camera/cmd/face_detection` | `std_msgs/Bool` | Face detection toggle |
| `/coffee_bot/camera/cmd/refresh` | `std_msgs/String` | Camera refresh request |

### Subscribed Topics (Camera → UI)

| Topic | Type | Description |
|-------|------|-------------|
| `/coffee_bot/camera/image_raw` | `sensor_msgs/Image` | Video stream |
| `/coffee_bot/camera/status/info` | `std_msgs/String` | Camera status updates |
| `/coffee_bot/camera/status/available` | `std_msgs/String` | Available cameras (JSON) |
| `/coffee_bot/camera/status/diagnostics` | `std_msgs/String` | Diagnostic information |

### Message Formats

#### Available Cameras (JSON)
```json
[
  {"index": 0, "name": "Camera 0 (/dev/video0)"},
  {"index": 1, "name": "Camera 1 (/dev/video1)"}
]
```

## Features

### Camera Control
- **Device Selection**: Dropdown list of available cameras
- **Quality Control**: Toggle between 480p and 1080p
- **Face Detection**: Enable/disable real-time face detection
- **Refresh**: Scan for new camera devices

### Video Display
- **Real-time Streaming**: Live video with face detection overlays
- **Performance Metrics**: FPS, latency, frame count display
- **Connection Monitoring**: Visual connection status indication
- **Auto-scaling**: Video automatically fits display area

### Diagnostics
- **System Information**: Camera devices, OpenCV version
- **Performance Stats**: Detailed timing and throughput metrics
- **Connection Status**: Real-time ROS communication monitoring

## Comparison with Integrated Approach

| Aspect | Integrated (camera_node) | Separated (coffee_vision_ui) |
|--------|--------------------------|------------------------------|
| **Deployment** | Single process | Distributed processes |
| **Latency** | ~1-2ms (direct) | ~5-15ms (ROS transport) |
| **Resource Usage** | Higher (UI + processing) | Lower per component |
| **Scalability** | One UI per camera | Multiple UIs per camera |
| **Development** | Coupled development | Independent development |
| **Remote Operation** | Local only | Network capable |
| **Headless Support** | No | Yes |

## Performance

### Typical Performance Metrics
- **Display FPS**: 25-30 FPS (video display)
- **Transport Latency**: 5-15ms (ROS Image messages)
- **Connection Recovery**: <2 seconds
- **Memory Usage**: ~50-100MB (UI process)

### Optimization Tips
1. **Network**: Use wired connection for best video streaming performance
2. **Resolution**: Use 480p for lower latency, 1080p for quality
3. **QoS**: Default settings optimized for real-time video

## Troubleshooting

### Common Issues

#### UI Shows "Connection Lost"
```bash
# Check if camera_node is running
ros2 node list | grep camera

# Check topic publication
ros2 topic hz /coffee_bot/camera/image_raw

# Restart camera_node
ros2 run coffee_vision camera_node
```

#### No Video Stream
```bash
# Check camera permissions
ls -l /dev/video*

# Test camera access
ros2 topic echo /coffee_bot/camera/image_raw --once
```

#### Poor Performance
```bash
# Monitor CPU usage
htop

# Check network latency (if remote)
ping <camera_node_host>

# Reduce video quality
# Use camera controls in UI or launch with lower resolution
```

### Debug Mode

Enable detailed logging:
```bash
ros2 launch coffee_vision_ui camera_ui.launch.py log_level:=debug
```

## Development

### Adding New UI Components

1. Create widget in `coffee_vision_ui/widgets/`
2. Add to `widgets/__init__.py`
3. Import and integrate in `camera_ui_node.py`

### Extending ROS Interface

1. Add publishers/subscribers in `ros_interface/camera_controller.py`
2. Update topic documentation in this README
3. Test with `ros2 topic` commands

### Testing

```bash
# Build and test
colcon build --packages-select coffee_vision_ui
source install/setup.bash

# Run with different configurations
ros2 launch coffee_vision_ui full_system.launch.py camera_index:=0
ros2 launch coffee_vision_ui full_system.launch.py camera_index:=1
```

## Related Packages

- **[coffee_vision](../coffee_vision/)**: Core camera processing and face detection
- **[coffee_robot_description](../coffee_robot_description/)**: Robot system description
- **[coffee_head_control](../coffee_head_control/)**: Robot head movement control
