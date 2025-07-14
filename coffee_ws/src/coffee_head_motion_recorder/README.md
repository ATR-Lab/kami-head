# Motion Recorder

A ROS2 package for recording and playing back motion sequences for Dynamixel servo-controlled robot head. This package allows you to manually position your robot's head and record the motion for later playback.

## Features

- Record head motion by manually moving it with torque disabled
- Mark keyframes during recording to identify important positions
- Save and load motion sequences
- Play back recorded motion sequences with smooth interpolation
- User-friendly Qt-based interface

## Requirements

- ROS2 (Jazzy or compatible)
- Python 3
- PyQt5
- Dynamixel SDK
- XM540-W270 Dynamixel servo motors

## Installation

1. Clone this repository into your ROS2 workspace:

   ```bash
   cd /path/to/your/workspace/src
   git clone https://github.com/yourusername/motion_recorder.git
   ```

2. Install dependencies:

   ```bash
   sudo apt-get install python3-pyqt5
   pip3 install dynamixel-sdk
   ```

3. Build the package:

   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select motion_recorder
   ```

4. Source the workspace:

   ```bash
   source /path/to/your/workspace/install/setup.bash
   ```

## Usage

### Launch the Motion Recorder

```bash
ros2 launch motion_recorder motion_recorder.launch.py
```

### Launch Parameters

The launch file accepts several parameters:

- `serial_port`: Serial port for connecting to Dynamixel servos (default: '/dev/ttyUSB0')
- `baudrate`: Baudrate for Dynamixel communication (default: 1000000)
- `sampling_rate`: Sampling rate in Hz for motion recording (default: 50.0)
- `motion_files_dir`: Directory to store motion files (default: ~/.ros/motion_files)

Example with custom parameters:

```bash
ros2 launch motion_recorder motion_recorder.launch.py serial_port:=/dev/ttyUSB1 baudrate:=57600
```

### Recording Motion

1. Launch the motion recorder
2. Click "Disable Torque" to allow manual positioning
3. Click "Start Recording" to begin recording
4. Manually move the robot head to create your motion
5. Click "Mark Keyframe" at important positions
6. Click "Stop Recording" when finished
7. Enter a name for your motion and click "Save Motion"

### Playing Back Motion

1. Launch the motion recorder
2. Select a motion from the "Motion Library" tab or load a motion
3. Click "Play Motion" to play back the recorded motion
4. Click "Stop Playback" to stop the motion

## License

Apache License 2.0 