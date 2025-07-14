# Coffee Head Motion Recorder

A ROS2 package for recording and playing back motion sequences for Dynamixel servo-controlled robot head. This package allows you to manually position your robot's head and record the motion for later playback.

## Package Structure

This functionality is split into two packages following ROS2 best practices:

- **`coffee_head_motion_recorder`**: Main implementation package containing the recorder node and UI
- **`coffee_head_motion_recorder_msgs`**: Interface package containing service definitions

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

1. Ensure both packages are in your ROS2 workspace:

   ```bash
   cd /path/to/your/workspace/src
   # coffee_head_motion_recorder/
   # coffee_head_motion_recorder_msgs/
   ```

2. Install dependencies:

   ```bash
   sudo apt-get install python3-pyqt5
   pip3 install dynamixel-sdk
   ```

3. Build the packages (interface package first):

   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select coffee_head_motion_recorder_msgs
   colcon build --packages-select coffee_head_motion_recorder
   ```

4. Source the workspace:

   ```bash
   source /path/to/your/workspace/install/setup.bash
   ```

## Usage

### Launch the Motion Recorder

```bash
ros2 launch coffee_head_motion_recorder motion_recorder.launch.py
```

### Launch Parameters

The launch file accepts several parameters:

- `serial_port`: Serial port for connecting to Dynamixel servos (default: '/dev/ttyUSB0')
- `baudrate`: Baudrate for Dynamixel communication (default: 1000000)
- `sampling_rate`: Sampling rate in Hz for motion recording (default: 50.0)
- `motion_files_dir`: Directory to store motion files (default: ~/.ros/motion_files)

Example with custom parameters:

```bash
ros2 launch coffee_head_motion_recorder motion_recorder.launch.py serial_port:=/dev/ttyUSB1 baudrate:=57600
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

## ROS2 Services

The package provides several ROS2 services for programmatic control:

### Recording Services
- `motion_recorder/start_recording` (std_srvs/Trigger)
- `motion_recorder/stop_recording` (std_srvs/Trigger)
- `motion_recorder/mark_keyframe` (std_srvs/Trigger)
- `motion_recorder/toggle_torque` (std_srvs/SetBool)

### Playback Services
- `motion_recorder/play_motion` (std_srvs/Trigger)
- `motion_recorder/stop_playback` (std_srvs/Trigger)

### File Management Services
- `motion_recorder/save_motion` (coffee_head_motion_recorder_msgs/SaveMotion)
- `motion_recorder/load_motion` (coffee_head_motion_recorder_msgs/LoadMotion)
- `motion_recorder/list_motions` (coffee_head_motion_recorder_msgs/ListMotions)

## License

Apache License 2.0 