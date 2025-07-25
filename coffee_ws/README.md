# Table of Contents

- [Overview](#overview)
- [3D Robot Visualization](#3d-robot-visualization)
- [Running the System](#running-the-system)

# Overview

Commands to launch all of the nodes:
```
source ../ros-source.sh
colcon build
ros-source
# Note: coffee_head package was split into coffee_head_control and coffee_vision
# Note: coffee_vision now features modular architecture with configurable parameters
# Launch individual nodes as needed - see sections below
```

## Recent Architecture Improvements

**Coffee Vision Package Refactoring:**
- **Modular Design**: Face detection and coordinate transformation separated into independent modules
- **Configurable Parameters**: Runtime tuning via ROS2 parameters without code changes
- **Headless Operation**: Removed GUI dependencies for better deployment flexibility
- **Improved Testability**: Components can be tested and reused independently

- Camera node handles opening the camera, tracking faces with modular face detection and coordinate transformation (`src/coffee_vision/coffee_vision/camera_node.py`).
  - **Modular Architecture**: Separated face detection (`face_detection.py`) and coordinate utilities (`coordinate_utils.py`)
  - **Configurable Parameters**: ROS2 parameters for face detection sensitivity, eye movement, and coordinate transformation
  - **Headless Operation**: No GUI dependencies, controlled via ROS topics
- Head tracking handles the PID controller, and is in coordination with the camera node to move the motors to center the detected face in frame (`src/coffee_head_control/coffee_head_control/head_tracking.py`).
  - Subscribes to the camera node.
- Coffee Expression show the latest version of the eye shapes with a new topic message (`src/coffee_expressions/coffee_expressions/plaipin_expressive_eyes.py`)
- Coffee Eyes shows the eye visuals for the Coffee Buddy robot,  (`src/coffee_face/coffee_face/coffee_eyes.py`).
  - Subscribes to the camera node to adjust eye position in window.expressions_test_ui
- Coffee Expressions Test UI shows the latest version of the eye shapes with a new topic message (`src/coffee_expressions_state_ui/coffee_expressions_state_ui/coffee_expressions_state_ui.py`).
- Coffee Expressions State Manager handles the state of the robot's expressions (`src/coffee_expressions_state_manager/coffee_expressions_state_manager/state_manager_node.py`).
- Coffee Expressions State UI shows the state of the robot's expressions (`src/coffee_expressions_state_ui/coffee_expressions_state_ui/coffee_expressions_state_ui.py`).


## Topics

- `/vision/emotion` - Vision emotion   

- `/voice/intent` - Voice intent   

- `/vision/face_position` - Face position -- the position of the face transformed to robot eye coordinates. Uses configurable sensitivity and range parameters for eye movement control. NOTE that we can dynamically update the field of view (FOV) of the camera viewer. The smaller the FOV, the faster the head tracking will be.   

- `/system/event` - System event   

- `/robot/affective_state` - Affective state -- the aggregation of the state of the robot's expressions.   

- `/robot/state_manager/diagnostics` - Diagnostics

# 3D Robot Visualization

The `coffee_robot_description` package provides 3D visualization capabilities for the Coffee Buddy robot using URDF, RViz, and Gazebo.

## Dependencies Installation

First, install the required ROS2 packages:

```bash
# Install core dependencies
sudo apt update
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2

# Optional: Install joint state publishers for manual control
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
```

## Building the Package

```bash
# Build the robot description package
cd coffee_ws
colcon build --packages-select coffee_robot_description
source install/setup.bash
```

## Available Launch Files

### 1. Basic Robot State Publisher
Real-time 3D visualization that integrates with existing head control system:

```bash
# Launch robot state publisher with TF integration
ros2 launch coffee_robot_description robot_state_publisher.launch.py

# This automatically:
# - Loads the robot URDF model
# - Starts TF publisher that converts motor angles to joint states
# - Publishes robot description to /robot_description topic
```

### 2. RViz Visualization with GUI Controls
Interactive 3D visualization with manual joint controls:

```bash
# Launch RViz with robot model and joint controls
ros2 launch coffee_robot_description rviz_display.launch.py

# Features:
# - 3D robot model visualization
# - Manual joint sliders for testing
# - TF tree visualization
# - Camera view display
# - Pre-configured RViz setup
```

### 3. Gazebo Physics Simulation
Full physics simulation environment:

```bash
# Launch Gazebo simulation with robot model
ros2 launch coffee_robot_description gazebo_sim.launch.py

# Features:
# - Physics simulation
# - Camera sensor simulation (1920x1080, 80° FOV)
# - IMU sensor simulation
# - ROS2 control integration
# - Joint trajectory controllers
```

## Robot Model Features

The URDF model includes:

- **Coffee Machine Base**: Delonghi Prima Donna with water tank, drip tray, control panel
- **3DOF Neck Assembly**: Pan/Tilt (current hardware) + Roll (future expansion)
- **Robot Head**: LCD display, Logitech Brio camera mount, movable ears
- **Sensor Integration**: Camera frames, IMU, Gazebo plugins

## Hardware Integration

The TF publisher automatically converts motor coordinates to URDF coordinates:

- **Current Hardware**: 2x Dynamixel XM540-W270 (Pan ID:1, Tilt ID:9)
- **Motor Limits**: Pan 143-210°, Tilt 169-206° (both centered at 180°)
- **URDF Limits**: Converted to standard 0° center coordinates
- **Future Ready**: 3rd DOF roll motor and ear actuation pre-defined

## Topics and TF Tree

**Subscribed Topics:**
- `/head_pan_angle` - Current pan motor angle (from existing head control)
- `/head_tilt_angle` - Current tilt motor angle (from existing head control)

**Published Topics:**
- `/robot_description` - Robot URDF description
- `/joint_states` - Current joint positions
- `/tf` and `/tf_static` - Transform tree

**TF Tree:**
```
world → base_link → neck_mount → neck_yaw → neck_pitch → neck_roll → head
                                                                  ├─ camera_link
                                                                  ├─ display_link
                                                                  ├─ left_ear
                                                                  └─ right_ear
```

## Configuration Files

- `config/joint_limits.yaml` - Joint limits based on current hardware
- `config/controllers.yaml` - ROS2 control configuration
- `rviz/coffee_robot.rviz` - Pre-configured RViz setup

## Testing Integration

To test the integration with the existing head control system:

```bash
# Terminal 1: Start robot visualization
ros2 launch coffee_robot_description robot_state_publisher.launch.py

# Terminal 2: Start RViz (optional)
ros2 launch coffee_robot_description rviz_display.launch.py

# Terminal 3: Run existing head control system
ros2 run coffee_head_control head_tracking

# Terminal 4: Run camera for face tracking
ros2 run coffee_vision camera_node

# The 3D model will now move in real-time with the physical robot head!
```

## Troubleshooting

**Import Error for FindPackageShare:**
- Fixed in ROS2 Jazzy - uses `launch_ros.substitutions.FindPackageShare`

**xacro command not found:**
- Install: `sudo apt install ros-jazzy-xacro`

**Joint limits:**
- Current limits based on physical motor constraints
- Future hardware can extend limits in `config/joint_limits.yaml`

# Running the System

Commands to launch separate nodes:

## Sourcing Environment
```
# OLD
source ../ros-source.sh
colcon build
ros-source
```

```
# NEW
# In root of the repo
source ./scripts/setup_env.sh
```

## Launching Camera
```
# Run camera node with default settings
ros2 run coffee_vision camera_node

# Run with custom face detection sensitivity
ros2 run coffee_vision camera_node --ros-args \
  -p face_confidence_threshold:=0.7 \
  -p face_smoothing_factor:=0.6

# Run with custom eye movement parameters
ros2 run coffee_vision camera_node --ros-args \
  -p eye_range:=1.5 \
  -p eye_sensitivity:=2.0 \
  -p invert_x:=true
```

### Camera Node Configuration

The camera node supports comprehensive configuration via ROS2 parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `face_confidence_threshold` | float | 0.5 | Minimum confidence for face detection |
| `face_smoothing_factor` | float | 0.4 | Temporal smoothing (higher = smoother) |
| `eye_range` | float | 1.0 | Maximum eye movement range |
| `eye_sensitivity` | float | 1.5 | Eye movement sensitivity multiplier |
| `invert_x` | bool | false | Invert X axis for eye movement |
| `invert_y` | bool | false | Invert Y axis for eye movement |

**External Control Topics:**
```bash
# Camera selection and quality control
ros2 topic pub /coffee_bot/camera/cmd/select std_msgs/Int32 "data: 1"
ros2 topic pub /coffee_bot/camera/cmd/quality std_msgs/Bool "data: true"
ros2 topic pub /coffee_bot/camera/cmd/face_detection std_msgs/Bool "data: false"

# Request diagnostics
ros2 topic pub /coffee_bot/camera/cmd/diagnostics std_msgs/String "data: 'get'"
```   

## Launching Eye Interface
```
# Run to launch Eye Interface

ros2 run coffee_expressions plaipin_expressive_eyes
```

```
# Run to launch Dynamixel Read Write Node

ros2 run dynamixel_sdk_examples read_write_node
```

## Launching State Manager
```
# Run to launch the proxy node that send data out

ros2 launch coffee_expressions_state_manager state_manager.launch.py
```

## Launching Head Tracking
```
# Run to launch node that receives data from the proxy node and sends it to the head motors via Dynamixel Read Write

ros2 run coffee_head_control head_tracking
```

## Launching MicroROS Agent for Ear Movement
```
# Run MicroROS Agent for Ear Movement
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB1 -v6
```

## Launching Joystick Head Control
```
# Run Joystick Head Control

ros2 run coffee_joystick joystick_control
```

## Launching Coffee Control Node

### Launch Coffee Machine Control with Actual Machine
```
ros2 run coffee_machine_control coffee_machine_control_node --ros-args -p mac_address:=<your_machine_mac>
```

Or use the launch file (recommended)
```
ros2 launch coffee_machine_control coffee_machine_control.launch.py  # Uses default MAC: 9C:95:6E:61B6:2C
ros2 launch coffee_machine_control coffee_machine_control.launch.py mac_address:=<your_machine_mac>
```

### Launch Coffee Machine Control with Mock Machine
```
ros2 run coffee_machine_control coffee_machine_control_node --ros-args -p use_mock_machine:=true -p "mac_address:=''"
```

### Sending Coffee Commands
```
# Make coffee
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'make', parameter: 'espresso'}"
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'make', parameter: 'coffee'}"
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'make', parameter: 'americano'}"

# Cancel brewing
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'cancel', parameter: ''}"

# Control cup light
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'cuplight', parameter: 'on'}"
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'cuplight', parameter: 'off'}"

# Control sound
ros2 service call /coffee_command coffee_machine_control_msgs/srv/CoffeeCommand "{action: 'sound', parameter: 'on'}"

# Get ALL Status
ros2 service call /coffee_machine/get_status coffee_machine_control_msgs/srv/MachineStatusRequest "{}"
```

## Launching Sui Coffee Order Indexer

### Sui Blockchain Coffee Ordering System
The Sui Coffee Order Indexer monitors coffee club events from the Sui blockchain and publishes them to ROS2 topics for coffee machine automation.

```bash
# Build the indexer package
colcon build --packages-select sui_coffee_order_indexer
source install/setup.bash

# Launch with coffee club contract (replace with actual package ID)
ros2 launch sui_coffee_order_indexer indexer.launch.py \
    "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'"

# Launch with different network (mainnet, testnet, devnet)
ros2 launch sui_coffee_order_indexer indexer.launch.py \
    "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'" \
    "network:='mainnet'"

# Launch with custom polling interval and database location
ros2 launch sui_coffee_order_indexer indexer.launch.py \
    "package_id:='0x2ee032ffc863a74a785ac3003fb8b61d639d9095b4431fdc1b12181c0a2a8c13'" \
    "polling_interval_ms:=2000" \
    "database_url:='file:/var/lib/sui_indexer/sui_indexer.db'"
```

**Architecture:** The indexer follows a decoupled design - it only publishes blockchain events to ROS2 topics. Coffee machine integration is handled by separate controller nodes that subscribe to these events.

**Topics Published:**
- `/sui_events` - Coffee club events from the Sui blockchain
- `/indexer_status` - Indexer status and health information

**Events Monitored:**
- `CafeCreated` - New coffee shop registrations
- `CoffeeOrderCreated` - New coffee orders placed
- `CoffeeOrderUpdated` - Order status changes (Processing, Completed, etc.)

**Database:** Events are stored persistently in `<workspace_root>/data/sui_indexer/sui_indexer.db` to survive builds and restarts.

### Integration with Coffee Machine
To create a complete blockchain-to-coffee flow, combine with a coffee controller node:

```bash
# Terminal 1: Run the blockchain indexer
ros2 launch sui_coffee_order_indexer indexer.launch.py "package_id:='0x...'"

# Terminal 2: Run coffee machine control
ros2 launch coffee_machine_control coffee_machine_control.launch.py

# Terminal 3: Run a custom coffee controller node (to be implemented)
# This node would:
# - Subscribe to /sui_events
# - Filter for "Processing" status orders  
# - Call /coffee_command service to make coffee
```

**Example Event Flow:**
```
Sui Blockchain → Indexer → /sui_events → Coffee Controller → /coffee_command → Coffee Machine
```

## Launching Voice Agent (NEW)

### Coffee Voice Agent ROS2 Node
The new integrated voice agent that combines LiveKit voice communication with ROS2:

```bash
# Build the voice agent package
colcon build --packages-select coffee_voice_agent
source install/setup.bash

# Run the voice agent
ros2 launch coffee_voice_agent voice_agent.launch.py

# Or run directly
ros2 run coffee_voice_agent voice_agent_node
```

**Features:**
- Wake word detection ("hey barista")
- Full voice conversation (STT, LLM, TTS)
- Emotion-aware responses
- Coffee menu and recommendations
- ROS2 integration with Coffee Buddy system
- Virtual request handling

**Topics:**
- `/coffee_voice_agent/state` - Agent state
- `/coffee_voice_agent/emotion` - Current emotion
- `/coffee_voice_agent/user_input` - User speech
- `/coffee_voice_agent/agent_response` - Agent responses
- `/coffee_voice_agent/virtual_request` - External requests

**Requirements:**
- `OPENAI_API_KEY` environment variable
- `PORCUPINE_ACCESS_KEY` for wake word (optional)

## Launching Dialogue System (Legacy)

### TTS Node
Receives text to be spoken and sends it to the TTS engine.   
In the LLM flow, all it does is receive the output from the LLM and send it to the TTS engine.
```
# Using launch file (recommended)
ros2 launch coffee_voice_service tts_node.launch.py

# Or run directly
ros2 run coffee_voice_service tts_node
```

`tts_node` explicitly handles the calls to Eleven Labs or Fish Audio. It run a `ROS2 Service` that makes a synchronous call to the server. An example service call is provided here:

```
# Default service endpoint
ros2 service call /coffee/voice/tts/query coffee_speech_msgs/srv/TTSQuery "{text: 'Hey, would you like a cup of coffee?'}"

# Alternative syntax
ros2 service call /coffee/voice/tts/query coffee_speech_msgs/srv/TTSQuery "{text: 'Your text prompt here'}"
```

**Monitor TTS Status:**
```
# Monitor TTS status
ros2 topic echo /coffee/voice/tts/status

# Monitor audio playback state
ros2 topic echo /coffee/voice/tts/audio_state
```

### Voice Intent Node
Open up an ongoing audio stream and employs Voice Activity Detection (VAD) to detect when speech is present and Audio-to-Text (ASR) to transcribe the speech into text. 

The transcribed text is then sent to the LLM processor node.
```
ros2 launch coffee_speech_processing voice_intent.launch.py use_vad:=true vad_silence_duration:=1500
```

### Large Language Model (LLM) Processor Node
In the LLM flow, this node receives the transcribed text from the voice intent node and sends it to the LLM (on Atoma Network or OpenAI).  

NOTE: Current Atoma Network is not supported.
```
# This defaults to OpenAI
#   by line: `self.declare_parameter('api_provider', 'openai')  # Options: 'openai' or 'atoma'`
# `coffee_llm_processor/coffee_llm_processor/language_model_processor_node/node.py`

ros2 run coffee_llm_processor language_model_processor_node
```

We can make a call to the service as follows:
```
# NOTE: This currently attempts to make a function call whenever it receives the "coffee" trigger word

ros2 service call /coffee/llm/chat coffee_llm_msgs/srv/ChatService "{prompt: 'Hello there, what are you doing here?'}"
```

# Terminal Window set up

## Window 1

```
# Camera node with configurable parameters
ros2 run coffee_vision camera_node

# Or with custom face detection and eye movement settings:
# ros2 run coffee_vision camera_node --ros-args -p face_confidence_threshold:=0.7 -p eye_sensitivity:=2.0

ros2 run coffee_expressions plaipin_expressive_eyes

ros2 run dynamixel_sdk_examples read_write_node

ros2 launch coffee_expressions_state_manager state_manager.launch.py

ros2 run coffee_head_control head_tracking

sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB1 -v6

```

## Window 2

```
ros2 launch coffee_voice_service tts_node.launch.py

ros2 run coffee_machine_control coffee_machine_control_node --ros-args -p use_mock_machine:=true -p "mac_address:=''"

ros2 run coffee_llm_processor language_model_processor_node

ros2 launch coffee_speech_processing voice_intent.launch.py use_vad:=true vad_silence_duration:=1500
```


# Miscellaneous
```
# ros2 run coffee_head eye_tracking  # DEPRECATED - functionality moved to coffee_vision or coffee_head_control
ros2 run coffee_face coffee_eyes
ros2 run coffee_expressions_test_ui expressions_test_ui

ros2 run coffee_expressions_state_ui state_ui

# View and modify camera node parameters
ros2 param list /camera_node
ros2 param get /camera_node face_confidence_threshold
ros2 param set /camera_node eye_sensitivity 2.5

#Run with VAD -- 
ros2 launch coffee_speech_processing voice_intent.launch.py use_vad:=true vad_silence_duration:=1500

# Use with threshold: float = 0.4, min_silence_duration_ms: int = 2000, speech_pad_ms: int = 1000
ros2 launch coffee_speech_processing voice_intent.launch.py use_vad:=true vad_silence_duration:=2000

# Run coffee machine control node directly
ros2 run coffee_machine_control coffee_machine_control_node --ros-args -p mac_address:=<your_machine_mac>

# Or use the launch file (recommended)
ros2 launch coffee_machine_control coffee_machine_control.launch.py  # Uses default MAC: 9C:95:6E:61:B6:2C
ros2 launch coffee_machine_control coffee_machine_control.launch.py mac_address:=<your_machine_mac>

```

## Run Joystick
```
$(ros2 pkg prefix coffee_joystick)/bin/joystick_control
```

## Sourcing Environment
```
sudo apt update

sudo apt install portaudio19-dev

# Run the first three in each terminal (except the pip install, run in only one)
python3 -m venv coffee_buddy_venv  # run at root of repo

source ./coffee_buddy_venv/bin/activate

pip install -r requirements.txt

source ./scripts/ros2_venv.sh enable ./coffee_buddy_venv

# Run each in separate windows
ros2 run coffee_speech_processing voice_intent_node

ros2 run coffee_llm_processor language_model_processor_node

ros2 launch coffee_voice_service tts_node.launch.py
```

# ROS2 Package Creation Compatibility

## empy Version Issue

When creating new ROS2 packages using `ros2 pkg create`, you may encounter the following error:

```
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```

This is a known compatibility issue between ROS2 Jazzy and newer versions of the `empy` package (version 4.2+).

### Solution

Downgrade empy to version 3.3.4:

```bash
pip install empy==3.3.4
```

### Why This Happens

- ROS2 Jazzy expects `empy` version 3.3.4 for template processing
- Newer versions of `empy` (4.2+) have removed the `BUFFERED_OPT` attribute
- This affects the `ros2 pkg create` command which uses empy for template expansion

### Alternative Workaround

If you need to use a newer version of empy for other purposes, you can temporarily downgrade just for package creation:

```bash
# Before creating packages
pip install empy==3.3.4

# Create your package
ros2 pkg create --build-type ament_python your_package_name

# Restore newer version if needed
pip install empy==4.2
```

# NOTES:
1. VAD Parameters (Speech Detection):
  - min_silence_duration_ms (Currently 1000ms):
    - Controls how long silence must be detected before ending a speech segment
    - Lower values (e.g., 500ms) = More aggressive segmentation, might split sentences
    - Higher values (e.g., 2000ms) = More natural pauses, better for conversational speech
    - Ideal: 1000-1500ms for command-based systems, as it balances responsiveness with natural speech patterns
  - speech_pad_ms (Currently 800ms):
    - Adds padding before/after detected speech to prevent word clipping
    - Lower values = Tighter segments but risk cutting words
    - Higher values = Safer but may include unwanted audio
    - Ideal: 600-800ms as it provides enough context without excessive padding
2. ASR Buffer Parameter:
  - vad_silence_duration (Your setting: 500ms):
    - Controls how often the ASR attempts to process accumulated audio
    - Impact of increasing:
      - Higher values (e.g., 1000ms+) = Fewer processing attempts, potentially more complete thoughts but higher latency
      - Lower values (e.g., 500ms) = More frequent processing, lower latency but might get partial phrases
    - Ideal: Should be less than or equal to min_silence_duration_ms to ensure ASR processes complete speech segments


# Voice Activity Detection (VAD)

The voice intent node supports Voice Activity Detection (VAD) using Silero VAD. This helps improve transcription accuracy by only processing audio when speech is detected.

## Configuration

VAD can be enabled with the following parameters:

```bash
# Without VAD (default):
ros2 launch coffee_speech_processing voice_intent.launch.py

# With VAD:
ros2 launch coffee_speech_processing voice_intent.launch.py use_vad:=true vad_silence_duration:=500
```

### VAD Parameters

- `use_vad` (bool, default: false): Enable/disable VAD
- `vad_silence_duration` (int, default: 500): Duration of silence in milliseconds before considering speech segment complete

### Advanced VAD Configuration

The VAD system uses the following internal parameters which can be tuned if needed:

- `threshold` (float, default: 0.5): Speech detection threshold
  - Higher values (0.6-0.7): More conservative, less false positives but might miss soft speech
  - Lower values (0.3-0.4): More sensitive, catches soft speech but might trigger on background noise
  - Tune based on environment noise level

- `speech_pad_ms` (int, default: 100): Padding around detected speech in milliseconds
  - Helps prevent cutting off word beginnings/endings
  - Can be increased to 150-200ms if word clipping occurs
  - Higher values trade off with responsiveness

- `sampling_rate` (int, default: 16000): Audio sampling rate in Hz
  - Standard rate for speech processing
  - Compatible with both Silero VAD and Whisper

### Buffer Management

The system uses segment-based buffer management with the same duration as the VAD silence threshold. This ensures:
- Reliable speech detection based on acoustic features
- Low latency processing
- Proper synchronization between VAD and transcription
- Language-independent operation
```

**NOTE:** We define the location of `expressions.json` in the `setup.py` file inside `coffee_expressions` directory.

Running the animator
- Animations are saved in `~/.ros/motion_files` as `.json` files.


# Sample Command

```
src $ ros2 pkg create --build-type ament_python coffee_expressions_test_ui --dependencies rclpy coffee_expressions_msgs python3-pyqt5
```

```
colcon build --packages-select coffee_expressions_test_ui
```

```
source install/setup.bash && ros2 run coffee_expressions_test_ui expressions_test_ui
```


```
ros2 topic echo /robot/affective_state --field gaze_target_v2
```
```
