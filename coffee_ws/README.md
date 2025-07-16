# Table of Contents

- [Overview](#overview)
- [Running the System](#running-the-system)

# Overview

Commands to launch all of the nodes:
```
source ../ros-source.sh
colcon build
ros-source
ros2 launch coffee_head all_nodes.launch.py
```

- Camera node handles opening the camera, tracking faces (`src/coffee_head/coffee_head/camera_node.py`).
- Head tracking handles the PID controller, and is in coordination with the camera node to move the motors to center the detected face in frame (`src/coffee_head/coffee_head/head_tracking.py`).
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

- `/vision/face_position` - Face position -- the position of the face in the frame of the camera viewer. NOTE that we can dynamically update the field of view (FOV) of the camera viewer. The smaller the FOV, the faster the head tracking will be.   

- `/system/event` - System event   

- `/robot/affective_state` - Affective state -- the aggregation of the state of the robot's expressions.   

- `/robot/state_manager/diagnostics` - Diagnostics

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
# Run for Launching Camera

ros2 run coffee_head camera_node
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

ros2 run coffee_head head_tracking
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
ros2 launch coffee_machine_control coffee_machine_control.launch.py  # Uses default MAC: 9C:95:6E:61:B6:2C
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
ros2 run coffee_voice_service tts_node
```

`tts_node` explicitly handles the calls to Eleven Labs or Fish Audio. It run a `ROS2 Service` that makes a synchronous call to the server. An example service call is provided here:

```
ros2 service call /system/effector/tts/tts_query coffee_buddy_msgs/srv/TTSQuery "{text: 'Hey, would you like a cup of coffee?'}"

ros2 service call /tts_query coffee_buddy_msgs/srv/TTSQuery "{text: 'Your text prompt here'}"
```

### Voice Intent Node
Open up an ongoing audio stream and employs Voice Activity Detection (VAD) to detect when speech is present and Audio-to-Text (ASR) to transcribe the speech into text. 

The transcribed text is then sent to the LLM processor node.
```
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=1500
```

### Large Language Model (LLM) Processor Node
In the LLM flow, this node receives the transcribed text from the voice intent node and sends it to the LLM (on Atoma Network or OpenAI).  

NOTE: Current Atoma Network is not supported.
```
# This defaults to OpenAI
#   by line: `self.declare_parameter('api_provider', 'openai')  # Options: 'openai' or 'atoma'`
# `behavior_nodes/behavior_nodes/language_model_processor_node/node.py`

ros2 run behavior_nodes language_model_processor_node
```

We can make a call to the service as follows:
```
# NOTE: This currently attempts to make a function call whenever it receives the "coffee" trigger word

ros2 service call /chat coffee_interfaces/srv/ChatService "{prompt: 'Hello there, what are you doing here?'}"
```

# Terminal Window set up

## Window 1

```
ros2 run coffee_head camera_node

ros2 run coffee_expressions plaipin_expressive_eyes

ros2 run dynamixel_sdk_examples read_write_node

ros2 launch coffee_expressions_state_manager state_manager.launch.py

ros2 run coffee_head head_tracking

sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB1 -v6

```

## Window 2

```
ros2 run coffee_voice_service tts_node

ros2 run coffee_machine_control coffee_machine_control_node --ros-args -p use_mock_machine:=true -p "mac_address:=''"

ros2 run behavior_nodes language_model_processor_node

ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=1500
```


# Miscellaneous
```
ros2 run coffee_head eye_tracking
ros2 run coffee_face coffee_eyes
ros2 run coffee_expressions_test_ui expressions_test_ui


ros2 run coffee_expressions_state_ui state_ui

#Run with VAD -- 
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=1500

# Use with threshold: float = 0.4, min_silence_duration_ms: int = 2000, speech_pad_ms: int = 1000
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=2000

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
ros2 run perception_nodes voice_intent_node

ros2 run behavior_nodes language_model_processor_node

ros2 run coffee_voice_service tts_node
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
ros2 launch perception_nodes voice_intent.launch.py

# With VAD:
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=500
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