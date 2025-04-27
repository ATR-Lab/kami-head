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


# Topics

- `/vision/emotion` - Vision emotion   

- `/voice/intent` - Voice intent   

- `/vision/face_position` - Face position -- the position of the face in the frame of the camera viewer. NOTE that we can dynamically update the field of view (FOV) of the camera viewer. The smaller the FOV, the faster the head tracking will be.   

- `/system/event` - System event   

- `/robot/affective_state` - Affective state -- the aggregation of the state of the robot's expressions.   

- `/robot/state_manager/diagnostics` - Diagnostics

Commands to launch separate nodes:

```
source ../ros-source.sh
colcon build
ros-source
ros2 run coffee_head camera_node
ros2 run coffee_head head_tracking
ros2 run coffee_head eye_tracking
ros2 run coffee_expressions plaipin_expressive_eyes
ros2 run coffee_face coffee_eyes
ros2 run coffee_expressions_test_ui expressions_test_ui
ros2 launch coffee_expressions_state_manager state_manager.launch.py

ros2 run coffee_expressions_state_ui state_ui

#Run with VAD -- 
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=1500

# Use with threshold: float = 0.4, min_silence_duration_ms: int = 2000, speech_pad_ms: int = 1000
ros2 launch perception_nodes voice_intent.launch.py use_vad:=true vad_silence_duration:=2000


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