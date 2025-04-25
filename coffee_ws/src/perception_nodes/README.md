# Perception Nodes

ROS2 package containing nodes for perception tasks like speech recognition and audio processing.

## Voice Intent Node

A ROS2 node that performs real-time speech recognition using OpenAI's Whisper model. It listens to the microphone input and publishes transcribed speech.

### Features

- Real-time speech transcription from microphone input
- Configurable Whisper model size (tiny, base, small, medium, large)
- Configurable language setting
- Automatic model download if not present
- Voice Activity Detection (VAD) to filter out silence

### Topics

- **Publishers**:
  - `/voice/intent` (std_msgs/String): Publishes transcribed speech

### Parameters

- `model_size` (string, default: 'base'): Whisper model size
  - Options: tiny, base, small, medium, large
  - Larger models are more accurate but slower and require more memory
  
- `language` (string, default: 'english'): Language for transcription
  - Options: english, auto, or other language codes
  - Use 'auto' for automatic language detection

### Usage

#### Basic Usage

To run the node with default parameters:

```bash
ros2 run perception_nodes voice_intent_node
```

#### Using Launch File

To run with the launch file (recommended):

```bash
ros2 launch perception_nodes voice_intent.launch.py
```

#### Custom Parameters

To specify a different model or language:

```bash
ros2 launch perception_nodes voice_intent.launch.py model_size:=small language:=auto
```

### Monitor Output

To monitor the transcribed speech:

```bash
ros2 topic echo /voice/intent
```

### Requirements

- Python 3.8+
- OpenAI Whisper
- PyAudio
- PyTorch
- LibROSA
- SoundFile

### Installation

The required Python packages are listed as dependencies in the package.xml file. They will be installed when building the package with colcon.

```bash
cd ~/your_workspace
colcon build --packages-select perception_nodes
``` 