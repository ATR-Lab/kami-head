# Coffee Speech Processing

ROS2 package containing nodes for speech processing tasks like speech recognition, intent classification, and audio processing for the Coffee Buddy robot system.

## Voice Intent Node

A ROS2 node that performs real-time speech recognition using OpenAI's Whisper model, detects wake words, and classifies user intents using local LLM models via Ollama.

### Architecture

The voice intent node is composed of several modular components:

- **VoiceIntentNode**: Main ROS2 node that coordinates all components
- **AudioProcessor**: Handles audio device management and microphone capture
- **ASRManager**: Manages Whisper ASR models and transcription processing
- **IntentClassifier**: Classifies user intents using local LLM models
- **MemoryManager**: Handles GPU memory management and cleanup

### Features

- Real-time speech transcription from microphone input
- Wake word detection ("buddy")
- Intent classification using local LLM models (Ollama)
- Configurable Whisper model size (tiny, base, small, medium, large)
- GPU acceleration with memory management
- Voice Activity Detection (VAD) to filter out silence
- Multiple supported intents for classification

### Topics

- **Publishers**:
  - `/voice/intent` (coffee_buddy_msgs/IntentClassification): Publishes classified intents with prompt text

### Parameters

#### ASR Parameters
- `model_size` (string, default: 'base'): Whisper model size
  - Options: tiny, base, small, medium, large
  - Larger models are more accurate but slower and require more memory
  
- `language` (string, default: 'en'): Language for transcription
  - Options: en, auto, or other language codes
  - Use 'auto' for automatic language detection

- `device_type` (string, default: 'cuda'): Device type for inference
  - Options: cuda, cpu, mps
  - Use 'cuda' for GPU acceleration if available

- `compute_type` (string, default: 'int8'): Compute type for inference
  - Options: float16, int8_float16, int8, float32
  - Lower precision types (int8) are faster but slightly less accurate

- `timeout_segments` (int, default: 5): Number of segments to process before timeout

- `audio_device_id` (int, default: 0): Audio input device ID

- `verbose_logging` (bool, default: true): Enable verbose logging

#### GPU Parameters
- `gpu_memory_monitoring` (bool, default: true): Enable GPU memory monitoring

- `memory_cleanup_interval` (int, default: 10): Inference cycles between memory cleanup

#### LLM Parameters
- `llm_model` (string, default: 'gemma3:1b'): Ollama model for intent classification

- `llm_timeout` (float, default: 3.0): Timeout in seconds for LLM classification

- `llm_retry` (int, default: 1): Number of retries for LLM classification

### Supported Intents

The intent classifier supports the following intents:

- Greeting (0)
- Goodbye (1)
- Thank (2)
- Apologize (3)
- Affirm (4)
- Deny (5)
- Inform (6)
- Request (7)
- Question (8)
- Confirm (9)
- Disconfirm (10)
- Clarify (11)
- Suggest (12)
- Complaint (13)
- Praise (14)
- Joke (15)
- SmallTalk (16)
- Fallback (17)
- Agree (18)
- Disagree (19)

### Usage

#### Basic Usage

To run the node with default parameters:

```bash
ros2 run coffee_speech_processing voice_intent_node
```

#### Using Launch File

To run with the launch file (recommended):

```bash
ros2 launch coffee_speech_processing voice_intent.launch.py
```

#### Custom Parameters

To specify a different model or language:

```bash
ros2 launch coffee_speech_processing voice_intent.launch.py model_size:=small language:=auto
```

To use CPU instead of GPU:

```bash
ros2 launch coffee_speech_processing voice_intent.launch.py device_type:=cpu
```

To change the LLM model:

```bash
ros2 launch coffee_speech_processing voice_intent.launch.py llm_model:=llama3:8b
```

### Monitor Output

To monitor the intent classification:

```bash
ros2 topic echo /voice/intent
```

### Requirements

- Python 3.8+
- OpenAI Whisper or Faster-Whisper
- PyAudio
- PyTorch (for GPU acceleration)
- Ollama (for intent classification)

### Installation

The required Python packages are listed as dependencies in the package.xml file. They will be installed when building the package with colcon.

```bash
cd ~/your_workspace
colcon build --packages-select coffee_speech_processing
```

### Troubleshooting

#### Audio Devices

If you encounter issues with audio capture, list available devices:

```bash
ros2 param get /voice_intent_node audio_device_id
```

Then set the appropriate device ID:

```bash
ros2 param set /voice_intent_node audio_device_id <ID>
```

#### GPU Memory Issues

If you encounter GPU memory issues, try using a smaller model or disabling GPU:

```bash
ros2 launch coffee_speech_processing voice_intent.launch.py model_size:=tiny device_type:=cpu
``` 