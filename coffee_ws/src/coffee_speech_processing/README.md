# Coffee Speech Processing

ROS2 package containing nodes for speech processing tasks like speech recognition, intent classification, and audio processing for the Coffee Buddy robot system.

## Overview

The `coffee_speech_processing` package provides real-time speech recognition and processing capabilities for the Coffee Buddy robot. It handles wake word detection, speech transcription, and integrates with the coffee system's LLM and voice services for complete conversational interaction.

## Voice Intent Node

A ROS2 node that performs real-time speech recognition using OpenAI's Whisper model, detects wake words, and integrates with the coffee system's LLM processor for intelligent response generation.

### Architecture

The voice intent node is composed of several modular components:

- **VoiceIntentNode**: Main ROS2 node that coordinates all components
- **AudioProcessor**: Handles audio device management and microphone capture
- **ASRManager**: Manages Whisper ASR models and transcription processing
- **IntentClassifier**: Classifies user intents using local LLM models
- **MemoryManager**: Handles GPU memory management and cleanup
- **LLM Integration**: Communicates with coffee LLM processor for response generation
- **TTS Integration**: Sends responses to coffee voice service for speech output

### Features

- Real-time speech transcription from microphone input
- Wake word detection ("buddy")
- Intent classification using local LLM models (Ollama)
- Integration with coffee LLM processor for intelligent responses
- Integration with coffee voice service for speech output
- Configurable Whisper model size (tiny, base, small, medium, large)
- GPU acceleration with memory management
- Voice Activity Detection (VAD) to filter out silence
- Self-contained configuration with no external dependencies

### Topics

- **Publishers**:
  - `/voice/intent` (coffee_buddy_msgs/IntentClassification): Publishes classified intents with prompt text

### Service Integration

The node integrates with other coffee system services:

- **LLM Service**: `/coffee/llm/chat` (coffee_interfaces/srv/ChatService)
  - Sends transcribed speech for intelligent response generation
  - Receives generated responses from the coffee LLM processor

- **TTS Service**: `/coffee/voice/tts/query` (coffee_interfaces/srv/TTSService)  
  - Sends generated responses for speech synthesis
  - Integrates with coffee voice service for audio output

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

To monitor service integration:

```bash
# Check LLM service availability
ros2 service list | grep coffee/llm

# Check TTS service availability  
ros2 service list | grep coffee/voice/tts

# Test LLM integration
ros2 service call /coffee/llm/chat coffee_interfaces/srv/ChatService "{prompt: 'Hello'}"

# Test TTS integration
ros2 service call /coffee/voice/tts/query coffee_interfaces/srv/TTSService "{text: 'Hello world'}"
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

### Dependencies

The package has the following ROS2 dependencies:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>coffee_buddy_msgs</depend>
<depend>coffee_interfaces</depend>
```

No external configuration dependencies - all constants are defined locally in `constants.py`.

## Architecture Details

### Complete Processing Flow

```
Audio Input → Voice Intent Node → LLM Service → TTS Service → Audio Output
    ↓              ↓                  ↓             ↓
Microphone → Speech Recognition → Response Gen → Speech Synthesis
             Intent Classification   (LLM)        (Voice Service)
```

### Service Communication

The voice intent node acts as the central coordinator:

1. **Speech Input**: Captures and processes audio from microphone
2. **Wake Word**: Detects "buddy" trigger word  
3. **Transcription**: Converts speech to text using Whisper
4. **Intent Classification**: Analyzes intent using local LLM (optional)
5. **LLM Integration**: Sends transcribed text to `/coffee/llm/chat`
6. **Response Processing**: Receives intelligent response from LLM processor
7. **TTS Integration**: Sends response to `/coffee/voice/tts/query` for speech output

### Configuration Management

The package uses local configuration constants defined in `constants.py`:
- `TTS_SERVICE`: TTS service endpoint (`/coffee/voice/tts/query`)
- `GENERATE_BEHAVIOR_RESPONSE_SERVICE`: LLM service endpoint (`/coffee/llm/chat`)
- `VOICE_INTENT_RESPONSE_TOPIC`: Intent publishing topic (`/voice/intent`)
- `INTENT_MAPPING_BYTE_TO_STRING`: Intent classification mappings

This approach follows ROS2 best practices by avoiding shared configuration dependencies and maintaining package autonomy.

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

#### Service Integration Issues

**LLM Service Not Available:**
```
[ERROR] Service /coffee/llm/chat not available
```
**Solution:** Ensure `coffee_llm_processor` node is running

**TTS Service Not Available:**
```
[ERROR] Service /coffee/voice/tts/query not available  
```
**Solution:** Ensure `coffee_voice_service` node is running

**Intent Classification Timeout:**
```
[WARN] LLM intent classification timed out
```
**Solution:** Increase `llm_timeout` parameter or check Ollama service

### Performance Optimization

- Use smaller Whisper models (`tiny`, `base`) for faster transcription
- Reduce `llm_timeout` for quicker intent classification
- Use `device_type:=cpu` if GPU memory is limited
- Adjust `memory_cleanup_interval` based on available GPU memory

### Development

#### Modifying Service Endpoints

Update the constants in `constants.py` to change service endpoints:
```python
TTS_SERVICE = "/coffee/voice/tts/query"
GENERATE_BEHAVIOR_RESPONSE_SERVICE = "/coffee/llm/chat"
```

#### Adding New Intent Types

Extend the `INTENT_MAPPING_BYTE_TO_STRING` dictionary in `constants.py` to support additional intent classifications.

#### Customizing Wake Words

Modify the wake word detection logic in the voice intent node to support different trigger words or phrases.

## Future Enhancements

- [ ] Multi-language wake word detection
- [ ] Continuous conversation mode (no wake word required)
- [ ] Voice activity detection improvements
- [ ] Real-time audio streaming optimization
- [ ] Custom wake word training
- [ ] Multi-microphone array support
- [ ] Noise cancellation and audio enhancement 