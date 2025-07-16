# Effector Nodes

A ROS2 package providing Text-to-Speech (TTS) effector nodes for the Coffee Buddy robot system using the ElevenLabs API.

## Overview

The `effector_nodes` package serves as the primary TTS effector in the Coffee Buddy robot ecosystem. It provides high-quality, multilingual text-to-speech synthesis through the ElevenLabs API, enabling the robot to communicate naturally with users through voice responses.

## Features

- **🎙️ High-Quality TTS**: Uses ElevenLabs API for professional-grade voice synthesis
- **🌍 Multilingual Support**: Built-in support for multiple languages via ElevenLabs multilingual models
- **🔄 Streaming Audio**: Direct audio streaming from API to speakers for low-latency response
- **⏱️ Smart Cooldown**: Prevents audio overlap with configurable cooldown periods
- **📊 State Management**: Real-time broadcasting of TTS and audio playback states
- **🔧 Configurable**: Fully configurable voice, model, and audio settings via ROS2 parameters
- **🛡️ Thread-Safe**: Concurrent audio processing with proper resource management
- **📈 Health Monitoring**: Regular status publishing for system monitoring

## Architecture

### ROS2 Interface

**Services:**
- `/system/effector/tts/tts_query` (coffee_buddy_msgs/TTSQuery)
  - Request: `string text`
  - Response: `bool success`

**Topics Published:**
- `/system/effector/tts/status` (std_msgs/String) - JSON status with health info
- `tts/audio_state` (std_msgs/String) - Audio playback state ('playing', 'cooldown', 'done')

### Integration

The TTS node integrates seamlessly with the Coffee Buddy voice pipeline:

```
VoiceIntentNode → LanguageModelProcessorNode → TTSNode → Audio Output
```

## Installation

### Prerequisites

1. **ROS2 Installation**: Requires ROS2 Humble or newer
2. **System Dependencies**: 
   ```bash
   sudo apt-get install python3-pyaudio portaudio19-dev
   ```

3. **ElevenLabs API Key**: Sign up at [ElevenLabs](https://elevenlabs.io/) and get an API key

### Build Instructions

1. **Clone and build the package:**
   ```bash
   cd coffee_ws
   colcon build --packages-select effector_nodes
   source install/setup.bash
   ```

2. **Install Python dependencies:**
   ```bash
   pip install elevenlabs>=1.0.0 pyaudio>=0.2.11
   ```

### API Key Setup

Set your ElevenLabs API key using one of these methods:

**Method 1: Environment Variable (Recommended)**
```bash
export ELEVEN_LABS_API_KEY="your_api_key_here"
```

**Method 2: ROS2 Parameter**
```bash
ros2 run effector_nodes tts_node --ros-args -p api_key:="your_api_key_here"
```

**Method 3: Launch File Parameter**
```bash
ros2 launch effector_nodes tts_node.launch.py api_key:="your_api_key_here"
```

## Usage

### Basic Usage

**Start the TTS node:**
```bash
# Using launch file (recommended)
ros2 launch effector_nodes tts_node.launch.py

# Or run directly
ros2 run effector_nodes tts_node
```

**Test TTS service:**
```bash
ros2 service call /system/effector/tts/tts_query coffee_buddy_msgs/srv/TTSQuery "{text: 'Hello, I am Coffee Buddy!'}"
```

**Monitor status:**
```bash
# Monitor general status
ros2 topic echo /system/effector/tts/status

# Monitor audio playback state
ros2 topic echo tts/audio_state
```

### Advanced Configuration

**Custom voice and model:**
```bash
ros2 launch effector_nodes tts_node.launch.py \
  voice_id:="pNInz6obpgDQGcFmaJgB" \
  model_id:="eleven_multilingual_v2" \
  cooldown_duration:=2.0
```

**Different audio format:**
```bash
ros2 launch effector_nodes tts_node.launch.py \
  output_format:="pcm_16000"
```

## Configuration

### ROS2 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voice_id` | string | `KTPVrSVAEUSJRClDzBw7` | ElevenLabs voice ID |
| `model_id` | string | `eleven_multilingual_v2` | ElevenLabs model ID |
| `api_key` | string | `""` | API key (falls back to env var) |
| `cooldown_duration` | double | `1.0` | Cooldown between requests (seconds) |
| `output_format` | string | `pcm_24000` | Audio format (pcm_16000, pcm_24000) |

### Voice Selection

**Popular ElevenLabs Voices:**
- `KTPVrSVAEUSJRClDzBw7` - Default Coffee Buddy voice (energetic, friendly)
- `pNInz6obpgDQGcFmaJgB` - Adam (deep, authoritative)
- `EXAVITQu4vr4xnSDxMaL` - Sarah (warm, conversational)
- `VR6AewLTigWG4xSOukaG` - Nicole (professional, clear)

Find more voices at: [ElevenLabs Voice Library](https://elevenlabs.io/app/voice-library)

### Model Options

- `eleven_multilingual_v2` - Best quality, supports 29+ languages
- `eleven_multilingual_v1` - Good quality, faster processing
- `eleven_monolingual_v1` - English only, fastest processing

## API Reference

### TTSQuery Service

**Service Type:** `coffee_buddy_msgs/srv/TTSQuery`

**Request:**
```
string text   # Text to synthesize
```

**Response:**
```
bool success  # True if TTS request accepted, False if busy/error
```

**Behavior:**
- Returns `success: true` immediately if request accepted
- Returns `success: false` if already playing audio or in cooldown
- Audio plays asynchronously in background thread
- Publishes state updates to `tts/audio_state` topic

### Status Topic

**Topic:** `/system/effector/tts/status`
**Type:** `std_msgs/String` (JSON format)

**Status JSON Format:**
```json
{
  "health": "ok",
  "voice_id": "KTPVrSVAEUSJRClDzBw7",
  "model_id": "eleven_multilingual_v2",
  "is_playing": false,
  "in_cooldown": false
}
```

### Audio State Topic

**Topic:** `tts/audio_state`
**Type:** `std_msgs/String`

**Possible Values:**
- `"playing"` - Audio is currently being synthesized/played
- `"cooldown"` - In cooldown period, rejecting new requests
- `"done"` - Ready for new requests

## Integration Examples

### Python Client Example

```python
import rclpy
from rclpy.node import Node
from coffee_buddy_msgs.srv import TTSQuery
from std_msgs.msg import String

class TTSClient(Node):
    def __init__(self):
        super().__init__('tts_client')
        self.client = self.create_client(TTSQuery, '/system/effector/tts/tts_query')
        
        # Subscribe to audio state for synchronization
        self.create_subscription(String, 'tts/audio_state', self.audio_state_callback, 10)
        
    def speak(self, text):
        request = TTSQuery.Request()
        request.text = text
        future = self.client.call_async(request)
        return future
        
    def audio_state_callback(self, msg):
        self.get_logger().info(f'Audio state: {msg.data}')

# Usage
rclpy.init()
client = TTSClient()
future = client.speak("Hello from Coffee Buddy!")
rclpy.spin_until_future_complete(client, future)
```

### C++ Client Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <coffee_buddy_msgs/srv/tts_query.hpp>

class TTSClient : public rclcpp::Node {
public:
    TTSClient() : Node("tts_client") {
        client_ = this->create_client<coffee_buddy_msgs::srv::TTSQuery>(
            "/system/effector/tts/tts_query");
    }
    
    void speak(const std::string& text) {
        auto request = std::make_shared<coffee_buddy_msgs::srv::TTSQuery::Request>();
        request->text = text;
        
        auto future = client_->async_send_request(request);
        // Handle response...
    }
    
private:
    rclcpp::Client<coffee_buddy_msgs::srv::TTSQuery>::SharedPtr client_;
};
```

## Troubleshooting

### Common Issues

**1. "ELEVEN_LABS_API_KEY not set" Error**
- Ensure API key is set as environment variable or ROS2 parameter
- Verify API key is valid and has sufficient credits

**2. "PyAudio not found" Error**
```bash
sudo apt-get install python3-pyaudio portaudio19-dev
pip install pyaudio
```

**3. Audio not playing**
- Check audio device permissions
- Verify PulseAudio/ALSA configuration
- Test with: `pactl list short sinks`

**4. High latency**
- Use `pcm_16000` format for faster processing
- Reduce cooldown_duration parameter
- Check network connection to ElevenLabs API

**5. Service call timeouts**
- TTS service responds immediately (non-blocking)
- Monitor `tts/audio_state` topic for actual completion
- Don't wait for audio completion in service call

### Debug Commands

```bash
# Check if node is running
ros2 node list | grep tts_node

# Monitor all TTS topics
ros2 topic list | grep tts

# Check service availability
ros2 service list | grep tts_query

# View node parameters
ros2 param list /tts_node

# Test with minimal text
ros2 service call /system/effector/tts/tts_query coffee_buddy_msgs/srv/TTSQuery "{text: 'test'}"
```

## Development

### Building from Source

```bash
cd coffee_ws
colcon build --packages-select effector_nodes --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests

```bash
colcon test --packages-select effector_nodes
colcon test-result --verbose
```

### Code Style

This package follows ROS2 and PEP8 coding standards:

```bash
# Check code style
ament_flake8 src/
ament_pep257 src/

# Run all tests
colcon test --packages-select effector_nodes
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes following ROS2 conventions
4. Add tests for new functionality
5. Submit a pull request
