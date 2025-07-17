# Coffee Speech Messages

Message and service definitions for Coffee Buddy's speech processing pipeline, including speech recognition, intent classification, and text-to-speech services.

## Overview

The `coffee_speech_msgs` package provides the interface definitions for the complete speech processing pipeline within the Coffee Buddy robot system. This package contains both message and service definitions that enable voice interaction, from audio input processing to speech output generation.

## Package Information

- **Package Name**: `coffee_speech_msgs`
- **Version**: 0.0.0
- **License**: Apache-2.0
- **Build Type**: ament_cmake

## Messages

### IntentClassification.msg

Used for publishing classified user intents from speech recognition.

**Purpose**: Carries the results of intent classification from natural language processing, allowing other system components to react to user intentions.

**Definition**:
```
string prompt_text   # The prompt text itself that the user inputted
byte intent          # Enum value for intent classification from LLM 
```

**Intent Mapping**:
The `intent` field uses byte values corresponding to:
- `1`: Agree
- `2`: Disagree  
- `3`: Joke
- `4`: Question
- `5`: None/Fallback

**Usage Example**:
```bash
# Subscribe to intent classifications
ros2 topic echo /voice/intent

# Example message content:
# prompt_text: "Buddy, what time is it?"
# intent: 4  # Question
```

## Services

### TTSQuery.srv

Interface for text-to-speech synthesis requests.

**Purpose**: Enables any component to request speech synthesis, providing a unified interface for voice output across the Coffee Buddy system.

**Definition**:
```
string text   # The text to have the TTS model inference with
---
bool success  # If the TTS query was successful or not
```

**Usage Example**:
```bash
# Request speech synthesis
ros2 service call /coffee/voice/tts/query coffee_speech_msgs/srv/TTSQuery \
  "{text: 'Hello, I am Coffee Buddy!'}"

# Response indicates success/failure
# success: true  # TTS request accepted
```

## Integration

### Speech Processing Pipeline

```
Audio Input → Speech Recognition → Intent Classification → LLM Processing → TTS Output
     ↓                ↓                    ↓                    ↓            ↓
Microphone → coffee_speech_processing → IntentClassification → ChatService → TTSQuery
```

### Primary Producers
- **`coffee_speech_processing`**: 
  - Publishes `IntentClassification` messages (planned feature)
  - Uses `TTSQuery` service for speech output

### Primary Consumers  
- **`coffee_voice_service`**: Implements `TTSQuery` service for speech synthesis
- **System components**: Subscribe to `IntentClassification` for intent-based actions

### Integration Partners
- **`coffee_llm_msgs`**: Works together in the speech → LLM → TTS pipeline
- **`coffee_expressions_msgs`**: May coordinate with facial expressions during speech

## Architecture

### Complete Voice Interaction Flow

```
1. Audio Capture (Microphone)
2. Wake Word Detection ("buddy")
3. Speech Recognition (Whisper ASR)
4. Intent Classification → IntentClassification.msg
5. LLM Processing (ChatService)
6. Speech Synthesis → TTSQuery.srv
7. Audio Output (Speakers)
```

### ROS2 Communication Pattern

```
/voice/intent (IntentClassification) ← coffee_speech_processing
                     ↓
         Intent-based system responses
                     ↓
coffee_speech_processing → /coffee/voice/tts/query (TTSQuery) → coffee_voice_service
```

## Dependencies

- `std_msgs` (for standard message types)
- `rosidl_default_generators` (build)
- `rosidl_default_runtime` (runtime)

## Building

```bash
cd your_workspace
colcon build --packages-select coffee_speech_msgs
source install/setup.bash
```

## Testing

### Check Interface Definitions
```bash
# View message structure
ros2 interface show coffee_speech_msgs/msg/IntentClassification

# View service structure  
ros2 interface show coffee_speech_msgs/srv/TTSQuery
```

### Test TTS Service
```bash
# Check if TTS service is available
ros2 service list | grep tts

# Test TTS functionality
ros2 service call /coffee/voice/tts/query coffee_speech_msgs/srv/TTSQuery \
  "{text: 'Testing the speech system'}"
```

### Monitor Intent Classification
```bash
# Listen for intent messages (when implemented)
ros2 topic echo /voice/intent
```

## Usage Patterns

### Python Usage Example

```python
import rclpy
from rclpy.node import Node
from coffee_speech_msgs.msg import IntentClassification
from coffee_speech_msgs.srv import TTSQuery

class SpeechClient(Node):
    def __init__(self):
        super().__init__('speech_client')
        
        # Subscribe to intent classifications
        self.intent_sub = self.create_subscription(
            IntentClassification, '/voice/intent', 
            self.intent_callback, 10)
        
        # Create TTS service client
        self.tts_client = self.create_client(TTSQuery, '/coffee/voice/tts/query')
        
    def intent_callback(self, msg):
        self.get_logger().info(f'Intent: {msg.intent}, Text: {msg.prompt_text}')
        
        # Respond with TTS
        if msg.intent == 4:  # Question
            self.speak("I heard your question!")
    
    def speak(self, text):
        request = TTSQuery.Request()
        request.text = text
        future = self.tts_client.call_async(request)
        return future
```

### C++ Usage Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <coffee_speech_msgs/msg/intent_classification.hpp>
#include <coffee_speech_msgs/srv/tts_query.hpp>

class SpeechClient : public rclcpp::Node {
public:
    SpeechClient() : Node("speech_client") {
        // Subscribe to intents
        intent_sub_ = this->create_subscription<coffee_speech_msgs::msg::IntentClassification>(
            "/voice/intent", 10, 
            std::bind(&SpeechClient::intent_callback, this, std::placeholders::_1));
        
        // Create TTS client
        tts_client_ = this->create_client<coffee_speech_msgs::srv::TTSQuery>("/coffee/voice/tts/query");
    }
    
private:
    void intent_callback(const coffee_speech_msgs::msg::IntentClassification::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Intent: %d, Text: %s", msg->intent, msg->prompt_text.c_str());
    }
    
    rclcpp::Subscription<coffee_speech_msgs::msg::IntentClassification>::SharedPtr intent_sub_;
    rclcpp::Client<coffee_speech_msgs::srv::TTSQuery>::SharedPtr tts_client_;
};
```

## Related Packages

- **`coffee_speech_processing`**: Primary producer and consumer of these interfaces
- **`coffee_voice_service`**: TTS service implementation using TTSQuery
- **`coffee_llm_msgs`**: Partner package for conversational AI in the speech pipeline
- **`coffee_expressions_msgs`**: Coordinates facial expressions with speech

## Design Principles

This package follows ROS2 best practices:
- **Domain-focused**: Dedicated to speech processing interfaces
- **Pipeline-oriented**: Supports the complete speech input → output flow
- **Loosely coupled**: Clean separation between speech processing and other domains
- **Standards-compliant**: Uses standard ROS2 message/service patterns

## Implementation Notes

### Intent Classification
- Currently, `IntentClassification` publishing is designed but not fully implemented in `coffee_speech_processing`
- The current implementation bypasses intent publishing and goes directly to LLM processing
- Future versions may restore the publish/subscribe pattern for intent distribution

### TTS Integration
- `TTSQuery` provides a non-blocking interface - service responds immediately
- Actual audio playback happens asynchronously
- Monitor `/coffee/voice/tts/audio_state` topic for playback status

## Future Enhancements

Potential additions to this package:
- **Advanced intent types**: More granular intent classifications
- **Speech confidence scores**: Quality metrics for speech recognition
- **Audio streaming interfaces**: Real-time audio processing services
- **Multi-language support**: Language-aware message fields
- **Emotion in speech**: Emotional context in TTS requests
