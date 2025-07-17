# Coffee LLM Messages

Message and service definitions for Coffee Buddy's Large Language Model (LLM) processing capabilities.

## Overview

The `coffee_llm_msgs` package provides the interface definitions for conversational AI and language processing within the Coffee Buddy robot system. This package contains service definitions that enable natural language interaction between users and the robot.

## Package Information

- **Package Name**: `coffee_llm_msgs`
- **Version**: 0.0.0
- **License**: Apache-2.0
- **Build Type**: ament_cmake

## Services

### ChatService.srv

The primary interface for conversational AI interactions.

**Purpose**: Handles natural language conversations with the Coffee Buddy robot, including context awareness and intelligent response generation.

**Definition**:
```
# Request
string prompt                  # The user's input text
string[] conversation_history  # Optional: previous messages for context
---
# Response
string response               # The LLM's response
bool success                  # Whether the request succeeded
string error                  # Error message if failed
```

**Usage Example**:
```bash
# Simple conversation
ros2 service call /coffee/llm/chat coffee_llm_msgs/srv/ChatService \
  "{prompt: 'Hello, how are you today?'}"

# Conversation with context
ros2 service call /coffee/llm/chat coffee_llm_msgs/srv/ChatService \
  "{prompt: 'What did we just talk about?', conversation_history: ['Hello', 'Hi there! How can I help you?']}"
```

## Integration

### Primary Consumer
- **`coffee_llm_processor`**: Implements the ChatService and provides LLM-based response generation

### Primary Clients
- **`coffee_speech_processing`**: Uses ChatService to process transcribed speech and generate responses
- **`coffee_voice_agent`**: May use ChatService for conversational interactions

## Architecture

```
User Speech → coffee_speech_processing → ChatService → coffee_llm_processor
                                           ↓
Generated Response ← coffee_voice_service ← LLM Response
```

## Dependencies

- `rosidl_default_generators` (build)
- `rosidl_default_runtime` (runtime)

## Building

```bash
cd your_workspace
colcon build --packages-select coffee_llm_msgs
source install/setup.bash
```

## Testing

Check that services are available:
```bash
ros2 interface show coffee_llm_msgs/srv/ChatService
ros2 service list | grep chat
```

## Related Packages

- **`coffee_llm_processor`**: Implementation of LLM processing using this interface
- **`coffee_speech_msgs`**: Speech processing interfaces that work with this package
- **`coffee_voice_service`**: TTS services for outputting LLM responses

## Design Principles

This package follows ROS2 best practices:
- **Domain-specific**: Focused solely on LLM/conversational interfaces
- **Self-contained**: No external configuration dependencies
- **Extensible**: Designed to accommodate future conversational AI features
- **Clear separation**: Separated from speech and voice interfaces for modularity

## Future Enhancements

Potential additions to this package:
- Multi-turn conversation management services
- Intent-based routing services  
- Emotion and sentiment analysis interfaces
- Personality configuration messages
