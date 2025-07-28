# Coffee Voice Agent Messages

This package contains ROS2 message definitions for the Coffee Robot's voice agent system. These messages provide a structured interface for communication between the voice agent and other robot components, particularly the orchestrator node that coordinates animatronics based on voice agent state.

## Architecture

The voice agent publishes state information through these message types, which are consumed by an orchestrator node that synthesizes this information with other sensor inputs (face tracking, hardware status) to coordinate robot expressions and movements.

```
Voice Agent → Voice Agent Bridge → ROS2 Topics → Orchestrator → Hardware Controllers
```

## Message Types

### TtsEvent.msg

Published when the voice agent starts or stops speaking. Critical for synchronizing robot animations with speech.

```
# Event type: "started" or "finished"  
string event

# Current emotion being expressed
string emotion

# Preview of the text being spoken (truncated for large text)
string text

# Source of the TTS: "manual", "llm", "announcement", etc.
string source

# Timestamp when the event occurred
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/tts_events`

**Usage Example**:
```python
# When agent starts speaking with excitement
{
  "event": "started",
  "emotion": "excited", 
  "text": "Hello! How can I help you today?",
  "source": "manual",
  "timestamp": "..."
}

# When agent finishes speaking
{
  "event": "finished",
  "emotion": "excited",
  "text": "Hello! How can I help you today?", 
  "source": "manual",
  "timestamp": "..."
}
```

### AgentState.msg

Published when the voice agent changes conversation states. Useful for high-level behavioral coordination.

```
# Current state: "dormant", "connecting", "active", "speaking", "disconnecting"
string current_state

# Previous state for transition tracking
string previous_state

# Timestamp when the state change occurred
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/state`

**State Transitions**:
- `dormant` → `connecting` (wake word detected)
- `connecting` → `active` (session established)
- `active` → `speaking` (agent is talking)
- `speaking` → `active` (agent finished talking)
- `active` → `disconnecting` (conversation ending)
- `disconnecting` → `dormant` (back to wake word detection)

### EmotionState.msg

Published when the voice agent's emotional expression changes. Used by orchestrator to coordinate eye expressions and ear movements.

```
# Current emotion: "friendly", "excited", "curious", "sleepy", etc.
string emotion

# Previous emotion for smooth transition tracking  
string previous_emotion

# Timestamp when the emotion change occurred
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/emotion`

**Supported Emotions**:
- `friendly` - Default warm expression
- `excited` - High energy, enthusiastic
- `curious` - Inquisitive, attentive
- `sleepy` - Low energy, tired
- `waiting` - Patient, expectant
- `excuse` - Apologetic, polite interruption

### ConversationItem.msg

Published for each turn in the conversation transcript. Provides context for engagement-based animations.

```
# Speaker role: "user" or "assistant"
string role

# The spoken/generated text content
string text

# Timestamp when this conversation item was added
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/conversation`

## Integration Examples

### Orchestrator Usage

```python
# Subscribe to TTS events for animation coordination
def tts_event_callback(msg):
    if msg.event == "started":
        # Coordinate: ears perk up, eyes show emotion, head oriented to user
        trigger_speech_animation(msg.emotion)
    elif msg.event == "finished": 
        # Return to neutral state, allow head movement
        return_to_neutral()

# Subscribe to state changes for behavioral modes
def state_change_callback(msg):
    if msg.current_state == "dormant":
        # Enable idle animations, wake word detection ready
        enable_idle_mode()
    elif msg.current_state == "active":
        # Focus on user, ready for interaction
        enable_conversation_mode()
```

### Message Dependencies

This package depends on:
- `std_msgs` - Standard ROS message types
- `builtin_interfaces` - Time stamping

## Building

```bash
# Build this package
colcon build --packages-select coffee_voice_agent_msgs

# Source the workspace  
source install/setup.bash
```

## Usage in Other Packages

To use these messages in other ROS2 packages:

1. Add dependency in `package.xml`:
```xml
<depend>coffee_voice_agent_msgs</depend>
```

2. Add to `CMakeLists.txt`:
```cmake
find_package(coffee_voice_agent_msgs REQUIRED)
```

3. Import in Python:
```python
from coffee_voice_agent_msgs.msg import TtsEvent, AgentState, EmotionState, ConversationItem
```

## Topics Published by Voice Agent Bridge

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `voice_agent/tts_events` | `TtsEvent` | Speech start/stop with emotion |
| `voice_agent/state` | `AgentState` | Conversation state changes |
| `voice_agent/emotion` | `EmotionState` | Emotional expression changes |
| `voice_agent/conversation` | `ConversationItem` | Conversation transcript |
| `voice_agent/connected` | `std_msgs/Bool` | WebSocket connection status |

## Design Principles

- **Separation of Concerns**: Voice agent publishes state only; orchestrator makes hardware decisions
- **Type Safety**: Structured messages prevent JSON parsing errors
- **Timestamps**: All events include timing for coordination
- **Transition Context**: Previous states/emotions enable smooth animations
- **Rich Context**: Emotion + text content enables intelligent coordination decisions
