# Coffee Voice Agent Messages

This package contains ROS2 message definitions for the Coffee Robot's voice agent system. These messages provide a structured interface for communication between the voice agent and other robot components, particularly the orchestrator node that coordinates animatronics based on voice agent state.

## Architecture

The voice agent publishes unified state information through these message types, which are consumed by an orchestrator node that synthesizes this information with other sensor inputs (face tracking, hardware status) to coordinate robot expressions and movements.

```
Voice Agent → Voice Agent Bridge → ROS2 Topics → Orchestrator → Hardware Controllers
```

## Message Types

### AgentStatus.msg

**Primary message** containing unified voice agent status. This single message provides all the context needed for robot coordination in one atomic update.

```
# Behavioral mode: "dormant", "active", "connecting", "disconnecting"
string behavioral_mode

# Speech status: "idle", "speaking"  
string speech_status

# Current emotion: "friendly", "excited", "curious", "sleepy", etc.
string emotion

# Speech text (preview when speaking, full text when idle)
string speech_text

# Previous emotion for smooth transitions
string previous_emotion

# Conversation phase: "", "greeting", "discussion", "announcement", "goodbye"
string conversation_phase

# Last function tool used: "get_current_time", "get_coffee_menu", "", etc.
string last_tool_used

# Timestamp when the status was generated
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/status`

**Behavioral Modes**:
- `dormant` - Wake word detection only, minimal activity
- `connecting` - Establishing LiveKit session
- `active` - Interactive conversation mode  
- `disconnecting` - Ending conversation, returning to dormant

**Speech Status**:
- `idle` - Not currently speaking
- `speaking` - Agent is actively speaking (TTS playback)

**Conversation Phases**:
- `""` - Default/unspecified
- `greeting` - Initial conversation start
- `discussion` - Active back-and-forth conversation
- `announcement` - Virtual request announcements
- `goodbye` - Conversation ending

**Supported Emotions**:
- `friendly` - Default warm expression
- `excited` - High energy, enthusiastic  
- `curious` - Inquisitive, attentive
- `sleepy` - Low energy, tired
- `waiting` - Patient, expectant
- `excuse` - Apologetic, polite interruption

### ToolEvent.msg

Published when the voice agent calls function tools (e.g., getting time, menu info). Useful for UI feedback and analytics.

```
# Tool name: "get_current_time", "get_coffee_menu", etc.
string tool_name

# Input parameters (JSON string array)
string[] parameters

# Tool output/result text
string result

# Status: "started", "completed", "failed"
string status

# Timestamp when the tool event occurred
builtin_interfaces/Time timestamp
```

**Topic**: `voice_agent/tool_events`

## Integration Examples

### Orchestrator Usage

```python
# Subscribe to unified agent status for comprehensive coordination
def agent_status_callback(msg):
    # Coordinate based on behavioral mode
    if msg.behavioral_mode == "dormant":
        enable_idle_mode()
    elif msg.behavioral_mode == "active":
        enable_conversation_mode()
    
    # Coordinate speech animations
    if msg.speech_status == "speaking":
        # Start speech animation with emotion context
        trigger_speech_animation(msg.emotion, msg.speech_text)
    elif msg.speech_status == "idle":
        # Return to listening pose
        return_to_neutral()
    
    # Handle conversation phases
    if msg.conversation_phase == "greeting":
        focus_on_user()
    elif msg.conversation_phase == "announcement":
        attention_getting_gesture()

# Subscribe to tool events for UI feedback
def tool_event_callback(msg):
    if msg.status == "started":
        show_thinking_indicator(msg.tool_name)
    elif msg.status == "completed":
        hide_thinking_indicator()
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
from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent
```

## Topics Published by Voice Agent Bridge

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `voice_agent/status` | `AgentStatus` | Unified agent status for robot coordination |
| `voice_agent/tool_events` | `ToolEvent` | Function tool call tracking |
| `voice_agent/connected` | `std_msgs/Bool` | WebSocket connection status |

## Design Principles

- **Unified Messaging**: Single `AgentStatus` message provides atomic, consistent state
- **Separation of Concerns**: Voice agent publishes state only; orchestrator makes hardware decisions
- **Type Safety**: Structured messages prevent JSON parsing errors
- **Timestamps**: All events include timing for coordination
- **Transition Context**: Previous states/emotions enable smooth animations
- **Rich Context**: Comprehensive status enables intelligent coordination decisions
