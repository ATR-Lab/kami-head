# Coffee Voice Agent ROS2 Package

A ROS2 wrapper for the Coffee Barista Voice Agent that integrates LiveKit voice communication with the Coffee Buddy robot system.

## Overview

This package wraps the original LiveKit voice agent (`livekit_voice_agent.py`) with a ROS2 node (`voice_agent_node.py`) to provide seamless integration with the Coffee Buddy robot ecosystem.

## Features

- **🎙️ Wake Word Detection**: "Hey barista" activation
- **🗣️ Voice Conversation**: STT, LLM, and TTS using LiveKit/OpenAI
- **😊 Emotion Processing**: Emotion-aware responses with animated expressions
- **☕ Coffee Functions**: Menu, recommendations, and coffee commands
- **🤖 ROS2 Integration**: Publishes/subscribes to Coffee Buddy topics
- **🔗 Virtual Requests**: External coffee requests via ROS2

## Architecture

```
coffee_voice_agent/
├── livekit_voice_agent.py      # Original voice agent (933 lines)
├── voice_agent_node.py         # ROS2 wrapper + integration
└── launch/
    └── voice_agent.launch.py   # Launch file
```

## Dependencies

### Environment Variables
```bash
export OPENAI_API_KEY="your_openai_api_key"
export PORCUPINE_ACCESS_KEY="your_porcupine_key"  # Optional for wake word
export VOICE_AGENT_VOICE="nova"                   # Optional TTS voice
export VOICE_AGENT_TEMPERATURE="0.7"              # Optional LLM temperature
```

### ROS2 Dependencies
- `rclpy`
- `std_msgs`
- `coffee_machine_control_msgs`
- `coffee_expressions_msgs`

### Python Dependencies
- `livekit`
- `livekit-agents[openai,deepgram,silero,turn-detector]`
- `pvporcupine==3.0.5`
- `pvrecorder==1.2.7`
- `python-dotenv`

## Usage

### Build the Package
```bash
cd coffee_ws
colcon build --packages-select coffee_voice_agent
source install/setup.bash
```

### Run the Voice Agent
```bash
# Using launch file (recommended)
ros2 launch coffee_voice_agent voice_agent.launch.py

# Or run directly
ros2 run coffee_voice_agent voice_agent_node
```

## ROS2 Topics

### Publishers
- `/coffee_voice_agent/state` (String) - Agent state (dormant, active, speaking, etc.)
- `/coffee_voice_agent/emotion` (String) - Current emotion (excited, friendly, curious, etc.)
- `/coffee_voice_agent/user_input` (String) - User speech input
- `/coffee_voice_agent/agent_response` (String) - Agent responses
- `/coffee_voice_agent/wake_word_detected` (Bool) - Wake word detection events

### Subscribers
- `/coffee_voice_agent/virtual_request` (String) - External coffee requests (JSON format)

### Integration Topics
- Publishes to `/robot/affective_state` for expression system
- Can subscribe to `/coffee_machine/get_status` for machine updates

## Virtual Requests

Send coffee requests via ROS2:

```bash
ros2 topic pub /coffee_voice_agent/virtual_request std_msgs/String '{
  "data": "{\"request_type\": \"NEW_COFFEE_REQUEST\", \"content\": \"Espresso\", \"priority\": \"normal\"}"
}'
```

## Monitoring

Monitor voice agent activity:

```bash
# Watch state changes
ros2 topic echo /coffee_voice_agent/state

# Watch emotions
ros2 topic echo /coffee_voice_agent/emotion

# Watch user input
ros2 topic echo /coffee_voice_agent/user_input

# Watch agent responses  
ros2 topic echo /coffee_voice_agent/agent_response
```

## Integration with Coffee Buddy

The voice agent integrates with existing Coffee Buddy systems:

1. **Expression System**: Publishes emotions to `/robot/affective_state`
2. **Coffee Control**: Can send coffee commands (future enhancement)
3. **Behavior System**: Receives and processes virtual requests
4. **Head Control**: Emotion changes trigger head movements

## Troubleshooting

### Missing API Keys
```
[ERROR] Missing required environment variables: ['OPENAI_API_KEY']
```
**Solution**: Set your OpenAI API key in environment or `.env` file

### Wake Word Not Working
```
[INFO] Wake Word Detection: ❌ Disabled (always-on mode)
```
**Solution**: Set `PORCUPINE_ACCESS_KEY` environment variable

### Build Errors
```
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
```
**Solution**: Install correct empy version: `pip install empy==3.3.4`

## Development

### Adding New Features
1. Modify `livekit_voice_agent.py` for voice agent functionality
2. Modify `voice_agent_node.py` for ROS2 integration
3. Add new topics/services as needed

### Testing
```bash
# Test basic ROS2 functionality
ros2 node info /coffee_voice_agent

# Test topic publishing
ros2 topic hz /coffee_voice_agent/state

# Test virtual requests
ros2 topic pub /coffee_voice_agent/virtual_request std_msgs/String '{"data": "test"}'
```

## Future Enhancements

- [ ] Add coffee machine control integration
- [ ] Add behavior tree integration  
- [ ] Add face recognition integration
- [ ] Add gesture control
- [ ] Add configuration via ROS2 parameters
- [ ] Add diagnostics and monitoring 