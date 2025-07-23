# Coffee Voice Agent ROS2 Package

A ROS2 package that integrates the LiveKit Coffee Barista Voice Agent with the Coffee Buddy robot system through a clean bridge architecture.

## Overview

This package provides ROS2 integration for the Coffee Barista Voice Agent while preserving its interactive console mode functionality. The voice agent runs as a standalone application with full console controls, while a separate ROS2 bridge node provides system integration via WebSocket communication.

## Features

- **🎙️ Wake Word Detection**: "Hey barista" activation with Porcupine
- **🗣️ Voice Conversation**: STT, LLM, and TTS using LiveKit/OpenAI  
- **😊 Emotion Processing**: Emotion-aware responses with animated expressions
- **☕ Coffee Functions**: Menu, recommendations, and ordering guidance
- **🖥️ Console Mode**: Full interactive controls (Ctrl+B, Q) in terminal
- **🌐 ROS2 Bridge**: WebSocket-based integration with Coffee Buddy system
- **📡 Virtual Requests**: External coffee requests via ROS2 topics

## Architecture

```
coffee_voice_agent/
├── scripts/
│   ├── livekit_voice_agent.py         # Original LiveKit voice agent
│   └── run_voice_agent.sh             # Smart bash launcher
├── coffee_voice_agent/
│   └── voice_agent_bridge.py          # ROS2 bridge node
└── launch/
    ├── voice_agent_bridge.launch.py   # Bridge only
    └── voice_agent_system.launch.py   # Voice agent + bridge together
```

### Communication Flow
```
┌─────────────────┐    WebSocket    ┌─────────────────┐    ROS2     ┌──────────────┐
│  Voice Agent    │◄───────────────►│  ROS2 Bridge    │◄───────────►│ Coffee Buddy │
│  (Console Mode) │    Port 8080    │  (Bridge Node)  │   Topics    │   System     │
│  Interactive    │                 │  Integration    │             │              │
└─────────────────┘                 └─────────────────┘             └──────────────┘
```

## Dependencies

### Environment Variables
```bash
export OPENAI_API_KEY="your_openai_api_key"           # Required
export PORCUPINE_ACCESS_KEY="your_porcupine_key"      # Optional for wake word
export VOICE_AGENT_VOICE="nova"                       # Optional TTS voice  
export VOICE_AGENT_TEMPERATURE="0.7"                  # Optional LLM temperature
export WEBSOCKET_HOST="localhost"                     # Optional WebSocket host
export WEBSOCKET_PORT="8080"                          # Optional WebSocket port
```

### ROS2 Dependencies
- `rclpy`
- `std_msgs` 
- `geometry_msgs`
- `websockets` (Python package)

### Python Dependencies (in setup.py)
- `livekit`
- `livekit-agents[openai,deepgram,silero,turn-detector]`
- `pvporcupine==3.0.5`
- `pvrecorder==1.2.7`
- `python-dotenv`
- `websockets`

## Installation & Usage

### 1. Build the Package
```bash
cd coffee_ws
colcon build --packages-select coffee_voice_agent
source install/setup.bash
```

### 2. Run Voice Agent (Console Mode)

**Primary Method - Direct Execution:**
```bash
# Run directly for full console mode with interactive controls
./src/coffee_voice_agent/scripts/run_voice_agent.sh

# Or after building:
./install/coffee_voice_agent/share/coffee_voice_agent/scripts/run_voice_agent.sh
```

**Console Controls:**
- `[Ctrl+B]` - Toggle between Text/Audio mode
- `[Q]` - Quit the application
- Wake word: Say **"hey barista"** to activate

### 3. ROS2 Integration (Optional)

**Bridge Only** (if voice agent running separately):
```bash
ros2 launch coffee_voice_agent voice_agent_bridge.launch.py
```

**Complete System** (voice agent + bridge together):
```bash  
ros2 launch coffee_voice_agent voice_agent_system.launch.py
```

## ROS2 Topics (Bridge Node)

### Publishers (Voice Agent → ROS2)
- `/voice_agent/state` (String) - Agent state changes (JSON)
- `/voice_agent/conversation` (String) - Conversation transcripts (JSON)
- `/voice_agent/emotion` (String) - Emotion changes (JSON)
- `/voice_agent/connected` (Bool) - Bridge connection status

### Subscribers (ROS2 → Voice Agent)
- `/voice_agent/virtual_requests` (String) - External coffee requests (JSON)
- `/voice_agent/commands` (String) - Voice agent commands (JSON)

## Virtual Requests

Send coffee requests to the voice agent via ROS2:

```bash
# New coffee order
ros2 topic pub /voice_agent/virtual_requests std_msgs/String '{
  "data": "{\"request_type\": \"NEW_COFFEE_REQUEST\", \"content\": \"Espresso\", \"priority\": \"normal\"}"
}'

# Order ready notification
ros2 topic pub /voice_agent/virtual_requests std_msgs/String '{
  "data": "{\"request_type\": \"ORDER_READY\", \"content\": \"Americano (Order: abc123)\", \"priority\": \"urgent\"}"
}'
```

## Monitoring

### Voice Agent Status
```bash
# Check if voice agent is running and WebSocket server is active
curl -I http://localhost:8080

# Monitor voice agent logs directly in console
# (Console mode shows all logs in real-time)
```

### ROS2 Bridge Status  
```bash
# Check bridge connection
ros2 topic echo /voice_agent/connected

# Monitor state changes
ros2 topic echo /voice_agent/state

# Monitor conversations
ros2 topic echo /voice_agent/conversation

# Monitor emotions
ros2 topic echo /voice_agent/emotion
```

## Configuration

### Launch File Parameters
```bash
# Bridge configuration
ros2 launch coffee_voice_agent voice_agent_bridge.launch.py \
    voice_agent_host:=192.168.1.100 \
    voice_agent_port:=8080 \
    reconnect_interval:=5.0

# System configuration  
ros2 launch coffee_voice_agent voice_agent_system.launch.py \
    voice_agent_port:=8080 \
    bridge_reconnect_interval:=3.0
```

## Smart Mode Detection

The bash launcher automatically detects the execution environment:

- **Interactive Terminal**: Uses console mode with full controls
- **Non-Interactive** (ROS2 launch): Automatically switches to start mode
- **Environment Setup**: Loads `.env` files and validates prerequisites

## Integration with Coffee Buddy

### 1. Expression System
```bash
# Bridge can forward emotion changes to expression system
ros2 topic echo /voice_agent/emotion
```

### 2. Coffee Machine Integration  
```bash
# Send machine status updates to voice agent
ros2 topic pub /voice_agent/virtual_requests std_msgs/String '{
  "data": "{\"request_type\": \"ORDER_PROCESSING\", \"content\": \"Your Espresso is brewing!\"}"
}'
```

### 3. System Orchestration
```python
# Include in larger system launch files
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('coffee_voice_agent'),
        '/launch/voice_agent_system.launch.py'
    ])
)
```

## Troubleshooting

### Voice Agent Issues
```bash
# Missing API keys
[ERROR] Missing required environment variables: ['OPENAI_API_KEY']
# Solution: Set OPENAI_API_KEY in environment or .env file

# Wake word not working  
[INFO] Wake Word Detection: ❌ Disabled (always-on mode)
# Solution: Set PORCUPINE_ACCESS_KEY environment variable

# Console mode not working via ROS2 launch
[WARNING] Not running in an interactive terminal
# Solution: Use direct script execution for console mode
```

### Bridge Connection Issues
```bash
# Bridge cannot connect to voice agent
[ERROR] WebSocket connection error: [Errno 111] Connection refused
# Solution: Ensure voice agent is running and WebSocket server is active

# Check WebSocket server status
curl -I http://localhost:8080
```

### Build Issues
```bash
# empy version conflict
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'  
# Solution: pip install empy==3.3.4
```

## Development

### Package Structure
- **Voice Agent**: Standalone CLI application in `scripts/`
- **Bridge Node**: ROS2 integration in `coffee_voice_agent/`
- **Launch Files**: System orchestration in `launch/`

### Adding New Features
1. **Voice functionality**: Modify `livekit_voice_agent.py`
2. **ROS2 integration**: Modify `voice_agent_bridge.py` 
3. **System integration**: Update launch files

### Testing Components
```bash
# Test voice agent directly
./scripts/run_voice_agent.sh

# Test bridge connection
ros2 run coffee_voice_agent voice_agent_bridge

# Test complete system
ros2 launch coffee_voice_agent voice_agent_system.launch.py
```

## Design Principles

- **Separation of Concerns**: Voice processing vs. system integration
- **Console Mode Priority**: Interactive functionality preserved
- **Clean Architecture**: WebSocket bridge avoids threading conflicts
- **Flexibility**: Can run components separately or together
- **ROS2 Native**: Bridge follows ROS2 patterns and conventions

## Future Enhancements

- [ ] Add service interfaces for synchronous voice agent control
- [ ] Add parameter server integration for dynamic configuration
- [ ] Add diagnostics and health monitoring
- [ ] Add audio stream bridging for ROS2 audio topics
- [ ] Add behavior tree integration for complex interaction flows 