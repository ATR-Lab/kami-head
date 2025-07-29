# Coffee Voice Agent UI

A comprehensive PyQt-based monitoring dashboard for the Coffee Voice Agent system. This package provides real-time visualization and control of voice agent status, emotions, tool usage, conversation flow, and system analytics.

![Coffee Voice Agent Monitor](https://img.shields.io/badge/ROS2-Jazzy-blue) ![PyQt5](https://img.shields.io/badge/UI-PyQt5-green) ![License](https://img.shields.io/badge/License-TODO-red)

## 🎯 Overview

The Coffee Voice Agent UI is a standalone ROS2 application that provides comprehensive monitoring and control capabilities for the coffee robot's voice interaction system. It connects to the voice agent bridge to display real-time status information, conversation transcripts, emotional states, and system performance metrics.

### Key Features

- **🤖 Real-time Agent Status**: Monitor connection state, behavioral mode, and session information
- **🎭 Emotion Visualization**: Track current emotions, transitions, and emotional journey
- **💬 Live Conversation Flow**: Real-time transcript with user speech and agent responses
- **🔧 Tool Activity Monitor**: Function tool execution tracking with performance metrics
- **📊 Analytics Dashboard**: Usage statistics, performance trends, and system health
- **⚙️ Manual Controls**: Testing interface for virtual requests and debug commands

## 📋 Prerequisites

- **ROS2 Jazzy** (or compatible version)
- **Python 3.8+**
- **PyQt5** (installed via `python_qt_binding`)
- **coffee_voice_agent_msgs** package
- **coffee_voice_agent** package (for full system operation)

## 🚀 Installation

### 1. Clone and Build

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_ws/src

# Build the package
cd ..
colcon build --packages-select coffee_voice_agent_ui

# Source the workspace
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check if the package is available
ros2 pkg list | grep coffee_voice_agent_ui

# Check available executables
ros2 pkg executables coffee_voice_agent_ui
```

## 🎮 Usage

### Option 1: Standalone Monitor (Recommended)

Launch just the monitoring UI to connect to an existing voice agent system:

```bash
# Direct execution
ros2 run coffee_voice_agent_ui voice_agent_monitor

# Or using launch file with parameters
ros2 launch coffee_voice_agent_ui monitor.launch.py \
    ui_node_name:=voice_monitor \
    log_level:=debug
```

### Option 2: Full System Launch

Launch both the voice agent bridge and monitor UI together:

```bash
ros2 launch coffee_voice_agent_ui full_system.launch.py \
    voice_agent_host:=localhost \
    voice_agent_port:=8080 \
    enable_ui:=true
```

### Option 3: RQT Plugin (Development)

For development purposes, the UI can also be loaded as an RQT plugin:

```bash
# Load in rqt
rqt

# Navigate to: Plugins → Coffee Robot → Voice Agent Monitor
# Or run standalone rqt plugin
rqt --standalone coffee_voice_agent_ui
```

## 🏗️ Architecture

### System Overview

```
Voice Agent ←→ Voice Agent Bridge ←→ ROS2 Topics ←→ Monitor UI
```

### Component Architecture

```
VoiceAgentMonitorApp (QMainWindow)
├── Agent Status Widget (connection, state, metrics)
├── Emotion Display Widget (current emotion, transitions, timeline)
├── Conversation Widget (live transcript, timeouts, turn tracking)
├── Tool Monitor Widget (function tool usage, performance)
├── Analytics Widget (usage trends, performance metrics)
└── Controls Widget (manual testing, debug commands)
```

### ROS2 Integration

The monitor subscribes to these topics:
- `voice_agent/status` (AgentStatus) - Unified agent state information
- `voice_agent/tool_events` (ToolEvent) - Function tool execution events
- `voice_agent/user_speech` (String) - User speech transcriptions
- `voice_agent/connected` (Bool) - Connection status updates

The monitor publishes to these topics:
- `voice_agent/virtual_requests` (String) - Test virtual coffee requests
- `voice_agent/commands` (String) - Debug and control commands

## 📊 Dashboard Layout

```
┌─────────────────┬─────────────────┬─────────────────┐
│ 🤖 Agent Status │ 💬 Conversation │ 📊 Analytics    │
│ • State & Mode  │   Flow          │ • Performance   │
│ • Connection    │ • Live Chat     │ • Usage Stats   │
│ • Session Info  │ • Timeouts      │ • Trends        │
├─────────────────┼─────────────────┼─────────────────┤
│ 🎭 Emotion      │ 🔧 Tool Monitor │ ⚙️ Controls     │
│   Center        │ • Active Tools  │ • Virtual Reqs  │
│ • Current       │ • Recent Calls  │ • Commands      │
│ • Transitions   │ • Statistics    │ • Debug Tools   │
└─────────────────┴─────────────────┴─────────────────┘
```

## 🎛️ Features Guide

### Agent Status Panel
- **Real-time state indicator**: Visual status with color coding
- **Connection monitoring**: WebSocket connection health
- **Session tracking**: Active conversation duration and metrics
- **Phase indication**: Current conversation phase (greeting, discussion, etc.)

### Emotion Center
- **Current emotion display**: Large visual indicator with emoji
- **Transition tracking**: Previous → current emotion flow
- **Emotion timeline**: Visual history of emotional journey
- **Eye animation preview**: How emotions translate to robot expressions

### Conversation Flow
- **Live transcript**: Real-time user and agent messages
- **Tool integration**: Function tool calls inline with conversation
- **Timeout monitoring**: User response countdown with visual indicators
- **Turn tracking**: Conversation metrics and response times

### Tool Monitor
- **Active tracking**: Currently executing tools with duration
- **Recent activity**: Last 10 tool calls with status and timing
- **Usage statistics**: Success rates, execution times, frequency
- **Performance metrics**: Tool reliability and efficiency tracking

### Analytics
- **Session performance**: Daily conversation count, success rates
- **Popular interactions**: Most requested functions and topics
- **Emotion trends**: Emotional state patterns over time
- **System metrics**: Message rates, connection health, queue status

### Controls Panel
- **Quick actions**: End conversation, reset state, pause wake word
- **Virtual request testing**: Simulate coffee orders with different priorities
- **Command interface**: Send debug commands with parameters
- **Debug tools**: Connection testing, log export, tool triggers

## 📁 Launch Files

### `monitor.launch.py`
Launches only the monitor UI for connecting to existing voice agent systems.

**Parameters:**
- `ui_node_name` (default: "voice_agent_monitor") - Name for the UI node
- `log_level` (default: "info") - Logging level
- `window_title` (default: "Coffee Voice Agent Monitor") - Window title

### `full_system.launch.py`
Launches both the voice agent bridge and monitor UI for complete system operation.

**Parameters:**
- `voice_agent_host` (default: "localhost") - Voice agent WebSocket host
- `voice_agent_port` (default: "8080") - Voice agent WebSocket port
- `enable_ui` (default: "true") - Enable/disable monitor UI
- `log_level` (default: "info") - Logging level for all nodes

## 🔧 Configuration

### Environment Variables
- `QT_AUTO_SCREEN_SCALE_FACTOR=1` - Qt high DPI scaling
- `ROS_DOMAIN_ID` - ROS2 domain for multi-robot systems

### Parameters
The monitor can be configured through ROS2 parameters:
- `use_sim_time` (default: false) - Use simulation time
- `window_title` - Custom window title

## 🐛 Troubleshooting

### Common Issues

**1. "No module named 'coffee_voice_agent_msgs'"**
```bash
# Ensure the messages package is built and sourced
colcon build --packages-select coffee_voice_agent_msgs
source install/setup.bash
```

**2. Qt High DPI Warnings**
```bash
# Set environment variable before running
export QT_AUTO_SCREEN_SCALE_FACTOR=1
ros2 run coffee_voice_agent_ui voice_agent_monitor
```

**3. No Data in UI Panels**
- Check if voice agent bridge is running: `ros2 node list | grep voice_agent_bridge`
- Verify topic publications: `ros2 topic list | grep voice_agent`
- Check topic data: `ros2 topic echo voice_agent/status`

**4. UI Won't Start**
```bash
# Check PyQt installation
python3 -c "from python_qt_binding.QtWidgets import QApplication; print('PyQt OK')"

# Check ROS2 dependencies
ros2 pkg deps coffee_voice_agent_ui
```

### Debug Mode

Run with debug logging for detailed information:
```bash
ros2 run coffee_voice_agent_ui voice_agent_monitor --ros-args --log-level debug
```

## 🔗 Related Packages

- **coffee_voice_agent** - Core voice agent implementation
- **coffee_voice_agent_msgs** - ROS2 message definitions
- **coffee_vision_ui** - Camera monitoring interface
- **coffee_expressions** - Emotion and expression control

## 📝 Development

### Adding New Widgets

1. Create widget in `coffee_voice_agent_ui/widgets/`
2. Inherit from `QWidget` and implement required interfaces
3. Add to main layout in `voice_agent_monitor_app.py`
4. Connect to ROS2 data streams as needed

### Extending Analytics

1. Add data collection in existing widgets' `get_analytics_data()` methods
2. Update `AnalyticsWidget` to process new data types
3. Create visualizations in analytics panel

## 📄 License

TODO: License declaration

## 🤝 Contributing

This package is part of the Coffee Robot project. Please follow the project's coding conventions and submit pull requests for review.

---

**For support or questions about the Coffee Voice Agent UI, please refer to the main project documentation or contact the development team.**
