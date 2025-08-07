# Coffee Voice Agent UI

A comprehensive PyQt-based monitoring dashboard for the Coffee Voice Agent system. This package provides real-time visualization and control of voice agent status, emotions, tool usage, conversation flow, and system analytics.

![Coffee Voice Agent Monitor](https://img.shields.io/badge/ROS2-Jazzy-blue) ![PyQt5](https://img.shields.io/badge/UI-PyQt5-green) ![Platform](https://img.shields.io/badge/Platform-Ubuntu%20%7C%20macOS-brightgreen) ![License](https://img.shields.io/badge/License-TODO-red)

## 🎯 Overview

The Coffee Voice Agent UI is a standalone ROS2 application that provides comprehensive monitoring and control capabilities for the coffee robot's voice interaction system. It connects to the voice agent bridge to display real-time status information, conversation transcripts, emotional states, and system performance metrics.

### Key Features

- **🤖 Real-time Agent Status**: Monitor connection state, behavioral mode, and session information
- **🎭 Emotion Visualization**: Track current emotions, transitions, and emotional journey
- **💬 Live Conversation Flow**: Real-time transcript with user speech and agent responses
- **🔧 Tool Activity Monitor**: Function tool execution tracking with performance metrics
- **📊 Analytics Dashboard**: Usage statistics, performance trends, and system health
- **👑 Admin Override Monitor**: VIP user detection, conversation extensions, and session management
- **⚙️ Manual Controls**: Testing interface for virtual requests and debug commands

## 📋 Prerequisites

- **ROS2 Jazzy** (or compatible version)
- **Python 3.8+**
- **PyQt5** (installed via `python_qt_binding`)
- **coffee_voice_agent_msgs** package
- **coffee_voice_agent** package (for full system operation)

### Platform Requirements

#### Ubuntu 20.04+ / Linux
- Standard ROS2 installation with `python_qt_binding`
- All emoji and Unicode characters supported natively

#### macOS 10.15+ / Apple Silicon
- **RoboStack** environment recommended for ROS2 on macOS
- **Mamba/Conda** package manager (via Miniforge)
- Qt environment variables automatically configured for stability
- Platform-specific emoji handling implemented for compatibility

## 🚀 Installation

### 1. Linux/Ubuntu Installation

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_ws/src

# Build the package
cd ..
colcon build --packages-select coffee_voice_agent_ui

# Source the workspace
source install/setup.bash
```

### 2. macOS Installation (RoboStack)

```bash
# Activate your ROS2 mamba environment
mamba activate ros_env

# Navigate to your workspace
cd ~/your_ros2_ws

# Build the package
colcon build --packages-select coffee_voice_agent_ui

# Source the workspace
source install/setup.bash
```

**Note for macOS**: The application automatically configures Qt environment variables for optimal stability and cross-platform emoji compatibility.

### 3. Verify Installation

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
├── Platform-Specific Initialization (Qt environment, emoji system)
├── Left Column Container (vertical layout)
│   ├── Agent Status Widget (connection, state, metrics)
│   ├── Emotion Display Widget (current emotion, transitions, timeline)
│   └── Admin Override Widget (VIP detection, extensions, history)
├── Conversation Widget (live transcript, timeouts, turn tracking)
├── Tool Monitor Widget (function tool usage, performance)
├── Analytics Widget (usage trends, performance metrics)
└── Virtual Request Widget (manual testing, virtual coffee requests)
```

### Cross-Platform Components

**Emoji Management System (`emoji_utils.py`)**:
```
EmojiManager
├── Platform Detection (macOS vs Linux/Ubuntu)
├── Emoji Mapping (Unicode ↔ Text alternatives)
├── Widget Integration (automatic emoji replacement)
└── Consistent UI Experience (cross-platform compatibility)
```

### ROS2 Integration

The monitor subscribes to these topics:
- `voice_agent/status` (AgentStatus) - Unified agent state information
- `voice_agent/tool_events` (ToolEvent) - Function tool execution events
- `voice_agent/user_speech` (String) - User speech transcriptions
- `voice_agent/vip_detections` (VipDetection) - VIP user detection events
- `voice_agent/extension_events` (ExtensionEvent) - Conversation extension events
- `voice_agent/connected` (Bool) - Connection status updates

The monitor publishes to these topics:
- `voice_agent/virtual_requests` (String) - Test virtual coffee requests

## 📊 Dashboard Layout

```
┌─────────────────┬─────────────────┬─────────────────┐
│ 🤖 Agent Status │ 💬 Conversation │ 📊 Analytics    │
│ • State & Mode  │   Flow          │ • Performance   │
│ • Connection    │ • Live Chat     │ • Usage Stats   │
│ • Session Info  │ • Timeouts      │ • Trends        │
├─────────────────┼─────────────────┼─────────────────┤
│ 🎭 Emotion      │ 🔧 Tool Monitor │ ☕ Virtual      │
│   Center        │ • Active Tools  │   Requests      │
│ • Current       │ • Recent Calls  │ • Test Orders   │
│ • Transitions   │ • Statistics    │ • Debug Tools   │
├─────────────────┼─────────────────┼─────────────────┤
│ 👑 Admin        │                 │                 │
│   Override      │                 │                 │
│ • VIP Status    │                 │                 │
│ • Extensions    │                 │                 │
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

### Admin Override Panel
- **VIP status display**: Real-time VIP user detection with status indicators
- **Extension monitoring**: Active conversation extensions with progress bars
- **VIP history table**: Recent VIP detections with timestamps and actions
- **Session management**: Visual feedback for unlimited VIP sessions vs timed regular sessions
- **Extension tracking**: Shows who granted extensions (auto_vip_detection, tool, manual)

### Virtual Request Panel
- **Test virtual requests**: Simulate coffee orders with different priorities
- **Request type selection**: NEW_COFFEE_REQUEST, ORDER_READY, ORDER_UPDATE, etc.
- **Priority control**: Normal, urgent priority testing
- **Tool testing**: Trigger function tools for development and debugging

## 👑 VIP Session Monitoring

The UI provides comprehensive monitoring of VIP user detection and session management:

### **🔍 VIP Detection Display**

The Admin Override widget shows real-time VIP detection status:

```
⚙️ ADMIN OVERRIDE
┌─────────────────────────────────┐
│ STATUS                          │
│ VIP User: ✅ Sui Foundation     │
│ Extension: [██████████] Active   │
│                                 │
│ VIP HISTORY                     │
│ Time    User           Action   │
│ 20:38   Sui Foundation VIP      │
│ 20:38   System        Extension │
└─────────────────────────────────┘
```

### **📊 Session Management Indicators**

- **VIP Status Light**: Green when VIP user is detected
- **Extension Progress**: Visual progress bar for active extensions  
- **Session Type Display**: "Unlimited" for VIP sessions, timer for regular users
- **History Table**: Chronological log of VIP detections and extension events

### **🎯 Real-Time Updates**

The UI automatically updates when:
- VIP users are detected (via `/voice_agent/vip_detections` topic)
- Conversation extensions are granted (via `/voice_agent/extension_events` topic)
- Sessions transition between regular and VIP modes
- Extension timers expire or are renewed

### **🔄 Message Integration**

**VipDetection Message Processing:**
```python
# Displays matched keywords, importance level, and recommendations
user_identifier: "Sui Foundation"
matched_keywords: ["sui foundation"]  
importance_level: "vip"
recommended_extension_minutes: 0  # 0 = unlimited
```

**ExtensionEvent Message Processing:**
```python
# Shows extension status and progress
action: "vip_session"           # or "granted", "expired"
extension_minutes: 0            # 0 = unlimited
granted_by: "auto_vip_detection" # or "tool", "manual"
```

### **⚙️ UI Layout Integration**

The Admin Override widget is positioned in the left column for easy monitoring:
- **Compact Design**: Fixed 400x300 size to fit alongside other status widgets
- **Always Visible**: Positioned in main dashboard for constant visibility
- **Status Reset**: Automatically clears when conversations end

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

**macOS-specific (automatically configured):**
- `QT_QPA_PLATFORM=cocoa` - Native macOS Qt platform
- `QT_MAC_WANTS_LAYER=1` - Layer-backed rendering for stability
- `QT_XCB_GL_INTEGRATION=none` - Disable X11/GL conflicts

### Parameters
The monitor can be configured through ROS2 parameters:
- `use_sim_time` (default: false) - Use simulation time
- `window_title` - Custom window title

### Cross-Platform Emoji System
The UI includes a platform-specific emoji handling system:
- **Ubuntu/Linux**: Native emoji support with full Unicode rendering
- **macOS**: Text-based alternatives for emojis to ensure stability and compatibility
- Automatic platform detection and appropriate emoji mapping
- Consistent visual experience across operating systems

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

**4. Admin Override Widget Shows No VIP Data**
- Verify VIP detection topics are active: `ros2 topic list | grep vip`
- Check VIP detection messages: `ros2 topic echo voice_agent/vip_detections`
- Check extension events: `ros2 topic echo voice_agent/extension_events`
- Ensure voice agent is running with VIP detection enabled

**5. UI Won't Start**
```bash
# Check PyQt installation
python3 -c "from python_qt_binding.QtWidgets import QApplication; print('PyQt OK')"

# Check ROS2 dependencies
ros2 pkg deps coffee_voice_agent_ui
```

**6. macOS-Specific Issues**

**"Bus error: 10" on macOS:**
- This was a known issue with emoji rendering on macOS external terminals
- **Solution**: Automatically resolved by the built-in platform-specific emoji system
- No manual intervention required - the UI now works reliably on all macOS terminals

**RoboStack Environment Issues:**
```bash
# Ensure you're in the correct mamba environment
mamba activate ros_env

# Verify ROS2 installation
ros2 --version

# Check Qt platform detection
echo $QT_QPA_PLATFORM  # Should show 'cocoa' on macOS
```

**Git/Homebrew Conflicts During Setup:**
- **Issue**: `git version did not run successfully` with `Symbol not found: _iconv`
- **Cause**: Homebrew git conflicts with conda libiconv library
- **Solution**: Automatically handled by setup script (pre-downloads git packages)
- **Manual fix if needed**: `mamba install git -c conda-forge`

**PyAudio Installation Issues:**
- **Ubuntu**: Automatically installs via pip after installing `portaudio19-dev`
- **macOS**: Requires Homebrew PortAudio, then installs via pip (handled automatically)
- **Manual fix if needed**: 
  - Ubuntu: `sudo apt install portaudio19-dev && pip install PyAudio`
  - macOS: `brew install portaudio && pip install PyAudio`

**Terminal Compatibility:**
- Works in VS Code integrated terminal ✅
- Works in macOS Terminal.app ✅  
- Works in iTerm2 ✅
- Works in external terminal applications ✅

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
