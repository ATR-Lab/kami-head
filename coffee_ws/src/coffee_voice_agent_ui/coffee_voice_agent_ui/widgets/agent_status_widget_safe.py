#!/usr/bin/env python3
"""
Agent Status Widget SAFE VERSION - Displays agent overview information

SAFE VERSION: Uses QGridLayout instead of nested QHBoxLayout inside QVBoxLayout
to avoid macOS Qt layout calculation crash in external terminals.

Shows the current agent state, connection status, conversation metrics,
and general system health in a compact overview panel.
"""

import time
from datetime import datetime, timedelta
from collections import deque

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QFrame, QGridLayout, QProgressBar
)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont, QPalette, QColor

from coffee_voice_agent_msgs.msg import AgentStatus


class AgentStatusWidgetSafe(QWidget):
    """SAFE VERSION: Widget for displaying agent overview and status information
    
    Uses QGridLayout instead of nested QHBoxLayouts to avoid macOS crash.
    """
    
    # State colors for visual indication
    STATE_COLORS = {
        'dormant': '#dc3545',      # Red
        'connecting': '#ffc107',   # Yellow
        'active': '#28a745',       # Green
        'speaking': '#007bff',     # Blue
        'disconnecting': '#6c757d' # Gray
    }
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(350, 250)
        
        # Data tracking
        self.current_status = None
        self.connection_status = False
        self.session_start_time = None
        self.conversation_count = 0
        self.uptime_start = datetime.now()
        
        # Status history for analytics
        self.status_history = deque(maxlen=100)
        
        self._setup_ui()
        
        # Update timer for dynamic elements
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_dynamic_elements)
        self.update_timer.start(1000)  # Update every second
    
    def _setup_ui(self):
        """Set up the widget UI using SAFE layout patterns"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("🤖 AGENT STATUS (SAFE)")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Status frame - KEEP this as it works fine
        self.status_frame = QFrame()
        self.status_frame.setFrameStyle(QFrame.Box)
        layout.addWidget(self.status_frame)
        
        # SAFE CHANGE: Use QGridLayout instead of QVBoxLayout with nested QHBoxLayouts
        status_layout = QGridLayout()
        self.status_frame.setLayout(status_layout)
        
        # Row 0: Current state display
        status_layout.addWidget(QLabel("State:"), 0, 0)
        self.state_label = QLabel("UNKNOWN")
        self.state_label.setAlignment(Qt.AlignCenter)
        # SAFE CHANGE: Use QFont.setBold() instead of font-weight in stylesheet
        font = self.state_label.font()
        font.setBold(True)
        self.state_label.setFont(font)
        self.state_label.setStyleSheet("padding: 4px 8px; border-radius: 4px;")  # No font-weight
        status_layout.addWidget(self.state_label, 0, 1)
        
        # Row 1: Connection status
        status_layout.addWidget(QLabel("Connection:"), 1, 0)
        self.connection_label = QLabel("❌ Disconnected")
        status_layout.addWidget(self.connection_label, 1, 1)
        
        # Row 2: Session info
        status_layout.addWidget(QLabel("Session:"), 2, 0)
        self.session_label = QLabel("No active session")
        status_layout.addWidget(self.session_label, 2, 1)
        
        # Row 3: Uptime
        status_layout.addWidget(QLabel("Uptime:"), 3, 0)
        self.uptime_label = QLabel("00:00:00")
        status_layout.addWidget(self.uptime_label, 3, 1)
        
        # Row 4: Separator - span across both columns
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        status_layout.addWidget(separator, 4, 0, 1, 2)  # span 2 columns
        
        # Row 5-7: Additional info - continue with grid layout
        status_layout.addWidget(QLabel("Phase:"), 5, 0)
        self.phase_label = QLabel("-")
        status_layout.addWidget(self.phase_label, 5, 1)
        
        status_layout.addWidget(QLabel("Last Tool:"), 6, 0)
        self.tool_label = QLabel("-")
        status_layout.addWidget(self.tool_label, 6, 1)
        
        status_layout.addWidget(QLabel("Conversations:"), 7, 0)
        self.conversation_count_label = QLabel("0")
        status_layout.addWidget(self.conversation_count_label, 7, 1)
    
    def update_status(self, status: AgentStatus):
        """Update the widget with new agent status"""
        self.current_status = status
        self.status_history.append((datetime.now(), status))
        
        # Update state display with color coding
        state = status.behavioral_mode.upper()
        self.state_label.setText(state)
        
        # Set state color - SAFE: Keep stylesheet simple, no font-weight
        color = self.STATE_COLORS.get(status.behavioral_mode, '#6c757d')
        self.state_label.setStyleSheet(f"""
            padding: 4px 8px; 
            border-radius: 4px;
            background-color: {color};
            color: white;
        """)
        
        # Update conversation phase
        phase = status.conversation_phase if status.conversation_phase else "idle"
        self.phase_label.setText(phase.title())
        
        # Update last tool used
        tool = status.last_tool_used if status.last_tool_used else "none"
        self.tool_label.setText(tool)
        
        # Track session changes
        if status.behavioral_mode in ['active', 'speaking']:
            if self.session_start_time is None:
                self.session_start_time = datetime.now()
                self.conversation_count += 1
                self.conversation_count_label.setText(str(self.conversation_count))
        elif status.behavioral_mode == 'dormant':
            if self.session_start_time is not None:
                self.session_start_time = None
    
    def update_connection(self, connected: bool):
        """Update connection status"""
        self.connection_status = connected
        
        if connected:
            self.connection_label.setText("✅ Connected")
            self.connection_label.setStyleSheet("color: green;")
        else:
            self.connection_label.setText("❌ Disconnected")
            self.connection_label.setStyleSheet("color: red;")
    
    def _update_dynamic_elements(self):
        """Update time-based elements"""
        # Update uptime
        uptime = datetime.now() - self.uptime_start
        uptime_str = str(uptime).split('.')[0]  # Remove microseconds
        self.uptime_label.setText(uptime_str)
        
        # Update session duration
        if self.session_start_time:
            session_duration = datetime.now() - self.session_start_time
            session_str = str(session_duration).split('.')[0]
            self.session_label.setText(f"Active: {session_str}")
        else:
            self.session_label.setText("No active session")
    
    def get_analytics_data(self):
        """Get analytics data for the analytics widget"""
        return {
            'uptime': datetime.now() - self.uptime_start,
            'conversation_count': self.conversation_count,
            'current_state': self.current_status.behavioral_mode if self.current_status else 'unknown',
            'connection_status': self.connection_status,
            'status_history': list(self.status_history),
            'session_active': self.session_start_time is not None
        } 