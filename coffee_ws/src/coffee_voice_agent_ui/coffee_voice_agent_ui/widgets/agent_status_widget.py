#!/usr/bin/env python3
"""
Agent Status Widget - Displays agent overview information

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


class AgentStatusWidget(QWidget):
    """Widget for displaying agent overview and status information"""
    
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
        """Set up the widget UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("🤖 AGENT STATUS")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Status frame
        self.status_frame = QFrame()
        self.status_frame.setFrameStyle(QFrame.Box)
        layout.addWidget(self.status_frame)
        
        status_layout = QVBoxLayout()
        self.status_frame.setLayout(status_layout)
        
        # Current state display
        state_layout = QHBoxLayout()
        state_layout.addWidget(QLabel("State:"))
        self.state_label = QLabel("UNKNOWN")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-weight: bold; padding: 4px 8px; border-radius: 4px;")
        state_layout.addWidget(self.state_label)
        state_layout.addStretch()
        status_layout.addLayout(state_layout)
        
        # Connection status
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("Connection:"))
        self.connection_label = QLabel("❌ Disconnected")
        conn_layout.addWidget(self.connection_label)
        conn_layout.addStretch()
        status_layout.addLayout(conn_layout)
        
        # Session info
        session_layout = QHBoxLayout()
        session_layout.addWidget(QLabel("Session:"))
        self.session_label = QLabel("No active session")
        session_layout.addWidget(self.session_label)
        session_layout.addStretch()
        status_layout.addLayout(session_layout)
        
        # Uptime
        uptime_layout = QHBoxLayout()
        uptime_layout.addWidget(QLabel("Uptime:"))
        self.uptime_label = QLabel("00:00:00")
        uptime_layout.addWidget(self.uptime_label)
        uptime_layout.addStretch()
        status_layout.addLayout(uptime_layout)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        status_layout.addWidget(separator)
        
        # Additional info grid
        info_layout = QGridLayout()
        
        info_layout.addWidget(QLabel("Phase:"), 0, 0)
        self.phase_label = QLabel("-")
        info_layout.addWidget(self.phase_label, 0, 1)
        
        info_layout.addWidget(QLabel("Last Tool:"), 1, 0)
        self.tool_label = QLabel("-")
        info_layout.addWidget(self.tool_label, 1, 1)
        
        info_layout.addWidget(QLabel("Conversations:"), 2, 0)
        self.conversation_count_label = QLabel("0")
        info_layout.addWidget(self.conversation_count_label, 2, 1)
        
        status_layout.addLayout(info_layout)
    
    def update_status(self, status: AgentStatus):
        """Update the widget with new agent status"""
        self.current_status = status
        self.status_history.append((datetime.now(), status))
        
        # Update state display with color coding
        state = status.behavioral_mode.upper()
        self.state_label.setText(state)
        
        # Set state color
        color = self.STATE_COLORS.get(status.behavioral_mode, '#6c757d')
        self.state_label.setStyleSheet(f"""
            font-weight: bold; 
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