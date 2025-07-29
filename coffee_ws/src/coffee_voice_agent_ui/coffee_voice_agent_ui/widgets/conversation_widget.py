#!/usr/bin/env python3
"""
Conversation Widget - Real-time conversation flow display

Shows live conversation transcript with user speech, agent responses, tool calls,
conversation metrics, and timeout tracking.
"""

from datetime import datetime, timedelta
from collections import deque

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, 
    QScrollArea, QFrame, QProgressBar, QPushButton
)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont, QTextCursor, QColor

from coffee_voice_agent_msgs.msg import AgentStatus, ToolEvent


class ConversationWidget(QWidget):
    """Widget for displaying real-time conversation flow"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 500)
        
        # Conversation tracking
        self.conversation_items = deque(maxlen=100)  # Keep last 100 conversation items
        self.current_phase = "idle"
        self.conversation_start_time = None
        self.user_response_timeout = 15  # seconds
        self.last_agent_message_time = None
        
        # Metrics tracking
        self.turn_count = 0
        self.response_times = deque(maxlen=20)  # Last 20 response times
        self.auto_scroll = True
        
        self._setup_ui()
        
        # Update timer for timestamps and timeouts
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_dynamic_elements)
        self.update_timer.start(1000)  # Update every second
    
    def _setup_ui(self):
        """Set up the conversation display UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Header with title and controls
        header_layout = QHBoxLayout()
        
        # Title
        title = QLabel("💬 CONVERSATION FLOW")
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # Auto-scroll toggle
        self.auto_scroll_btn = QPushButton("🔒 Lock Scroll")
        self.auto_scroll_btn.setCheckable(True)
        self.auto_scroll_btn.clicked.connect(self._toggle_auto_scroll)
        self.auto_scroll_btn.setMaximumWidth(100)
        header_layout.addWidget(self.auto_scroll_btn)
        
        layout.addLayout(header_layout)
        
        # Status bar
        status_layout = QHBoxLayout()
        
        # Phase and duration
        self.phase_label = QLabel("Phase: idle")
        status_layout.addWidget(self.phase_label)
        
        self.duration_label = QLabel("Duration: --")
        status_layout.addWidget(self.duration_label)
        
        status_layout.addStretch()
        
        # Turn counter
        self.turns_label = QLabel("Turns: 0")
        status_layout.addWidget(self.turns_label)
        
        layout.addLayout(status_layout)
        
        # Timeout progress bar
        self.timeout_frame = QFrame()
        timeout_layout = QVBoxLayout()
        self.timeout_frame.setLayout(timeout_layout)
        
        timeout_label_layout = QHBoxLayout()
        timeout_label_layout.addWidget(QLabel("⏱️ User Response Timeout:"))
        self.timeout_label = QLabel("--")
        timeout_label_layout.addWidget(self.timeout_label)
        timeout_label_layout.addStretch()
        timeout_layout.addLayout(timeout_label_layout)
        
        self.timeout_progress = QProgressBar()
        self.timeout_progress.setMaximum(100)
        self.timeout_progress.setVisible(False)
        timeout_layout.addWidget(self.timeout_progress)
        
        layout.addWidget(self.timeout_frame)
        
        # Main conversation display
        self.conversation_text = QTextEdit()
        self.conversation_text.setReadOnly(True)
        self.conversation_text.setFont(QFont("Consolas", 10))
        layout.addWidget(self.conversation_text)
        
        # Metrics footer
        metrics_layout = QHBoxLayout()
        
        self.avg_response_label = QLabel("Avg Response: --")
        metrics_layout.addWidget(self.avg_response_label)
        
        metrics_layout.addStretch()
        
        # Clear button
        clear_btn = QPushButton("🗑️ Clear")
        clear_btn.clicked.connect(self._clear_conversation)
        clear_btn.setMaximumWidth(80)
        metrics_layout.addWidget(clear_btn)
        
        layout.addLayout(metrics_layout)
    
    def update_agent_state(self, status: AgentStatus):
        """Update conversation state from agent status"""
        new_phase = status.conversation_phase if status.conversation_phase else "idle"
        
        # Track conversation start/end
        if new_phase != "idle" and self.current_phase == "idle":
            self.conversation_start_time = datetime.now()
            self.turn_count = 0
        elif new_phase == "idle" and self.current_phase != "idle":
            self.conversation_start_time = None
        
        self.current_phase = new_phase
        self.phase_label.setText(f"Phase: {new_phase}")
        
        # Handle agent speech
        if status.speech_status == "speaking" and status.speech_text:
            self._add_agent_message(status.speech_text, status.emotion)
    
    def add_user_speech(self, speech_text: str):
        """Add user speech to conversation"""
        self._add_conversation_item("👤 USER", speech_text, "#007bff")
        self.turn_count += 1
        self.turns_label.setText(f"Turns: {self.turn_count}")
        
        # Calculate response time if we were waiting for user
        if self.last_agent_message_time:
            response_time = (datetime.now() - self.last_agent_message_time).total_seconds()
            self.response_times.append(response_time)
            self._update_response_metrics()
    
    def add_tool_event(self, event: ToolEvent):
        """Add tool event to conversation"""
        if event.status == "started":
            self._add_conversation_item("🔧 TOOL", f"{event.tool_name} [STARTED]", "#ffc107")
        elif event.status == "completed":
            # Show abbreviated result
            result_preview = event.result[:50] + "..." if len(event.result) > 50 else event.result
            self._add_conversation_item("🔧 TOOL", f"{event.tool_name} [COMPLETED] → {result_preview}", "#28a745")
        elif event.status == "failed":
            self._add_conversation_item("🔧 TOOL", f"{event.tool_name} [FAILED]", "#dc3545")
    
    def _add_agent_message(self, text: str, emotion: str = ""):
        """Add agent message to conversation"""
        emotion_prefix = f"[{emotion}] " if emotion else ""
        self._add_conversation_item("🤖 AGENT", f"{emotion_prefix}{text}", "#28a745")
        self.last_agent_message_time = datetime.now()
        
        # Start timeout tracking
        self.timeout_progress.setVisible(True)
        self.timeout_progress.setValue(0)
    
    def _add_conversation_item(self, role: str, text: str, color: str):
        """Add an item to the conversation display"""
        timestamp = datetime.now()
        self.conversation_items.append((timestamp, role, text, color))
        
        # Format and display
        time_str = timestamp.strftime("%H:%M:%S")
        formatted_text = f"<span style='color: {color}; font-weight: bold;'>[{time_str}] {role}:</span> {text}<br>"
        
        # Add to text display
        cursor = self.conversation_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertHtml(formatted_text)
        
        # Auto-scroll if enabled
        if self.auto_scroll:
            scrollbar = self.conversation_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
    
    def _toggle_auto_scroll(self):
        """Toggle auto-scroll functionality"""
        self.auto_scroll = not self.auto_scroll
        if self.auto_scroll:
            self.auto_scroll_btn.setText("🔒 Lock Scroll")
            # Scroll to bottom when re-enabling
            scrollbar = self.conversation_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        else:
            self.auto_scroll_btn.setText("📜 Auto Scroll")
    
    def _clear_conversation(self):
        """Clear the conversation display"""
        self.conversation_text.clear()
        self.conversation_items.clear()
        self.turn_count = 0
        self.turns_label.setText("Turns: 0")
        self.response_times.clear()
        self._update_response_metrics()
    
    def _update_dynamic_elements(self):
        """Update time-based elements"""
        # Update conversation duration
        if self.conversation_start_time:
            duration = datetime.now() - self.conversation_start_time
            duration_str = str(duration).split('.')[0]  # Remove microseconds
            self.duration_label.setText(f"Duration: {duration_str}")
        else:
            self.duration_label.setText("Duration: --")
        
        # Update user response timeout
        if self.last_agent_message_time and self.current_phase != "idle":
            elapsed = (datetime.now() - self.last_agent_message_time).total_seconds()
            remaining = max(0, self.user_response_timeout - elapsed)
            
            if remaining > 0:
                progress = int((elapsed / self.user_response_timeout) * 100)
                self.timeout_progress.setValue(min(progress, 100))
                self.timeout_label.setText(f"{remaining:.0f}s left")
                
                # Color coding: green -> yellow -> red
                if progress < 50:
                    self.timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
                elif progress < 80:
                    self.timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #ffc107; }")
                else:
                    self.timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #dc3545; }")
            else:
                self.timeout_progress.setVisible(False)
                self.timeout_label.setText("Timeout reached")
        else:
            self.timeout_progress.setVisible(False)
            self.timeout_label.setText("--")
    
    def _update_response_metrics(self):
        """Update response time metrics"""
        if self.response_times:
            avg_response = sum(self.response_times) / len(self.response_times)
            self.avg_response_label.setText(f"Avg Response: {avg_response:.1f}s")
        else:
            self.avg_response_label.setText("Avg Response: --")
    
    def update_timestamps(self):
        """Update relative timestamps (called by main widget)"""
        # This could update relative timestamps like "2 minutes ago"
        pass
    
    def get_analytics_data(self):
        """Get conversation analytics data"""
        return {
            'conversation_active': self.conversation_start_time is not None,
            'current_phase': self.current_phase,
            'turn_count': self.turn_count,
            'conversation_duration': (
                datetime.now() - self.conversation_start_time
                if self.conversation_start_time else timedelta(0)
            ),
            'average_response_time': (
                sum(self.response_times) / len(self.response_times)
                if self.response_times else 0
            ),
            'total_messages': len(self.conversation_items)
        } 