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
from ..emoji_utils import format_title, get_emoji


class ConversationWidget(QWidget):
    """Widget for displaying real-time conversation flow"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 500)
        
        # Configuration parameters (will be updated by monitor)
        self.user_response_timeout = 15.0  # Default fallback value
        self.max_conversation_time = 180.0  # Default fallback value
        self.config_source = "fallback"  # Track configuration source
        self.config_timestamp = None  # Track when config was last updated
        
        # Conversation tracking
        self.conversation_items = deque(maxlen=100)  # Keep last 100 conversation items
        self.current_phase = "idle"
        self.conversation_start_time = None
        self.last_agent_message_time = None
        
        # Metrics tracking
        self.turn_count = 0
        self.response_times = deque(maxlen=20)  # Last 20 response times
        self.auto_scroll = True
        self.last_agent_message_index = None  # Track last agent message for updates
        
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
        title = QLabel(format_title('conversation', 'CONVERSATION FLOW'))
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # Auto-scroll toggle
        self.auto_scroll_btn = QPushButton(f"{get_emoji('lock_scroll')} Lock Scroll")
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
        
        # Max conversation timeout progress
        self.max_timeout_frame = QFrame()
        max_timeout_layout = QVBoxLayout()
        self.max_timeout_frame.setLayout(max_timeout_layout)
        
        max_timeout_label_layout = QHBoxLayout()
        max_timeout_label_layout.addWidget(QLabel(f"{get_emoji('conversation_time')} Conversation Time:"))
        self.max_timeout_label = QLabel("--")
        max_timeout_label_layout.addWidget(self.max_timeout_label)
        max_timeout_label_layout.addStretch()
        max_timeout_layout.addLayout(max_timeout_label_layout)
        
        self.max_timeout_progress = QProgressBar()
        self.max_timeout_progress.setMaximum(100)
        self.max_timeout_progress.setVisible(False)
        max_timeout_layout.addWidget(self.max_timeout_progress)
        
        layout.addWidget(self.max_timeout_frame)
        
        # Configuration status display
        config_status_layout = QHBoxLayout()
        config_status_layout.addWidget(QLabel("Config:"))
        self.config_status_label = QLabel("Fallback timeouts")
        self.config_status_label.setStyleSheet("color: #ffc107; font-size: 10px;")  # Yellow for fallback
        config_status_layout.addWidget(self.config_status_label)
        config_status_layout.addStretch()
        layout.addLayout(config_status_layout)

        # Timeout progress bar (user response)
        self.timeout_frame = QFrame()
        timeout_layout = QVBoxLayout()
        self.timeout_frame.setLayout(timeout_layout)
        
        timeout_label_layout = QHBoxLayout()
        timeout_label_layout.addWidget(QLabel(f"{get_emoji('user_timeout')} User Response Timeout:"))
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
        clear_btn = QPushButton(f"{get_emoji('clear')} Clear")
        clear_btn.clicked.connect(self._clear_conversation)
        clear_btn.setMaximumWidth(80)
        metrics_layout.addWidget(clear_btn)
        
        layout.addLayout(metrics_layout)
    
    def update_configuration(self, config_data, source="disconnected"):
        """
        Update widget configuration from monitor
        
        Args:
            config_data: Dictionary containing configuration values
            source: Connection status ("connected", "disconnected")
        """
        # Update timeout values
        if 'user_response_timeout' in config_data:
            self.user_response_timeout = float(config_data['user_response_timeout'])
            
        if 'max_conversation_time' in config_data:
            self.max_conversation_time = float(config_data['max_conversation_time'])
        
        # Update configuration tracking
        self.config_source = source
        self.config_timestamp = datetime.now()
        
        # Update configuration status display based on connection
        if source == "connected":
            status_text = f"Bridge Connected ({self.user_response_timeout:.0f}s, {self.max_conversation_time/60:.0f}m)"
            self.config_status_label.setText(status_text)
            self.config_status_label.setStyleSheet("color: #28a745; font-size: 10px;")  # Green for connected
        else:
            status_text = f"Bridge Disconnected ({self.user_response_timeout:.0f}s, {self.max_conversation_time/60:.0f}m)"
            self.config_status_label.setText(status_text)
            self.config_status_label.setStyleSheet("color: #dc3545; font-size: 10px;")  # Red for disconnected
    
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
        
        # Handle agent speech - both starting and completion
        if status.speech_status == "speaking" and status.speech_text:
            # Agent started speaking - add new message with preview text
            self._add_agent_message(status.speech_text, status.emotion)
        elif status.speech_status == "idle" and status.speech_text and self.last_agent_message_index is not None:
            # Agent finished speaking - update last message with full text
            self._update_last_agent_message(status.speech_text)
    
    def add_user_speech(self, speech_text: str):
        """Add user speech to conversation"""
        self._add_conversation_item(f"{get_emoji('user')} USER", speech_text, "#007bff")
        self.turn_count += 1
        self.turns_label.setText(f"Turns: {self.turn_count}")
        
        # Calculate response time if we were waiting for user
        if self.last_agent_message_time:
            response_time = (datetime.now() - self.last_agent_message_time).total_seconds()
            self.response_times.append(response_time)
            self._update_response_metrics()
    
    def add_tool_event(self, event: ToolEvent):
        """Add tool event to conversation"""
        tool_prefix = f"{get_emoji('tool')} TOOL"
        if event.status == "started":
            self._add_conversation_item(tool_prefix, f"{event.tool_name} [STARTED]", "#ffc107")
        elif event.status == "completed":
            # Show abbreviated result
            result_preview = event.result[:50] + "..." if len(event.result) > 50 else event.result
            self._add_conversation_item(tool_prefix, f"{event.tool_name} [COMPLETED] → {result_preview}", "#28a745")
        elif event.status == "failed":
            self._add_conversation_item(tool_prefix, f"{event.tool_name} [FAILED]", "#dc3545")
    
    def _add_agent_message(self, text: str, emotion: str = ""):
        """Add agent message to conversation"""
        emotion_prefix = f"[{emotion}] " if emotion else ""
        self._add_conversation_item(f"{get_emoji('agent')} AGENT", f"{emotion_prefix}{text}", "#28a745")
        self.last_agent_message_time = datetime.now()
        
        # Track this as the last agent message for potential updates
        self.last_agent_message_index = len(self.conversation_items) - 1
        
        # Start timeout tracking
        self.timeout_progress.setVisible(True)
        self.timeout_progress.setValue(0)
    
    def _update_last_agent_message(self, full_text: str):
        """Update the last agent message with the full text"""
        if self.last_agent_message_index is not None and self.last_agent_message_index < len(self.conversation_items):
            # Get the last agent message item
            timestamp, role, old_text, color = self.conversation_items[self.last_agent_message_index]
            
            # Extract emotion prefix from old text if present
            emotion_prefix = ""
            if old_text.startswith("[") and "]" in old_text:
                end_bracket = old_text.find("]")
                emotion_prefix = old_text[:end_bracket + 2]  # Include space after ]
            
            # Create updated text with same emotion prefix
            updated_text = f"{emotion_prefix}{full_text}"
            
            # Update the item in the deque
            self.conversation_items[self.last_agent_message_index] = (timestamp, role, updated_text, color)
            
            # Rebuild entire conversation display to show update
            self._refresh_conversation_display()
    
    def _refresh_conversation_display(self):
        """Refresh the entire conversation display to show updates"""
        # Clear current display
        self.conversation_text.clear()
        
        # Re-add all conversation items
        for timestamp, role, text, color in self.conversation_items:
            time_str = timestamp.strftime("%H:%M:%S")
            formatted_text = f"<span style='color: {color}; font-weight: bold;'>[{time_str}] {role}:</span> {text}<br>"
            
            cursor = self.conversation_text.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertHtml(formatted_text)
        
        # Auto-scroll if enabled
        if self.auto_scroll:
            scrollbar = self.conversation_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
    
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
            self.auto_scroll_btn.setText(f"{get_emoji('lock_scroll')} Lock Scroll")
            # Scroll to bottom when re-enabling
            scrollbar = self.conversation_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        else:
            self.auto_scroll_btn.setText(f"{get_emoji('auto_scroll')} Auto Scroll")
    
    def _clear_conversation(self):
        """Clear the conversation display"""
        self.conversation_text.clear()
        self.conversation_items.clear()
        self.turn_count = 0
        self.turns_label.setText("Turns: 0")
        self.response_times.clear()
        self.last_agent_message_index = None  # Reset agent message tracking
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
        
        # Update max conversation timeout progress
        if self.conversation_start_time:
            conversation_elapsed = (datetime.now() - self.conversation_start_time).total_seconds()
            conversation_remaining = max(0, self.max_conversation_time - conversation_elapsed)
            
            if conversation_elapsed > 0:
                conversation_progress = int((conversation_elapsed / self.max_conversation_time) * 100)
                self.max_timeout_progress.setValue(min(conversation_progress, 100))
                self.max_timeout_progress.setVisible(True)
                
                # Format time display
                elapsed_minutes = int(conversation_elapsed // 60)
                elapsed_seconds = int(conversation_elapsed % 60)
                total_minutes = int(self.max_conversation_time // 60)
                
                if conversation_remaining > 0:
                    remaining_minutes = int(conversation_remaining // 60)
                    remaining_seconds = int(conversation_remaining % 60)
                    self.max_timeout_label.setText(f"{elapsed_minutes}:{elapsed_seconds:02d} / {total_minutes}:00 ({remaining_minutes}:{remaining_seconds:02d} left)")
                else:
                    self.max_timeout_label.setText(f"{elapsed_minutes}:{elapsed_seconds:02d} / {total_minutes}:00 (Limit reached)")
                
                # Color coding with warnings
                if conversation_progress < 75:  # Green: 0-75% (0-2:15 for 3min limit)
                    self.max_timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
                elif conversation_progress < 90:  # Yellow: 75-90% (2:15-2:42 for 3min limit)
                    self.max_timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #ffc107; }")
                else:  # Red: 90-100% (2:42-3:00 for 3min limit)
                    self.max_timeout_progress.setStyleSheet("QProgressBar::chunk { background-color: #dc3545; }")
            else:
                self.max_timeout_progress.setVisible(False)
        else:
            self.max_timeout_progress.setVisible(False)
            self.max_timeout_label.setText("--")
    
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
        # Calculate timeout status if conversation is active
        approaching_user_timeout = False
        approaching_max_timeout = False
        max_timeout_reached = False
        
        if self.conversation_start_time:
            conversation_elapsed = (datetime.now() - self.conversation_start_time).total_seconds()
            
            # Check if approaching max conversation timeout
            max_progress = (conversation_elapsed / self.max_conversation_time) * 100
            approaching_max_timeout = max_progress >= 75  # Warning at 75%
            max_timeout_reached = conversation_elapsed >= self.max_conversation_time
            
            # Check if approaching user response timeout
            if self.last_agent_message_time:
                user_elapsed = (datetime.now() - self.last_agent_message_time).total_seconds()
                user_progress = (user_elapsed / self.user_response_timeout) * 100
                approaching_user_timeout = user_progress >= 75
        
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
            'total_messages': len(self.conversation_items),
            # Timeout analytics data
            'user_response_timeout': self.user_response_timeout,
            'max_conversation_time': self.max_conversation_time,
            'approaching_user_timeout': approaching_user_timeout,
            'approaching_max_timeout': approaching_max_timeout,
            'max_timeout_reached': max_timeout_reached,
            'config_source': self.config_source,
            'config_timestamp': self.config_timestamp
        } 