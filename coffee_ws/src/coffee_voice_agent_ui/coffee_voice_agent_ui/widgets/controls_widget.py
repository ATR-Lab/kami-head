#!/usr/bin/env python3
"""
Controls Widget - Manual controls and testing interface

Provides manual controls for sending virtual requests, commands,
debugging tools, and system testing functionality.
"""

import json
from datetime import datetime

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QPushButton, QLineEdit, QComboBox, QTextEdit, QGroupBox
)
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtGui import QFont


class ControlsWidget(QWidget):
    """Widget for manual controls and testing functionality"""
    
    # Signals for sending data to ROS
    virtual_request_signal = pyqtSignal(str)
    command_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(350, 500)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the controls UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("⚙️ CONTROLS")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Quick Actions section
        self._create_quick_actions_section(layout)
        
        # Virtual Request Testing section
        self._create_virtual_request_section(layout)
        
        # Command Testing section
        self._create_command_section(layout)
        
        # Debug Tools section
        self._create_debug_section(layout)
        
        layout.addStretch()
    
    def _create_quick_actions_section(self, parent_layout):
        """Create quick actions section"""
        quick_frame = QFrame()
        quick_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(quick_frame)
        
        quick_layout = QVBoxLayout()
        quick_frame.setLayout(quick_layout)
        
        # Header
        quick_header = QLabel("🚀 QUICK ACTIONS")
        quick_header.setFont(QFont("Arial", 10, QFont.Bold))
        quick_layout.addWidget(quick_header)
        
        # Action buttons in grid
        actions_layout = QVBoxLayout()
        
        # Row 1
        row1_layout = QHBoxLayout()
        
        self.end_conversation_btn = QPushButton("End Conversation")
        self.end_conversation_btn.clicked.connect(self._end_conversation)
        row1_layout.addWidget(self.end_conversation_btn)
        
        self.reset_state_btn = QPushButton("Reset State")
        self.reset_state_btn.clicked.connect(self._reset_state)
        row1_layout.addWidget(self.reset_state_btn)
        
        actions_layout.addLayout(row1_layout)
        
        # Row 2
        row2_layout = QHBoxLayout()
        
        self.pause_wake_btn = QPushButton("Pause Wake Word")
        self.pause_wake_btn.clicked.connect(self._pause_wake_word)
        row2_layout.addWidget(self.pause_wake_btn)
        
        self.force_dormant_btn = QPushButton("Force Dormant")
        self.force_dormant_btn.clicked.connect(self._force_dormant)
        row2_layout.addWidget(self.force_dormant_btn)
        
        actions_layout.addLayout(row2_layout)
        
        quick_layout.addLayout(actions_layout)
    
    def _create_virtual_request_section(self, parent_layout):
        """Create virtual request testing section"""
        request_frame = QFrame()
        request_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(request_frame)
        
        request_layout = QVBoxLayout()
        request_frame.setLayout(request_layout)
        
        # Header
        request_header = QLabel("☕ TEST VIRTUAL REQUEST")
        request_header.setFont(QFont("Arial", 10, QFont.Bold))
        request_layout.addWidget(request_header)
        
        # Request type dropdown
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("Type:"))
        
        self.request_type_combo = QComboBox()
        self.request_type_combo.addItems([
            "NEW_COFFEE_REQUEST",
            "ORDER_UPDATE", 
            "ORDER_READY",
            "SYSTEM_NOTIFICATION"
        ])
        type_layout.addWidget(self.request_type_combo)
        request_layout.addLayout(type_layout)
        
        # Content input
        content_layout = QHBoxLayout()
        content_layout.addWidget(QLabel("Content:"))
        
        self.content_input = QLineEdit()
        self.content_input.setPlaceholderText("e.g., Americano (Order a1b2...)")
        content_layout.addWidget(self.content_input)
        request_layout.addLayout(content_layout)
        
        # Priority dropdown
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("Priority:"))
        
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["normal", "urgent"])
        priority_layout.addWidget(self.priority_combo)
        
        # Send button
        self.send_request_btn = QPushButton("Send Request")
        self.send_request_btn.clicked.connect(self._send_virtual_request)
        priority_layout.addWidget(self.send_request_btn)
        
        request_layout.addLayout(priority_layout)
    
    def _create_command_section(self, parent_layout):
        """Create command testing section"""
        command_frame = QFrame()
        command_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(command_frame)
        
        command_layout = QVBoxLayout()
        command_frame.setLayout(command_layout)
        
        # Header
        command_header = QLabel("🎛️ SEND COMMAND")
        command_header.setFont(QFont("Arial", 10, QFont.Bold))
        command_layout.addWidget(command_header)
        
        # Command type dropdown
        cmd_type_layout = QHBoxLayout()
        cmd_type_layout.addWidget(QLabel("Action:"))
        
        self.command_type_combo = QComboBox()
        self.command_type_combo.addItems([
            "end_conversation",
            "pause_wake_word",
            "resume_wake_word", 
            "set_emotion",
            "clear_queue"
        ])
        cmd_type_layout.addWidget(self.command_type_combo)
        command_layout.addLayout(cmd_type_layout)
        
        # Parameters input
        params_layout = QVBoxLayout()
        params_layout.addWidget(QLabel("Parameters (JSON):"))
        
        self.params_input = QTextEdit()
        self.params_input.setMaximumHeight(60)
        self.params_input.setPlaceholderText('{"param": "value"}')
        params_layout.addWidget(self.params_input)
        command_layout.addLayout(params_layout)
        
        # Send command button
        self.send_command_btn = QPushButton("Send Command")
        self.send_command_btn.clicked.connect(self._send_command)
        command_layout.addWidget(self.send_command_btn)
    
    def _create_debug_section(self, parent_layout):
        """Create debug tools section"""
        debug_frame = QFrame()
        debug_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(debug_frame)
        
        debug_layout = QVBoxLayout()
        debug_frame.setLayout(debug_layout)
        
        # Header
        debug_header = QLabel("🔧 DEBUG TOOLS")
        debug_header.setFont(QFont("Arial", 10, QFont.Bold))
        debug_layout.addWidget(debug_header)
        
        # Debug buttons
        debug_buttons_layout = QVBoxLayout()
        
        # Row 1
        debug_row1 = QHBoxLayout()
        
        self.trigger_tool_btn = QPushButton("Trigger Tool")
        self.trigger_tool_btn.clicked.connect(self._trigger_tool_test)
        debug_row1.addWidget(self.trigger_tool_btn)
        
        self.simulate_user_btn = QPushButton("Simulate User")
        self.simulate_user_btn.clicked.connect(self._simulate_user_speech)
        debug_row1.addWidget(self.simulate_user_btn)
        
        debug_buttons_layout.addLayout(debug_row1)
        
        # Row 2
        debug_row2 = QHBoxLayout()
        
        self.connection_test_btn = QPushButton("Connection Test")
        self.connection_test_btn.clicked.connect(self._test_connection)
        debug_row2.addWidget(self.connection_test_btn)
        
        self.export_logs_btn = QPushButton("Export Logs")
        self.export_logs_btn.clicked.connect(self._export_logs)
        debug_row2.addWidget(self.export_logs_btn)
        
        debug_buttons_layout.addLayout(debug_row2)
        
        debug_layout.addLayout(debug_buttons_layout)
    
    def _send_virtual_request(self):
        """Send a virtual request"""
        try:
            request_data = {
                "request_type": self.request_type_combo.currentText(),
                "content": self.content_input.text() or "Test Coffee Order",
                "priority": self.priority_combo.currentText()
            }
            
            request_json = json.dumps(request_data)
            self.virtual_request_signal.emit(request_json)
            
            # Clear input for next request
            self.content_input.clear()
            
        except Exception as e:
            print(f"Error sending virtual request: {e}")
    
    def _send_command(self):
        """Send a command"""
        try:
            # Parse parameters
            params_text = self.params_input.toPlainText().strip()
            if params_text:
                try:
                    parameters = json.loads(params_text)
                except json.JSONDecodeError:
                    parameters = {}
            else:
                parameters = {}
            
            command_data = {
                "action": self.command_type_combo.currentText(),
                "parameters": parameters,
                "timestamp": datetime.now().isoformat()
            }
            
            command_json = json.dumps(command_data)
            self.command_signal.emit(command_json)
            
            # Clear parameters for next command
            self.params_input.clear()
            
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def _end_conversation(self):
        """Send end conversation command"""
        command_data = {
            "action": "end_conversation",
            "parameters": {},
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _reset_state(self):
        """Send reset state command"""
        command_data = {
            "action": "reset_state",
            "parameters": {},
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _pause_wake_word(self):
        """Send pause wake word command"""
        command_data = {
            "action": "pause_wake_word",
            "parameters": {},
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _force_dormant(self):
        """Send force dormant command"""
        command_data = {
            "action": "force_dormant",
            "parameters": {},
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _trigger_tool_test(self):
        """Send a test tool trigger"""
        # Send virtual request to trigger get_coffee_menu tool
        request_data = {
            "request_type": "NEW_COFFEE_REQUEST",
            "content": "Test Menu Request",
            "priority": "normal"
        }
        self.virtual_request_signal.emit(json.dumps(request_data))
    
    def _simulate_user_speech(self):
        """Simulate user speech by sending a typical user query"""
        # This would typically be handled differently, but for testing
        # we can send a command that simulates user input
        command_data = {
            "action": "simulate_user_speech",
            "parameters": {
                "text": "Can you recommend a good coffee?"
            },
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _test_connection(self):
        """Test the connection with a ping command"""
        command_data = {
            "action": "connection_test",
            "parameters": {
                "test_type": "ping"
            },
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data))
    
    def _export_logs(self):
        """Export system logs (placeholder for actual implementation)"""
        # In a real implementation, this would trigger log export
        # For now, just send a command
        command_data = {
            "action": "export_logs",
            "parameters": {
                "format": "json",
                "timerange": "last_hour"
            },
            "timestamp": datetime.now().isoformat()
        }
        self.command_signal.emit(json.dumps(command_data)) 