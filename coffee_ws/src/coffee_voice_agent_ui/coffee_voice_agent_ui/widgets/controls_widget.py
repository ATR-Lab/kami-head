#!/usr/bin/env python3
"""
Controls Widget - Virtual request testing interface

Provides manual controls for sending virtual coffee requests
for testing and debugging agent functionality.
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
    """Widget for virtual request testing functionality"""
    
    # Signal for sending virtual requests to ROS
    virtual_request_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(400, 200)
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the controls UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("☕ VIRTUAL REQUESTS")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Virtual Request Testing section
        self._create_virtual_request_section(layout)
        
        layout.addStretch()
    
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
        
        # Priority dropdown and send button
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("Priority:"))
        
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["normal", "urgent"])
        priority_layout.addWidget(self.priority_combo)
        
        # Send button
        self.send_request_btn = QPushButton("Send Request")
        self.send_request_btn.clicked.connect(self._send_virtual_request)
        priority_layout.addWidget(self.send_request_btn)
        
        # Tool test button
        self.trigger_tool_btn = QPushButton("Test Tool")
        self.trigger_tool_btn.clicked.connect(self._trigger_tool_test)
        priority_layout.addWidget(self.trigger_tool_btn)
        
        request_layout.addLayout(priority_layout)
    
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
    
    def _trigger_tool_test(self):
        """Send a test tool trigger"""
        # Send virtual request to trigger get_coffee_menu tool
        request_data = {
            "request_type": "NEW_COFFEE_REQUEST",
            "content": "Test Menu Request",
            "priority": "normal"
        }
        self.virtual_request_signal.emit(json.dumps(request_data)) 