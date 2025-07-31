#!/usr/bin/env python3
"""
Admin Override Widget - VIP detection and extension monitoring interface

Displays real-time information about VIP user detection and conversation extensions
for debugging and monitoring the intelligent timing system.
"""

from datetime import datetime
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QTableWidget, QTableWidgetItem, QPushButton, QProgressBar
)
from python_qt_binding.QtCore import Qt, pyqtSlot
from python_qt_binding.QtGui import QFont


class AdminOverrideWidget(QWidget):
    """Widget for admin override monitoring functionality"""
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(350, 300)
        
        # Data storage
        self.current_vip_user = None
        self.extension_active = False
        self.extension_minutes_remaining = 0
        self.vip_history = []
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the admin override UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("⚙️ ADMIN OVERRIDE")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Status section
        self._create_status_section(layout)
        
        # VIP History section
        self._create_vip_history_section(layout)
        
        layout.addStretch()
    
    def _create_status_section(self, parent_layout):
        """Create status display section"""
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(status_frame)
        
        status_layout = QVBoxLayout()
        status_frame.setLayout(status_layout)
        
        # Header
        status_header = QLabel("📊 STATUS")
        status_header.setFont(QFont("Arial", 10, QFont.Bold))
        status_layout.addWidget(status_header)
        
        # Current VIP User
        vip_layout = QHBoxLayout()
        vip_layout.addWidget(QLabel("VIP User:"))
        
        self.vip_user_label = QLabel("None")
        self.vip_user_label.setStyleSheet("font-weight: bold; color: #2E8B57;")
        vip_layout.addWidget(self.vip_user_label)
        vip_layout.addStretch()
        status_layout.addLayout(vip_layout)
        
        # Extension Status
        ext_layout = QHBoxLayout()
        ext_layout.addWidget(QLabel("Extension:"))
        
        self.extension_status_label = QLabel("Inactive")
        self.extension_status_label.setStyleSheet("color: #CC5500;")
        ext_layout.addWidget(self.extension_status_label)
        ext_layout.addStretch()
        status_layout.addLayout(ext_layout)
        
        # Extension Progress Bar
        self.extension_progress = QProgressBar()
        self.extension_progress.setVisible(False)
        self.extension_progress.setFormat("%v min remaining")
        status_layout.addWidget(self.extension_progress)
    
    def _create_vip_history_section(self, parent_layout):
        """Create VIP history table"""
        history_frame = QFrame()
        history_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(history_frame)
        
        history_layout = QVBoxLayout()
        history_frame.setLayout(history_layout)
        
        # Header
        history_header = QLabel("📋 VIP HISTORY")
        history_header.setFont(QFont("Arial", 10, QFont.Bold))
        history_layout.addWidget(history_header)
        
        # History table
        self.history_table = QTableWidget()
        self.history_table.setColumnCount(3)
        self.history_table.setHorizontalHeaderLabels(["Time", "User", "Action"])
        self.history_table.setMaximumHeight(100)
        self.history_table.setSelectionBehavior(QTableWidget.SelectRows)
        
        # Set column widths
        header = self.history_table.horizontalHeader()
        header.setStretchLastSection(True)
        self.history_table.setColumnWidth(0, 60)
        self.history_table.setColumnWidth(1, 120)
        
        history_layout.addWidget(self.history_table)
    
    @pyqtSlot(str, list, str, int)
    def update_vip_detection(self, user_identifier, matched_keywords, importance_level, recommended_extension):
        """Update VIP detection display"""
        self.current_vip_user = user_identifier
        self.vip_user_label.setText(user_identifier)
        self.vip_user_label.setStyleSheet("font-weight: bold; color: #2E8B57;")
        
        # Add to history
        current_time = datetime.now().strftime("%H:%M")
        action = f"VIP detected (+{recommended_extension}min)"
        
        self._add_history_entry(current_time, user_identifier, action)
    
    @pyqtSlot(str, int, str, str)
    def update_extension_event(self, action, extension_minutes, reason, granted_by):
        """Update extension event display"""
        if action == "granted":
            self.extension_active = True
            self.extension_minutes_remaining = extension_minutes
            
            self.extension_status_label.setText(f"Active ({extension_minutes}min)")
            self.extension_status_label.setStyleSheet("font-weight: bold; color: #2E8B57;")
            
            # Show and update progress bar
            self.extension_progress.setVisible(True)
            self.extension_progress.setRange(0, extension_minutes)
            self.extension_progress.setValue(extension_minutes)
            
            # Add to history
            current_time = datetime.now().strftime("%H:%M")
            user = self.current_vip_user or "System"
            action_text = f"Extended {extension_minutes}min"
            
            self._add_history_entry(current_time, user, action_text)
    
    def _add_history_entry(self, time_str, user_str, action_str):
        """Add entry to VIP history table"""
        # Limit history to last 5 entries
        if self.history_table.rowCount() >= 5:
            self.history_table.removeRow(0)
        
        # Add new row
        row_position = self.history_table.rowCount()
        self.history_table.insertRow(row_position)
        
        # Set items
        self.history_table.setItem(row_position, 0, QTableWidgetItem(time_str))
        self.history_table.setItem(row_position, 1, QTableWidgetItem(user_str))
        self.history_table.setItem(row_position, 2, QTableWidgetItem(action_str))
        
        # Auto-scroll to bottom
        self.history_table.scrollToBottom()
    
    @pyqtSlot()
    def reset_vip_status(self):
        """Reset VIP status when conversation ends"""
        self.current_vip_user = None
        self.vip_user_label.setText("None")
        self.vip_user_label.setStyleSheet("color: #666666;")
        
        self.extension_active = False
        self.extension_status_label.setText("Inactive")
        self.extension_status_label.setStyleSheet("color: #CC5500;")
        
        self.extension_progress.setVisible(False) 