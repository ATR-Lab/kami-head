#!/usr/bin/env python3
"""
Tool Monitor Widget - Function tool execution monitoring

Shows tool usage statistics, execution times, recent activity,
and success rates for all function tools.
"""

from datetime import datetime, timedelta
from collections import deque, defaultdict

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget, 
    QTableWidgetItem, QProgressBar, QFrame, QHeaderView
)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QFont, QColor

from coffee_voice_agent_msgs.msg import ToolEvent
from ..emoji_utils import format_title


class ToolMonitorWidget(QWidget):
    """Widget for monitoring function tool execution and statistics"""
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(450, 500)
        
        # Tool tracking data
        self.active_tools = {}  # tool_name -> start_time
        self.recent_events = deque(maxlen=50)  # Recent tool events
        self.tool_statistics = defaultdict(lambda: {
            'total_calls': 0,
            'successful_calls': 0,
            'failed_calls': 0,
            'total_execution_time': 0.0,
            'last_called': None
        })
        
        self._setup_ui()
        
        # Update timer for execution timing
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_timing)
        self.update_timer.start(100)  # 10 FPS for timing updates
    
    def _setup_ui(self):
        """Set up the tool monitor UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel(format_title('tool_activity', 'TOOL ACTIVITY'))
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Current activity section
        activity_frame = QFrame()
        activity_frame.setFrameStyle(QFrame.Box)
        layout.addWidget(activity_frame)
        
        activity_layout = QVBoxLayout()
        activity_frame.setLayout(activity_layout)
        
        # Active tools header
        active_header = QLabel(format_title('active_tools', 'ACTIVE TOOLS'))
        active_header.setFont(QFont("Arial", 10, QFont.Bold))
        activity_layout.addWidget(active_header)
        
        # Active tools display
        self.active_tools_label = QLabel("None")
        activity_layout.addWidget(self.active_tools_label)
        
        # Total calls today
        stats_layout = QHBoxLayout()
        stats_layout.addWidget(QLabel(format_title('total_today', 'Total today:')))
        self.total_calls_label = QLabel("0 calls")
        stats_layout.addWidget(self.total_calls_label)
        stats_layout.addStretch()
        activity_layout.addLayout(stats_layout)
        
        # Recent activity section
        recent_label = QLabel(format_title('recent_activity', 'RECENT ACTIVITY'))
        recent_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(recent_label)
        
        # Recent activity table
        self.recent_table = QTableWidget()
        self.recent_table.setColumnCount(4)
        self.recent_table.setHorizontalHeaderLabels(["Time", "Tool", "Status", "Duration"])
        
        # Configure table
        header = self.recent_table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)  # Time
        header.setSectionResizeMode(1, QHeaderView.Stretch)           # Tool 
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)  # Status
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)  # Duration
        
        self.recent_table.setMaximumHeight(150)
        layout.addWidget(self.recent_table)
        
        # Usage statistics section
        stats_label = QLabel(format_title('usage_statistics', 'USAGE STATISTICS'))
        stats_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(stats_label)
        
        # Statistics table
        self.stats_table = QTableWidget()
        self.stats_table.setColumnCount(5)
        self.stats_table.setHorizontalHeaderLabels(["Tool", "Calls", "Success Rate", "Avg Time", "Last Used"])
        
        # Configure stats table
        stats_header = self.stats_table.horizontalHeader()
        stats_header.setStretchLastSection(True)
        stats_header.setSectionResizeMode(0, QHeaderView.Stretch)           # Tool
        stats_header.setSectionResizeMode(1, QHeaderView.ResizeToContents)  # Calls
        stats_header.setSectionResizeMode(2, QHeaderView.ResizeToContents)  # Success Rate
        stats_header.setSectionResizeMode(3, QHeaderView.ResizeToContents)  # Avg Time
        stats_header.setSectionResizeMode(4, QHeaderView.ResizeToContents)  # Last Used
        
        layout.addWidget(self.stats_table)
    
    def add_tool_event(self, event: ToolEvent):
        """Add a new tool event"""
        timestamp = datetime.now()
        
        if event.status == "started":
            # Track active tool
            self.active_tools[event.tool_name] = timestamp
            
            # Update statistics
            self.tool_statistics[event.tool_name]['total_calls'] += 1
            self.tool_statistics[event.tool_name]['last_called'] = timestamp
            
            # Add to recent events
            self.recent_events.append({
                'timestamp': timestamp,
                'tool_name': event.tool_name,
                'status': 'STARTED',
                'duration': None,
                'parameters': event.parameters,
                'result': event.result
            })
            
        elif event.status in ["completed", "failed"]:
            # Calculate execution time if we tracked the start
            duration = None
            if event.tool_name in self.active_tools:
                start_time = self.active_tools[event.tool_name]
                duration = (timestamp - start_time).total_seconds()
                del self.active_tools[event.tool_name]
                
                # Update statistics
                if event.status == "completed":
                    self.tool_statistics[event.tool_name]['successful_calls'] += 1
                else:
                    self.tool_statistics[event.tool_name]['failed_calls'] += 1
                
                self.tool_statistics[event.tool_name]['total_execution_time'] += duration
            
            # Add to recent events
            self.recent_events.append({
                'timestamp': timestamp,
                'tool_name': event.tool_name,
                'status': event.status.upper(),
                'duration': duration,
                'parameters': event.parameters,
                'result': event.result
            })
        
        # Update displays
        self._update_active_tools_display()
        self._update_recent_activity_table()
        self._update_statistics_table()
        self._update_total_calls()
    
    def _update_active_tools_display(self):
        """Update the active tools display"""
        if self.active_tools:
            active_list = []
            for tool_name, start_time in self.active_tools.items():
                duration = (datetime.now() - start_time).total_seconds()
                active_list.append(f"{tool_name} ({duration:.1f}s)")
            self.active_tools_label.setText(", ".join(active_list))
            self.active_tools_label.setStyleSheet("color: #007bff; font-weight: bold;")
        else:
            self.active_tools_label.setText("None")
            self.active_tools_label.setStyleSheet("color: #6c757d;")
    
    def _update_recent_activity_table(self):
        """Update the recent activity table"""
        # Show last 10 events
        recent_events_to_show = list(self.recent_events)[-10:]
        
        self.recent_table.setRowCount(len(recent_events_to_show))
        
        for row, event in enumerate(reversed(recent_events_to_show)):  # Most recent first
            # Time
            time_str = event['timestamp'].strftime("%H:%M:%S")
            self.recent_table.setItem(row, 0, QTableWidgetItem(time_str))
            
            # Tool name
            self.recent_table.setItem(row, 1, QTableWidgetItem(event['tool_name']))
            
            # Status with color coding
            status_item = QTableWidgetItem(event['status'])
            if event['status'] == 'STARTED':
                status_item.setBackground(QColor("#ffc107"))  # Yellow
            elif event['status'] == 'COMPLETED':
                status_item.setBackground(QColor("#28a745"))  # Green
            elif event['status'] == 'FAILED':
                status_item.setBackground(QColor("#dc3545"))  # Red
            self.recent_table.setItem(row, 2, status_item)
            
            # Duration
            if event['duration'] is not None:
                duration_str = f"{event['duration']:.2f}s"
            else:
                duration_str = "..."
            self.recent_table.setItem(row, 3, QTableWidgetItem(duration_str))
    
    def _update_statistics_table(self):
        """Update the usage statistics table"""
        tools = list(self.tool_statistics.keys())
        self.stats_table.setRowCount(len(tools))
        
        for row, tool_name in enumerate(tools):
            stats = self.tool_statistics[tool_name]
            
            # Tool name
            self.stats_table.setItem(row, 0, QTableWidgetItem(tool_name))
            
            # Total calls
            self.stats_table.setItem(row, 1, QTableWidgetItem(str(stats['total_calls'])))
            
            # Success rate
            if stats['total_calls'] > 0:
                success_rate = (stats['successful_calls'] / stats['total_calls']) * 100
                success_item = QTableWidgetItem(f"{success_rate:.0f}%")
                
                # Color code success rate
                if success_rate >= 90:
                    success_item.setBackground(QColor("#28a745"))  # Green
                elif success_rate >= 70:
                    success_item.setBackground(QColor("#ffc107"))  # Yellow
                else:
                    success_item.setBackground(QColor("#dc3545"))  # Red
            else:
                success_item = QTableWidgetItem("N/A")
            
            self.stats_table.setItem(row, 2, success_item)
            
            # Average execution time
            if stats['successful_calls'] > 0:
                avg_time = stats['total_execution_time'] / stats['successful_calls']
                avg_time_str = f"{avg_time:.2f}s"
            else:
                avg_time_str = "N/A"
            self.stats_table.setItem(row, 3, QTableWidgetItem(avg_time_str))
            
            # Last used
            if stats['last_called']:
                # Show relative time
                time_diff = datetime.now() - stats['last_called']
                if time_diff.total_seconds() < 60:
                    last_used_str = f"{int(time_diff.total_seconds())}s ago"
                elif time_diff.total_seconds() < 3600:
                    last_used_str = f"{int(time_diff.total_seconds() / 60)}m ago"
                else:
                    last_used_str = f"{int(time_diff.total_seconds() / 3600)}h ago"
            else:
                last_used_str = "Never"
            
            self.stats_table.setItem(row, 4, QTableWidgetItem(last_used_str))
    
    def _update_total_calls(self):
        """Update the total calls counter"""
        total_calls = sum(stats['total_calls'] for stats in self.tool_statistics.values())
        self.total_calls_label.setText(f"{total_calls} calls")
    
    def _update_timing(self):
        """Update timing for active tools"""
        if self.active_tools:
            self._update_active_tools_display()
    
    def update_timing(self):
        """Called by main widget for timing updates"""
        self._update_timing()
    
    def get_analytics_data(self):
        """Get tool analytics data"""
        total_calls = sum(stats['total_calls'] for stats in self.tool_statistics.values())
        total_successful = sum(stats['successful_calls'] for stats in self.tool_statistics.values())
        
        # Most used tools
        most_used = sorted(
            self.tool_statistics.items(),
            key=lambda x: x[1]['total_calls'],
            reverse=True
        )[:5]
        
        return {
            'total_tool_calls': total_calls,
            'active_tools_count': len(self.active_tools),
            'overall_success_rate': (total_successful / total_calls * 100) if total_calls > 0 else 0,
            'most_used_tools': most_used,
            'tools_statistics': dict(self.tool_statistics),
            'recent_events_count': len(self.recent_events)
        } 