#!/usr/bin/env python3
"""
Analytics Widget - Performance and usage analytics

Shows session statistics, performance metrics, usage trends,
and system analytics for the voice agent system.
"""

from datetime import datetime, timedelta
from collections import defaultdict

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QProgressBar, QGridLayout, QScrollArea
)
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont, QPainter, QColor, QPen


class AnalyticsWidget(QWidget):
    """Widget for displaying performance and usage analytics"""
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(350, 500)
        
        # Analytics data
        self.session_data = {
            'conversations_today': 0,
            'avg_duration': timedelta(0),
            'success_rate': 100.0,
            'peak_activity_hour': 'N/A'
        }
        
        self.interaction_data = {
            'popular_interactions': [],
            'emotion_trends': {},
            'tool_usage_distribution': {}
        }
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the analytics UI"""
        # Main scroll area for analytics
        scroll = QScrollArea()
        scroll_widget = QWidget()
        scroll.setWidget(scroll_widget)
        scroll.setWidgetResizable(True)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)
        main_layout.addWidget(scroll)
        
        layout = QVBoxLayout()
        scroll_widget.setLayout(layout)
        
        # Title
        title = QLabel("📊 ANALYTICS")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Session Performance section
        self._create_session_performance_section(layout)
        
        # Popular Interactions section
        self._create_popular_interactions_section(layout)
        
        # Emotion Trends section
        self._create_emotion_trends_section(layout)
        
        # System Metrics section
        self._create_system_metrics_section(layout)
        
        layout.addStretch()
    
    def _create_session_performance_section(self, parent_layout):
        """Create session performance section"""
        # Session Performance
        perf_frame = QFrame()
        perf_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(perf_frame)
        
        perf_layout = QVBoxLayout()
        perf_frame.setLayout(perf_layout)
        
        # Header
        perf_header = QLabel("📈 SESSION PERFORMANCE")
        perf_header.setFont(QFont("Arial", 10, QFont.Bold))
        perf_layout.addWidget(perf_header)
        
        # Metrics grid
        metrics_layout = QGridLayout()
        
        # Conversations today
        metrics_layout.addWidget(QLabel("Conversations today:"), 0, 0)
        self.conversations_label = QLabel("0")
        self.conversations_label.setFont(QFont("Arial", 12, QFont.Bold))
        metrics_layout.addWidget(self.conversations_label, 0, 1)
        
        # Average duration
        metrics_layout.addWidget(QLabel("Avg duration:"), 1, 0)
        self.avg_duration_label = QLabel("00:00")
        self.avg_duration_label.setFont(QFont("Arial", 12, QFont.Bold))
        metrics_layout.addWidget(self.avg_duration_label, 1, 1)
        
        # Success rate
        metrics_layout.addWidget(QLabel("Success rate:"), 2, 0)
        self.success_rate_label = QLabel("100%")
        self.success_rate_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.success_rate_label.setStyleSheet("color: #28a745;")
        metrics_layout.addWidget(self.success_rate_label, 2, 1)
        
        perf_layout.addLayout(metrics_layout)
        
        # Success rate progress bar
        self.success_progress = QProgressBar()
        self.success_progress.setMaximum(100)
        self.success_progress.setValue(100)
        self.success_progress.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
        perf_layout.addWidget(self.success_progress)
    
    def _create_popular_interactions_section(self, parent_layout):
        """Create popular interactions section"""
        interactions_frame = QFrame()
        interactions_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(interactions_frame)
        
        interactions_layout = QVBoxLayout()
        interactions_frame.setLayout(interactions_layout)
        
        # Header
        interactions_header = QLabel("🎯 POPULAR INTERACTIONS")
        interactions_header.setFont(QFont("Arial", 10, QFont.Bold))
        interactions_layout.addWidget(interactions_header)
        
        # Popular interactions list
        self.interactions_layout = QVBoxLayout()
        interactions_layout.addLayout(self.interactions_layout)
        
        # Default content
        self._update_popular_interactions([
            ("Coffee menu requests", 45),
            ("Drink recommendations", 30),
            ("Ordering instructions", 25)
        ])
    
    def _create_emotion_trends_section(self, parent_layout):
        """Create emotion trends section"""
        emotion_frame = QFrame()
        emotion_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(emotion_frame)
        
        emotion_layout = QVBoxLayout()
        emotion_frame.setLayout(emotion_layout)
        
        # Header
        emotion_header = QLabel("🎭 EMOTION TRENDS")
        emotion_header.setFont(QFont("Arial", 10, QFont.Bold))
        emotion_layout.addWidget(emotion_header)
        
        # Trends info
        self.peak_happiness_label = QLabel("Peak happiness: 2:30 PM")
        emotion_layout.addWidget(self.peak_happiness_label)
        
        self.most_curious_label = QLabel("Most curious: Morning hours")
        emotion_layout.addWidget(self.most_curious_label)
        
        self.dominant_emotion_label = QLabel("Dominant emotion: friendly (60%)")
        emotion_layout.addWidget(self.dominant_emotion_label)
    
    def _create_system_metrics_section(self, parent_layout):
        """Create system metrics section"""
        metrics_frame = QFrame()
        metrics_frame.setFrameStyle(QFrame.Box)
        parent_layout.addWidget(metrics_frame)
        
        metrics_layout = QVBoxLayout()
        metrics_frame.setLayout(metrics_layout)
        
        # Header
        metrics_header = QLabel("⚙️ SYSTEM METRICS")
        metrics_header.setFont(QFont("Arial", 10, QFont.Bold))
        metrics_layout.addWidget(metrics_header)
        
        # Metrics grid
        sys_metrics_layout = QGridLayout()
        
        # Message rates
        sys_metrics_layout.addWidget(QLabel("Message rate:"), 0, 0)
        self.message_rate_label = QLabel("2.1/s")
        sys_metrics_layout.addWidget(self.message_rate_label, 0, 1)
        
        # Connection health
        sys_metrics_layout.addWidget(QLabel("Connection:"), 1, 0)
        self.connection_health_label = QLabel("Excellent")
        self.connection_health_label.setStyleSheet("color: #28a745;")
        sys_metrics_layout.addWidget(self.connection_health_label, 1, 1)
        
        # Queue health
        sys_metrics_layout.addWidget(QLabel("Queue delay:"), 2, 0)
        self.queue_delay_label = QLabel("0.2s")
        sys_metrics_layout.addWidget(self.queue_delay_label, 2, 1)
        
        metrics_layout.addLayout(sys_metrics_layout)
    
    def _update_popular_interactions(self, interactions):
        """Update the popular interactions display"""
        # Clear existing widgets
        while self.interactions_layout.count():
            child = self.interactions_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        
        # Add new interaction bars
        for i, (interaction, percentage) in enumerate(interactions[:5]):  # Top 5
            interaction_layout = QHBoxLayout()
            
            # Rank and name
            rank_label = QLabel(f"{i+1}.")
            rank_label.setFixedWidth(20)
            interaction_layout.addWidget(rank_label)
            
            name_label = QLabel(interaction)
            interaction_layout.addWidget(name_label)
            
            # Percentage
            pct_label = QLabel(f"({percentage}%)")
            pct_label.setFixedWidth(40)
            interaction_layout.addWidget(pct_label)
            
            self.interactions_layout.addLayout(interaction_layout)
            
            # Progress bar
            progress = QProgressBar()
            progress.setMaximum(100)
            progress.setValue(percentage)
            progress.setMaximumHeight(10)
            
            # Color based on rank
            if i == 0:
                color = "#28a745"  # Green for #1
            elif i == 1:
                color = "#ffc107"  # Yellow for #2
            elif i == 2:
                color = "#fd7e14"  # Orange for #3
            else:
                color = "#6c757d"  # Gray for others
            
            progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")
            self.interactions_layout.addWidget(progress)
    
    def update_analytics(self, agent_data, tool_data, conversation_data):
        """Update analytics with new data"""
        # Update session performance
        if agent_data:
            self.conversations_label.setText(str(agent_data.get('conversation_count', 0)))
            
            # Update success rate based on connection status
            success_rate = 100.0 if agent_data.get('connection_status', False) else 85.0
            self.success_rate_label.setText(f"{success_rate:.0f}%")
            self.success_progress.setValue(int(success_rate))
            
            # Color code success rate
            if success_rate >= 95:
                color = "#28a745"  # Green
            elif success_rate >= 85:
                color = "#ffc107"  # Yellow
            else:
                color = "#dc3545"  # Red
            
            self.success_rate_label.setStyleSheet(f"color: {color};")
            self.success_progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")
        
        # Update popular interactions from tool data
        if tool_data and 'most_used_tools' in tool_data:
            popular_tools = []
            total_calls = tool_data.get('total_tool_calls', 1)
            
            for tool_name, stats in tool_data['most_used_tools']:
                if total_calls > 0:
                    percentage = int((stats['total_calls'] / total_calls) * 100)
                    # Convert tool names to user-friendly descriptions
                    friendly_name = self._get_friendly_tool_name(tool_name)
                    popular_tools.append((friendly_name, percentage))
            
            if popular_tools:
                self._update_popular_interactions(popular_tools)
        
        # Update conversation metrics
        if conversation_data:
            if conversation_data.get('conversation_duration'):
                duration = conversation_data['conversation_duration']
                duration_str = str(duration).split('.')[0]  # Remove microseconds
                self.avg_duration_label.setText(duration_str)
        
        # Update system metrics with mock data (in real implementation, this would come from actual metrics)
        self._update_system_metrics()
    
    def _get_friendly_tool_name(self, tool_name):
        """Convert tool names to user-friendly descriptions"""
        friendly_names = {
            'get_coffee_menu': 'Coffee menu requests',
            'recommend_drink': 'Drink recommendations',
            'get_ordering_instructions': 'Ordering instructions',
            'get_current_time': 'Time requests',
            'get_current_date': 'Date requests'
        }
        return friendly_names.get(tool_name, tool_name)
    
    def _update_system_metrics(self):
        """Update system metrics (mock implementation)"""
        # In a real implementation, these would be calculated from actual metrics
        import random
        
        # Simulate some variability in metrics
        message_rate = 1.8 + random.random() * 0.6  # 1.8-2.4 range
        self.message_rate_label.setText(f"{message_rate:.1f}/s")
        
        # Queue delay simulation
        queue_delay = random.random() * 0.5  # 0-0.5s range
        self.queue_delay_label.setText(f"{queue_delay:.1f}s")
        
        # Connection health based on message rate
        if message_rate > 2.0:
            self.connection_health_label.setText("Excellent")
            self.connection_health_label.setStyleSheet("color: #28a745;")
        elif message_rate > 1.5:
            self.connection_health_label.setText("Good")
            self.connection_health_label.setStyleSheet("color: #ffc107;")
        else:
            self.connection_health_label.setText("Poor")
            self.connection_health_label.setStyleSheet("color: #dc3545;") 