#!/usr/bin/env python3
"""
Analytics Widget - Performance and usage analytics

Shows session statistics, performance metrics, usage trends,
and system analytics for the voice agent system.

ANALYTICS METHODOLOGY:
======================

This widget provides real-time analytics based on actual system data rather than simulated metrics.
It consumes data from the ROS2 bridge which receives events from the voice agent system.

SUCCESS RATE CALCULATION:
------------------------
The success rate is a weighted combination of two key effectiveness metrics:

1. Tool Success Rate (40% weight):
   - Measures the percentage of function tool calls that complete successfully
   - Based on tool events with status "completed" vs "failed"
   - Indicates how well the agent can execute requested actions

2. Conversation Success Rate (60% weight):
   - Measures the percentage of conversations that end naturally/successfully
   - Natural endings: User says goodbye, task completion
   - Failed endings: Timeouts, unexpected disconnects, very short interactions
   - Indicates user satisfaction and conversation quality

Combined Formula: (Tool Success × 0.4) + (Conversation Success × 0.6)

FALLBACK BEHAVIOR:
- If only one metric is available, uses that metric with appropriate labeling
- If no effectiveness data available, falls back to connection status as basic health indicator
- Labels indicate data source: "85% (tools)", "92% (conv)", "100% (conn)"

DATA SOURCES:
------------
- agent_data: Connection status, emotion states, conversation counts
- tool_data: Tool usage statistics, success/failure rates, response times  
- conversation_data: Session timing, turn counts, conversation outcomes

REAL-TIME UPDATES:
-----------------
- Message rate: Calculated from actual ROS message timestamps
- Popular interactions: Ranked by actual tool usage counts
- Emotion trends: Analyzed from real emotion transition events
- System metrics: Derived from live system performance data

This approach provides meaningful operational insights rather than placeholder metrics.
"""

from datetime import datetime, timedelta
from collections import defaultdict, deque

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
        
        # Real data tracking structures
        self.message_timestamps = deque(maxlen=100)  # For rate calculation
        self.tool_usage_counts = defaultdict(int)    # For popularity ranking
        self.tool_response_times = defaultdict(list) # For response time calculation
        self.emotion_history = []                    # For trend analysis
        self.conversation_sessions = []              # For success rate calculation
        self.connection_events = []                  # For health metrics
        self.user_speech_events = []                 # For interaction tracking
        
        # Success rate tracking - measures actual system effectiveness
        self.tool_success_events = []                # Tool completion/failure events
        self.conversation_outcomes = []              # Natural vs timeout conversation endings
        
        # Session data - now calculated from real events
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
        
        # Current session tracking
        self.current_session_start = None
        self.current_connection_status = False
        self.daily_conversation_count = 0
        
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
        self.avg_duration_label = QLabel("--")
        self.avg_duration_label.setFont(QFont("Arial", 12, QFont.Bold))
        metrics_layout.addWidget(self.avg_duration_label, 1, 1)
        
        # Success rate
        metrics_layout.addWidget(QLabel("Success rate:"), 2, 0)
        self.success_rate_label = QLabel("--")
        self.success_rate_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.success_rate_label.setStyleSheet("color: #28a745;")
        metrics_layout.addWidget(self.success_rate_label, 2, 1)
        
        perf_layout.addLayout(metrics_layout)
        
        # Success rate progress bar
        self.success_progress = QProgressBar()
        self.success_progress.setMaximum(100)
        self.success_progress.setValue(0)
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
        
        # Show "No data" initially
        self._update_popular_interactions([])
    
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
        
        # Trends info - now calculated from real data
        self.peak_happiness_label = QLabel("Peak happiness: --")
        emotion_layout.addWidget(self.peak_happiness_label)
        
        self.most_curious_label = QLabel("Most curious: --")
        emotion_layout.addWidget(self.most_curious_label)
        
        self.dominant_emotion_label = QLabel("Dominant emotion: --")
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
        self.message_rate_label = QLabel("--")
        sys_metrics_layout.addWidget(self.message_rate_label, 0, 1)
        
        # Connection health
        sys_metrics_layout.addWidget(QLabel("Connection:"), 1, 0)
        self.connection_health_label = QLabel("--")
        self.connection_health_label.setStyleSheet("color: #6c757d;")
        sys_metrics_layout.addWidget(self.connection_health_label, 1, 1)
        
        # Tool response time
        sys_metrics_layout.addWidget(QLabel("Avg tool time:"), 2, 0)
        self.tool_response_label = QLabel("--")
        sys_metrics_layout.addWidget(self.tool_response_label, 2, 1)
        
        metrics_layout.addLayout(sys_metrics_layout)
    
    def _update_popular_interactions(self, interactions):
        """Update the popular interactions display"""
        # Clear existing widgets
        while self.interactions_layout.count():
            child = self.interactions_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        
        if not interactions:
            # Show "No data" when no interactions available
            no_data_label = QLabel("No interaction data available")
            no_data_label.setStyleSheet("color: #6c757d; font-style: italic;")
            self.interactions_layout.addWidget(no_data_label)
            return
        
        # Add new interaction bars
        for i, (interaction, count) in enumerate(interactions[:5]):  # Top 5
            interaction_layout = QHBoxLayout()
            
            # Rank and name
            rank_label = QLabel(f"{i+1}.")
            rank_label.setFixedWidth(20)
            interaction_layout.addWidget(rank_label)
            
            name_label = QLabel(interaction)
            interaction_layout.addWidget(name_label)
            
            # Count
            count_label = QLabel(f"({count})")
            count_label.setFixedWidth(40)
            interaction_layout.addWidget(count_label)
            
            self.interactions_layout.addLayout(interaction_layout)
            
            # Progress bar (show relative usage)
            if interactions:
                max_count = max(item[1] for item in interactions)
                percentage = int((count / max_count) * 100) if max_count > 0 else 0
            else:
                percentage = 0
            
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
        current_time = datetime.now()
        self.message_timestamps.append(current_time)
        
        # Update session performance
        if agent_data:
            self._process_agent_data(agent_data)
        
        # Update tool metrics
        if tool_data:
            self._process_tool_data(tool_data)
        
        # Update conversation metrics
        if conversation_data:
            self._process_conversation_data(conversation_data)
        
        # Calculate and update all metrics
        self._calculate_session_metrics()
        self._calculate_popular_interactions()
        self._calculate_emotion_trends()
        self._calculate_system_metrics()
    
    def _process_agent_data(self, agent_data):
        """Process agent status data"""
        # Track conversation count
        if agent_data.get('conversation_count'):
            self.daily_conversation_count = agent_data['conversation_count']
        
        # Track connection status changes
        connection_status = agent_data.get('connection_status', False)
        if connection_status != self.current_connection_status:
            self.connection_events.append({
                'timestamp': datetime.now(),
                'connected': connection_status
            })
            self.current_connection_status = connection_status
        
        # Track emotions
        if 'emotion' in agent_data:
            self.emotion_history.append({
                'timestamp': datetime.now(),
                'emotion': agent_data['emotion']
            })
            # Keep only recent emotions
            if len(self.emotion_history) > 100:
                self.emotion_history = self.emotion_history[-100:]
    
    def _process_tool_data(self, tool_data):
        """Process tool usage data and track success/failure rates"""
        if 'most_used_tools' in tool_data:
            for tool_name, stats in tool_data['most_used_tools']:
                self.tool_usage_counts[tool_name] = stats.get('total_calls', 0)
                
                # Track response times
                if 'avg_response_time' in stats:
                    self.tool_response_times[tool_name].append(stats['avg_response_time'])
                    # Keep only recent response times
                    if len(self.tool_response_times[tool_name]) > 20:
                        self.tool_response_times[tool_name] = self.tool_response_times[tool_name][-20:]
                
                # Track tool success/failure events for success rate calculation
                successful_calls = stats.get('successful_calls', 0)
                failed_calls = stats.get('failed_calls', 0)
                
                if successful_calls > 0 or failed_calls > 0:
                    self.tool_success_events.append({
                        'timestamp': datetime.now(),
                        'tool_name': tool_name,
                        'successful': successful_calls,
                        'failed': failed_calls,
                        'success_rate': (successful_calls / (successful_calls + failed_calls)) * 100 if (successful_calls + failed_calls) > 0 else 0
                    })
                    
                    # Keep only recent events for analysis
                    if len(self.tool_success_events) > 100:
                        self.tool_success_events = self.tool_success_events[-100:]

    def _process_conversation_data(self, conversation_data):
        """Process conversation analytics data and track conversation outcomes"""
        if conversation_data.get('conversation_active'):
            if not self.current_session_start:
                self.current_session_start = datetime.now()
        else:
            if self.current_session_start:
                # Session ended - determine outcome type
                duration = datetime.now() - self.current_session_start
                
                # Analyze conversation outcome using enhanced timeout information
                outcome_type = 'completed'  # Default assumption
                timeout_category = 'none'   # Track which timeout caused ending
                
                # Check for timeout indicators from conversation widget
                if conversation_data.get('max_timeout_reached', False):
                    outcome_type = 'max_timeout'
                    timeout_category = 'max_conversation'
                elif conversation_data.get('timeout_reached', False):
                    outcome_type = 'user_timeout'
                    timeout_category = 'user_response'
                elif conversation_data.get('user_disconnected', False):
                    outcome_type = 'user_disconnect'
                elif conversation_data.get('natural_ending', False):
                    outcome_type = 'natural'
                elif duration.total_seconds() < 30:  # Very short conversations might be failures
                    outcome_type = 'failed'
                
                # Check warning states for analytics
                approaching_max_timeout = conversation_data.get('approaching_max_timeout', False)
                approaching_user_timeout = conversation_data.get('approaching_user_timeout', False)
                
                session_record = {
                    'start': self.current_session_start,
                    'duration': duration,
                    'outcome': outcome_type,
                    'timeout_category': timeout_category,
                    'turn_count': conversation_data.get('turn_count', 0),
                    'successful': outcome_type in ['completed', 'natural'],
                    'user_response_timeout': conversation_data.get('user_response_timeout', 15.0),
                    'max_conversation_time': conversation_data.get('max_conversation_time', 180.0),
                    'config_source': conversation_data.get('config_source', 'unknown'),
                    'approaching_max_timeout': approaching_max_timeout,
                    'approaching_user_timeout': approaching_user_timeout
                }
                
                self.conversation_sessions.append(session_record)
                self.conversation_outcomes.append({
                    'timestamp': datetime.now(),
                    'outcome': outcome_type,
                    'timeout_category': timeout_category,
                    'duration': duration,
                    'successful': outcome_type in ['completed', 'natural'],
                    'config_source': conversation_data.get('config_source', 'unknown')
                })
                
                self.current_session_start = None
                
                # Keep only recent sessions and outcomes
                if len(self.conversation_sessions) > 50:
                    self.conversation_sessions = self.conversation_sessions[-50:]
                if len(self.conversation_outcomes) > 100:
                    self.conversation_outcomes = self.conversation_outcomes[-100:]

    def _calculate_session_metrics(self):
        """Calculate and update session performance metrics"""
        # Conversations today
        self.conversations_label.setText(str(self.daily_conversation_count))
        
        # Average duration
        if self.conversation_sessions:
            total_duration = sum((session['duration'] for session in self.conversation_sessions), timedelta())
            avg_duration = total_duration / len(self.conversation_sessions)
            duration_str = str(avg_duration).split('.')[0]  # Remove microseconds
            self.avg_duration_label.setText(duration_str)
        else:
            self.avg_duration_label.setText("--")
        
        # Success rate - now based on actual system effectiveness
        self._calculate_success_rate()

    def _calculate_success_rate(self):
        """
        Calculate comprehensive success rate based on tool effectiveness and conversation outcomes.
        
        Success Rate Methodology:
        - Tool Success Rate (40% weight): Percentage of tool calls that complete successfully
        - Conversation Success Rate (60% weight): Percentage of conversations that end naturally/successfully
        
        This provides a meaningful measure of system effectiveness rather than just connectivity.
        
        Success Criteria:
        - Tool Success: Tool calls with status "completed" vs "failed"
        - Conversation Success: Natural endings, user satisfaction vs timeouts, disconnects
        
        Fallback: If insufficient data, shows connection status as basic health indicator.
        """
        
        tool_success_rate = None
        conversation_success_rate = None
        
        # Calculate tool success rate
        if self.tool_success_events:
            recent_tool_events = [event for event in self.tool_success_events 
                                if (datetime.now() - event['timestamp']).total_seconds() < 3600]  # Last hour
            
            if recent_tool_events:
                total_successful = sum(event['successful'] for event in recent_tool_events)
                total_failed = sum(event['failed'] for event in recent_tool_events)
                total_calls = total_successful + total_failed
                
                if total_calls > 0:
                    tool_success_rate = (total_successful / total_calls) * 100
        
        # Calculate conversation success rate
        if self.conversation_outcomes:
            recent_outcomes = [outcome for outcome in self.conversation_outcomes 
                             if (datetime.now() - outcome['timestamp']).total_seconds() < 3600]  # Last hour
            
            if recent_outcomes:
                successful_conversations = sum(1 for outcome in recent_outcomes if outcome['successful'])
                total_conversations = len(recent_outcomes)
                
                if total_conversations > 0:
                    conversation_success_rate = (successful_conversations / total_conversations) * 100
        
        # Combine success rates with weighting
        if tool_success_rate is not None and conversation_success_rate is not None:
            # Both metrics available - use weighted combination
            success_rate = (tool_success_rate * 0.4) + (conversation_success_rate * 0.6)
            self.success_rate_label.setText(f"{success_rate:.0f}%")
            
        elif tool_success_rate is not None:
            # Only tool data available
            success_rate = tool_success_rate
            self.success_rate_label.setText(f"{success_rate:.0f}% (tools)")
            
        elif conversation_success_rate is not None:
            # Only conversation data available
            success_rate = conversation_success_rate
            self.success_rate_label.setText(f"{success_rate:.0f}% (conv)")
            
        else:
            # Fallback to connection status when no meaningful data available
            if self.connection_events:
                connected_time = sum(1 for event in self.connection_events if event['connected'])
                total_events = len(self.connection_events)
                success_rate = (connected_time / total_events) * 100 if total_events > 0 else 0
                self.success_rate_label.setText(f"{success_rate:.0f}% (conn)")
            else:
                success_rate = 100.0 if self.current_connection_status else 0.0
                self.success_rate_label.setText(f"{success_rate:.0f}% (conn)")
        
        # Update progress bar and color coding
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

    def get_success_rate_debug_info(self):
        """
        Get detailed information about success rate calculation for debugging/logging.
        
        Returns:
            dict: Debug information including component metrics and data sources
        """
        debug_info = {
            'tool_events_count': len(self.tool_success_events),
            'conversation_outcomes_count': len(self.conversation_outcomes),
            'connection_events_count': len(self.connection_events),
            'tool_success_rate': None,
            'conversation_success_rate': None,
            'combined_success_rate': None,
            'data_source': 'none'
        }
        
        # Calculate tool success rate for debug
        if self.tool_success_events:
            recent_tool_events = [event for event in self.tool_success_events 
                                if (datetime.now() - event['timestamp']).total_seconds() < 3600]
            
            if recent_tool_events:
                total_successful = sum(event['successful'] for event in recent_tool_events)
                total_failed = sum(event['failed'] for event in recent_tool_events)
                total_calls = total_successful + total_failed
                
                if total_calls > 0:
                    debug_info['tool_success_rate'] = (total_successful / total_calls) * 100
                    debug_info['tool_successful_calls'] = total_successful
                    debug_info['tool_failed_calls'] = total_failed
        
        # Calculate conversation success rate for debug
        if self.conversation_outcomes:
            recent_outcomes = [outcome for outcome in self.conversation_outcomes 
                             if (datetime.now() - outcome['timestamp']).total_seconds() < 3600]
            
            if recent_outcomes:
                successful_conversations = sum(1 for outcome in recent_outcomes if outcome['successful'])
                total_conversations = len(recent_outcomes)
                
                if total_conversations > 0:
                    debug_info['conversation_success_rate'] = (successful_conversations / total_conversations) * 100
                    debug_info['successful_conversations'] = successful_conversations
                    debug_info['total_conversations'] = total_conversations
        
        # Determine final calculation
        if debug_info['tool_success_rate'] is not None and debug_info['conversation_success_rate'] is not None:
            debug_info['combined_success_rate'] = (debug_info['tool_success_rate'] * 0.4) + (debug_info['conversation_success_rate'] * 0.6)
            debug_info['data_source'] = 'combined'
        elif debug_info['tool_success_rate'] is not None:
            debug_info['combined_success_rate'] = debug_info['tool_success_rate']
            debug_info['data_source'] = 'tools_only'
        elif debug_info['conversation_success_rate'] is not None:
            debug_info['combined_success_rate'] = debug_info['conversation_success_rate']
            debug_info['data_source'] = 'conversations_only'
        else:
            debug_info['data_source'] = 'connection_fallback'
            
        return debug_info
    
    def _calculate_popular_interactions(self):
        """Calculate and update popular interactions"""
        if self.tool_usage_counts:
            # Sort tools by usage count
            sorted_tools = sorted(self.tool_usage_counts.items(), key=lambda x: x[1], reverse=True)
            
            # Convert to friendly names and counts
            popular_tools = []
            for tool_name, count in sorted_tools:
                friendly_name = self._get_friendly_tool_name(tool_name)
                popular_tools.append((friendly_name, count))
            
            self._update_popular_interactions(popular_tools)
        else:
            self._update_popular_interactions([])
    
    def _calculate_emotion_trends(self):
        """Calculate and update emotion trends"""
        if not self.emotion_history:
            self.peak_happiness_label.setText("Peak happiness: --")
            self.most_curious_label.setText("Most curious: --")
            self.dominant_emotion_label.setText("Dominant emotion: --")
            return
        
        # Count emotions
        emotion_counts = defaultdict(int)
        for entry in self.emotion_history:
            emotion_counts[entry['emotion']] += 1
        
        # Find dominant emotion
        if emotion_counts:
            dominant_emotion = max(emotion_counts.items(), key=lambda x: x[1])
            total_emotions = sum(emotion_counts.values())
            percentage = (dominant_emotion[1] / total_emotions) * 100
            self.dominant_emotion_label.setText(f"Dominant emotion: {dominant_emotion[0]} ({percentage:.0f}%)")
        
        # Analyze by time (simplified - could be more sophisticated)
        recent_emotions = [entry for entry in self.emotion_history if (datetime.now() - entry['timestamp']).total_seconds() < 3600]
        
        if recent_emotions:
            friendly_emotions = [e for e in recent_emotions if e['emotion'] in ['friendly', 'happy', 'excited']]
            curious_emotions = [e for e in recent_emotions if e['emotion'] == 'curious']
            
            if friendly_emotions:
                self.peak_happiness_label.setText("Peak happiness: Recent activity")
            else:
                self.peak_happiness_label.setText("Peak happiness: --")
            
            if curious_emotions:
                self.most_curious_label.setText("Most curious: Recent activity")
            else:
                self.most_curious_label.setText("Most curious: --")
        else:
            self.peak_happiness_label.setText("Peak happiness: --")
            self.most_curious_label.setText("Most curious: --")
        
        # Add timeout analytics to emotion trends section
        self._update_timeout_analytics()
    
    def _update_timeout_analytics(self):
        """Update timeout analytics in the existing emotion trend labels"""
        # Calculate timeout statistics from recent outcomes
        recent_outcomes = [outcome for outcome in self.conversation_outcomes 
                          if (datetime.now() - outcome['timestamp']).total_seconds() < 3600]
        
        if recent_outcomes:
            # Count different timeout types
            max_timeout_count = sum(1 for o in recent_outcomes if o.get('timeout_category') == 'max_conversation')
            user_timeout_count = sum(1 for o in recent_outcomes if o.get('timeout_category') == 'user_response')
            natural_count = sum(1 for o in recent_outcomes if o.get('outcome') == 'natural')
            total_conversations = len(recent_outcomes)
            
            # Update peak happiness label with timeout completion info
            if total_conversations > 0:
                natural_percentage = (natural_count / total_conversations) * 100
                self.peak_happiness_label.setText(f"Natural endings: {natural_count}/{total_conversations} ({natural_percentage:.0f}%)")
            else:
                self.peak_happiness_label.setText("Natural endings: --")
            
            # Update curious label with timeout breakdown
            if max_timeout_count > 0 or user_timeout_count > 0:
                self.most_curious_label.setText(f"Timeouts: {max_timeout_count} max, {user_timeout_count} user")
            else:
                self.most_curious_label.setText("Timeouts: None recently")
        else:
            self.peak_happiness_label.setText("Natural endings: --")
            self.most_curious_label.setText("Timeouts: --")
    
    def _calculate_system_metrics(self):
        """Calculate and update system metrics from real data"""
        # Message rate calculation
        if len(self.message_timestamps) >= 2:
            recent_messages = [ts for ts in self.message_timestamps if (datetime.now() - ts).total_seconds() < 60]
            if len(recent_messages) >= 2:
                time_span = (recent_messages[-1] - recent_messages[0]).total_seconds()
                message_rate = len(recent_messages) / time_span if time_span > 0 else 0
                self.message_rate_label.setText(f"{message_rate:.1f}/s")
            else:
                self.message_rate_label.setText("--")
        else:
            self.message_rate_label.setText("--")
        
        # Connection health
        if self.current_connection_status:
            self.connection_health_label.setText("Connected")
            self.connection_health_label.setStyleSheet("color: #28a745;")
        else:
            self.connection_health_label.setText("Disconnected")
            self.connection_health_label.setStyleSheet("color: #dc3545;")
        
        # Average tool response time
        if self.tool_response_times:
            all_times = []
            for tool_times in self.tool_response_times.values():
                all_times.extend(tool_times)
            
            if all_times:
                avg_time = sum(all_times) / len(all_times)
                self.tool_response_label.setText(f"{avg_time:.1f}s")
            else:
                self.tool_response_label.setText("--")
        else:
            self.tool_response_label.setText("--")
    
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