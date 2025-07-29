#!/usr/bin/env python3
"""
Emotion Display Widget - Shows agent emotional state and transitions

Displays the current emotion, previous emotion transitions, emotion timeline,
and provides visual feedback for emotional state changes.
"""

from datetime import datetime
from collections import deque

from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QProgressBar
)
from python_qt_binding.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve
from python_qt_binding.QtGui import QFont, QPainter, QColor, QPen


class EmotionDisplayWidget(QWidget):
    """Widget for displaying agent emotional state and transitions"""
    
    # Emotion to emoji mapping for visual display
    EMOTION_EMOJIS = {
        'friendly': '😊',
        'excited': '🤩', 
        'curious': '🤔',
        'sleepy': '😴',
        'waiting': '😌',
        'excuse': '😅',
        'helpful': '🤝',
        'empathetic': '🥺',
        'confused': '😕',
        'proud': '😊',
        'playful': '😄',
        'focused': '🧐',
        'surprised': '😮',
        'enthusiastic': '🎉',
        'warm': '🤗',
        'professional': '👔',
        'cheerful': '😁'
    }
    
    # Emotion colors for background/border effects
    EMOTION_COLORS = {
        'friendly': '#28a745',      # Green
        'excited': '#ffc107',       # Yellow
        'curious': '#17a2b8',       # Cyan
        'sleepy': '#6f42c1',        # Purple
        'waiting': '#20c997',       # Teal
        'excuse': '#fd7e14',        # Orange
        'helpful': '#28a745',       # Green
        'empathetic': '#e83e8c',    # Pink
        'confused': '#6c757d',      # Gray
        'proud': '#007bff',         # Blue
        'playful': '#ffc107',       # Yellow
        'focused': '#343a40',       # Dark
        'surprised': '#dc3545',     # Red
        'enthusiastic': '#fd7e14',  # Orange
        'warm': '#fd7e14',          # Orange
        'professional': '#343a40',  # Dark
        'cheerful': '#ffc107'       # Yellow
    }
    
    def __init__(self):
        super().__init__()
        self.setFixedSize(350, 250)
        
        # Emotion tracking
        self.current_emotion = 'waiting'
        self.previous_emotion = ''
        self.emotion_history = deque(maxlen=10)  # Last 10 emotions for timeline
        self.transition_time = None
        
        self._setup_ui()
        
        # Animation timer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._update_animations)
        self.animation_timer.start(100)  # 10 FPS for smooth animations
    
    def _setup_ui(self):
        """Set up the emotion display UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("🎭 EMOTION CENTER")
        title.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setBold(True)
        font.setPointSize(12)
        title.setFont(font)
        layout.addWidget(title)
        
        # Main emotion frame
        emotion_frame = QFrame()
        emotion_frame.setFrameStyle(QFrame.Box)
        layout.addWidget(emotion_frame)
        
        emotion_layout = QVBoxLayout()
        emotion_frame.setLayout(emotion_layout)
        
        # Current emotion display
        current_layout = QHBoxLayout()
        current_layout.addWidget(QLabel("Current:"))
        
        self.current_emotion_label = QLabel("😌 waiting")
        self.current_emotion_label.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.current_emotion_label.setFont(font)
        self.current_emotion_label.setStyleSheet("""
            padding: 8px;
            border: 2px solid #20c997;
            border-radius: 8px;
            background-color: rgba(32, 201, 151, 0.1);
        """)
        current_layout.addWidget(self.current_emotion_label)
        emotion_layout.addLayout(current_layout)
        
        # Transition display
        transition_layout = QHBoxLayout()
        transition_layout.addWidget(QLabel("Previous:"))
        self.transition_label = QLabel("none → waiting")
        transition_layout.addWidget(self.transition_label)
        transition_layout.addStretch()
        emotion_layout.addLayout(transition_layout)
        
        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        emotion_layout.addWidget(separator)
        
        # Emotion journey title
        journey_title = QLabel("Emotion Journey:")
        font = QFont()
        font.setBold(True)
        journey_title.setFont(font)
        emotion_layout.addWidget(journey_title)
        
        # Emotion timeline (visual history)
        self.timeline_frame = QFrame()
        self.timeline_frame.setFixedHeight(40)
        self.timeline_frame.setFrameStyle(QFrame.Box)
        emotion_layout.addWidget(self.timeline_frame)
        
        # Eye animation preview
        animation_layout = QHBoxLayout()
        animation_layout.addWidget(QLabel("Eye Animation:"))
        self.animation_label = QLabel("👀 Waiting patiently...")
        animation_layout.addWidget(self.animation_label)
        emotion_layout.addLayout(animation_layout)
    
    def update_emotion(self, emotion: str, previous_emotion: str = ''):
        """Update the current emotion display"""
        # Store previous state for transition
        if emotion != self.current_emotion:
            self.previous_emotion = self.current_emotion
            self.current_emotion = emotion
            self.transition_time = datetime.now()
            
            # Add to history
            self.emotion_history.append((datetime.now(), emotion))
        
        # Update displays
        self._update_current_emotion_display()
        self._update_transition_display()
        self._update_animation_preview()
        self.update()  # Trigger repaint for timeline
    
    def _update_current_emotion_display(self):
        """Update the main emotion display"""
        emoji = self.EMOTION_EMOJIS.get(self.current_emotion, '🤖')
        text = f"{emoji} {self.current_emotion}"
        self.current_emotion_label.setText(text)
        
        # Update color scheme
        color = self.EMOTION_COLORS.get(self.current_emotion, '#6c757d')
        self.current_emotion_label.setStyleSheet(f"""
            padding: 8px;
            border: 2px solid {color};
            border-radius: 8px;
            background-color: {color}20;
            color: {color};
        """)
    
    def _update_transition_display(self):
        """Update the emotion transition display"""
        if self.previous_emotion:
            prev_emoji = self.EMOTION_EMOJIS.get(self.previous_emotion, '🤖')
            curr_emoji = self.EMOTION_EMOJIS.get(self.current_emotion, '🤖')
            self.transition_label.setText(f"{prev_emoji} {self.previous_emotion} → {curr_emoji} {self.current_emotion}")
        else:
            emoji = self.EMOTION_EMOJIS.get(self.current_emotion, '🤖')
            self.transition_label.setText(f"→ {emoji} {self.current_emotion}")
    
    def _update_animation_preview(self):
        """Update the eye animation preview description"""
        animations = {
            'friendly': '👀 Warm, gentle gaze with soft blinks',
            'excited': '✨ Wide eyes with rapid, enthusiastic blinks',
            'curious': '🔍 Focused gaze with inquisitive movements',
            'sleepy': '😴 Slow, drowsy blinks with droopy eyes',
            'waiting': '⏳ Patient, steady gaze with regular blinks',
            'excuse': '😅 Apologetic glances with nervous blinking',
            'helpful': '🤝 Attentive, supportive eye contact',
            'empathetic': '🥺 Caring, understanding expression',
            'confused': '😕 Puzzled looks with questioning glances',
            'proud': '😊 Confident, satisfied expression',
            'playful': '😄 Mischievous winks and animated blinks',
            'focused': '🧐 Intense concentration with minimal blinking',
            'surprised': '😮 Wide open eyes with rapid blinking',
            'enthusiastic': '🎉 Bright, energetic eye movements',
            'warm': '🤗 Gentle, welcoming gaze',
            'professional': '👔 Composed, attentive expression',
            'cheerful': '😁 Happy, upbeat eye sparkles'
        }
        
        description = animations.get(self.current_emotion, '🤖 Standard neutral expression')
        self.animation_label.setText(description)
    
    def paintEvent(self, event):
        """Custom paint event for emotion timeline"""
        super().paintEvent(event)
        
        # Draw emotion timeline
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get timeline frame geometry
        timeline_rect = self.timeline_frame.geometry()
        
        if len(self.emotion_history) > 1:
            # Draw emotion history as colored rectangles
            rect_width = timeline_rect.width() // len(self.emotion_history)
            
            for i, (timestamp, emotion) in enumerate(self.emotion_history):
                color = QColor(self.EMOTION_COLORS.get(emotion, '#6c757d'))
                painter.setBrush(color)
                painter.setPen(QPen(color.darker(120), 1))
                
                x = timeline_rect.x() + i * rect_width
                y = timeline_rect.y()
                painter.drawRect(x, y, rect_width - 1, timeline_rect.height())
                
                # Draw emoji on top if space allows
                if rect_width > 20:
                    emoji = self.EMOTION_EMOJIS.get(emotion, '🤖')
                    painter.setPen(QPen(Qt.white))
                    painter.drawText(x + 2, y + timeline_rect.height() - 5, emoji)
    
    def update_animations(self):
        """Update any running animations"""
        # This can be used for smooth emotion transitions
        if self.transition_time:
            elapsed = (datetime.now() - self.transition_time).total_seconds()
            if elapsed > 2.0:  # Transition complete after 2 seconds
                self.transition_time = None
    
    def _update_animations(self):
        """Timer callback for animation updates"""
        self.update_animations()
    
    def get_analytics_data(self):
        """Get emotion analytics data"""
        # Calculate emotion distribution
        emotion_counts = {}
        for _, emotion in self.emotion_history:
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1
        
        return {
            'current_emotion': self.current_emotion,
            'emotion_history': list(self.emotion_history),
            'emotion_distribution': emotion_counts,
            'transition_count': len(self.emotion_history)
        } 