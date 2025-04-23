#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                           QHBoxLayout, QLabel, QPushButton, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import String
from geometry_msgs.msg import Point
from coffee_expressions_msgs.msg import AffectiveState
import time


class StateMonitorWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle('Coffee Expressions State Monitor')
        self.setGeometry(100, 100, 800, 600)

        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Create input monitoring section
        input_group = QGroupBox("Input Monitoring")
        input_layout = QVBoxLayout()
        
        # Vision emotion
        self.vision_label = QLabel("Vision Emotion: -")
        input_layout.addWidget(self.vision_label)
        
        # Voice intent
        self.voice_label = QLabel("Voice Intent: -")
        input_layout.addWidget(self.voice_label)
        
        # Face position
        self.face_pos_label = QLabel("Face Position: x=0.0, y=0.0, z=0.0")
        input_layout.addWidget(self.face_pos_label)
        
        # System event
        self.event_label = QLabel("System Event: -")
        input_layout.addWidget(self.event_label)
        
        input_group.setLayout(input_layout)
        main_layout.addWidget(input_group)

        # Create state management section
        state_group = QGroupBox("State Management")
        state_layout = QVBoxLayout()
        
        # Current affective state
        self.affective_label = QLabel("Current Expression: -")
        state_layout.addWidget(self.affective_label)
        
        # Override status
        self.override_label = QLabel("Override: None")
        state_layout.addWidget(self.override_label)
        
        # Diagnostics
        self.diagnostics_label = QLabel("Diagnostics: -")
        state_layout.addWidget(self.diagnostics_label)
        
        state_group.setLayout(state_layout)
        main_layout.addWidget(state_group)

        # Create testing controls section
        control_group = QGroupBox("Testing Controls")
        control_layout = QVBoxLayout()
        
        # System event buttons
        event_layout = QHBoxLayout()
        for event in ["PaymentSuccess", "PaymentFailed", "CoffeeReady", "CoffeeError"]:
            btn = QPushButton(event)
            btn.clicked.connect(lambda checked, e=event: self.trigger_event(e))
            event_layout.addWidget(btn)
        control_layout.addLayout(event_layout)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)

        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_labels)
        self.update_timer.start(100)  # Update every 100ms

    def update_labels(self):
        # Update all labels with latest data from the node
        self.vision_label.setText(f"Vision Emotion: {self.node.last_vision_emotion}")
        self.voice_label.setText(f"Voice Intent: {self.node.last_voice_intent}")
        self.face_pos_label.setText(
            f"Face Position: x={self.node.last_face_pos.x:.2f}, "
            f"y={self.node.last_face_pos.y:.2f}, z={self.node.last_face_pos.z:.2f}")
        self.event_label.setText(f"System Event: {self.node.last_system_event}")
        self.affective_label.setText(
            f"Current Expression: {self.node.current_expression}")
        
        if self.node.override_active:
            remaining = max(0, self.node.override_end_time - time.time())
            self.override_label.setText(
                f"Override: {self.node.override_expression} "
                f"({remaining:.1f}s remaining)")
        else:
            self.override_label.setText("Override: None")
        
        self.diagnostics_label.setText(f"Diagnostics: {self.node.last_diagnostic}")

    def trigger_event(self, event):
        self.node.publish_event(event)


class StateUINode(Node):
    def __init__(self):
        super().__init__('state_ui_node')
        
        # Initialize state variables
        self.last_vision_emotion = "-"
        self.last_voice_intent = "-"
        self.last_face_pos = Point()
        self.last_system_event = "-"
        self.current_expression = "-"
        self.last_diagnostic = "-"
        self.override_active = False
        self.override_expression = None
        self.override_end_time = 0
        
        # Create subscribers
        self.create_subscription(
            String, '/vision/emotion', self.vision_callback, 10)
        self.create_subscription(
            String, '/voice/intent', self.voice_callback, 10)
        self.create_subscription(
            Point, '/vision/face_position', self.face_callback, 10)
        self.create_subscription(
            String, '/system/event', self.event_callback, 10)
        self.create_subscription(
            AffectiveState, '/robot/affective_state', self.state_callback, 10)
        self.create_subscription(
            String, '/robot/state_manager/diagnostics', self.diagnostics_callback, 10)
            
        # Create publisher for testing events
        self.event_pub = self.create_publisher(String, '/system/event', 10)

    def vision_callback(self, msg):
        self.last_vision_emotion = msg.data

    def voice_callback(self, msg):
        self.last_voice_intent = msg.data

    def face_callback(self, msg):
        self.last_face_pos = msg

    def event_callback(self, msg):
        self.last_system_event = msg.data

    def state_callback(self, msg):
        self.current_expression = msg.expression
        self.override_active = msg.is_override
        if self.override_active:
            self.override_expression = msg.expression
            self.override_end_time = time.time() + msg.override_duration

    def diagnostics_callback(self, msg):
        self.last_diagnostic = msg.data

    def publish_event(self, event):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateUINode()
    
    app = QApplication(sys.argv)
    window = StateMonitorWindow(node)
    window.show()
    
    # Setup timer for ROS spinning
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin_timer.start(10)  # Spin every 10ms
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
