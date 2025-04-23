#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                           QHBoxLayout, QComboBox, QRadioButton, QButtonGroup,
                           QSlider, QLabel, QCheckBox, QPushButton, QFrame)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen
from coffee_expressions_msgs.msg import AffectiveState
from geometry_msgs.msg import Point

class GazePreviewWidget(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.x = 0.0
        self.y = 0.0
        
    def setPosition(self, x: float, y: float):
        self.x = x
        self.y = y
        self.update()
        
    def mousePressEvent(self, event):
        size = min(self.width(), self.height())
        x = (event.x() - self.width()/2) / (size/2)
        y = -(event.y() - self.height()/2) / (size/2)  # Invert Y for Qt coordinates
        self.x = max(-1.0, min(1.0, x))
        self.y = max(-1.0, min(1.0, y))
        self.update()
        if self.parent():
            self.parent().updateSliders(self.x, self.y)
            
    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw coordinate system
        painter.translate(self.width()/2, self.height()/2)
        size = min(self.width(), self.height())
        
        # Draw axes
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.drawLine(-size/2, 0, size/2, 0)
        painter.drawLine(0, -size/2, 0, size/2)
        
        # Draw gaze target point
        painter.setPen(QPen(Qt.red, 3))
        x_pos = self.x * size/2
        y_pos = -self.y * size/2  # Invert Y for Qt coordinates
        painter.drawPoint(x_pos, y_pos)
        
class ExpressionsTestUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize ROS node
        rclpy.init()
        self.node = Node('expressions_test_ui')
        self.publisher = self.node.create_publisher(
            AffectiveState,
            'affective_state',
            10
        )
        
        # Setup UI
        self.setWindowTitle('Expressions Test UI')
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        # Expression dropdown
        self.expression_layout = QHBoxLayout()
        self.expression_label = QLabel('Expression:')
        self.expression_combo = QComboBox()
        self.expression_combo.addItems(['Happy', 'Curious', 'Neutral', 'Sad', 'Surprised'])
        self.expression_layout.addWidget(self.expression_label)
        self.expression_layout.addWidget(self.expression_combo)
        self.layout.addLayout(self.expression_layout)
        
        # Trigger source radio buttons
        self.trigger_group = QButtonGroup()
        self.trigger_layout = QHBoxLayout()
        self.trigger_label = QLabel('Trigger Source:')
        self.trigger_layout.addWidget(self.trigger_label)
        
        for source in ['vision', 'audio', 'event', 'mock']:
            radio = QRadioButton(source)
            self.trigger_layout.addWidget(radio)
            self.trigger_group.addButton(radio)
            if source == 'mock':
                radio.setChecked(True)
                
        self.layout.addLayout(self.trigger_layout)
        
        # Gaze preview
        self.preview = GazePreviewWidget(self)
        self.layout.addWidget(self.preview)
        
        # X and Y sliders
        for coord in ['X', 'Y']:
            slider_layout = QHBoxLayout()
            label = QLabel(f'{coord}:')
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            value_label = QLabel('0.0')
            
            slider.valueChanged.connect(
                lambda v, c=coord, l=value_label: self.onSliderChanged(v, c, l))
            
            setattr(self, f'{coord.lower()}_slider', slider)
            setattr(self, f'{coord.lower()}_label', value_label)
            
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(value_label)
            self.layout.addLayout(slider_layout)
            
        # Is Idle checkbox
        self.idle_check = QCheckBox('Is Idle')
        self.layout.addWidget(self.idle_check)
        
        # Real-time publishing checkbox
        self.realtime_check = QCheckBox('Real-time Publishing')
        self.realtime_check.stateChanged.connect(self.onRealtimeChanged)
        self.layout.addWidget(self.realtime_check)
        
        # Publish button
        self.publish_button = QPushButton('Publish Message')
        self.publish_button.clicked.connect(self.publishMessage)
        self.layout.addWidget(self.publish_button)
        
        # Status bar
        self.status_label = QLabel('Status: Ready')
        self.layout.addWidget(self.status_label)
        
        # Timer for ROS spin
        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.spin_timer.start(100)  # 10Hz
        
        # Timer for real-time publishing
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publishMessage)
        
    def updateSliders(self, x: float, y: float):
        self.x_slider.setValue(int(x * 100))
        self.y_slider.setValue(int(y * 100))
        
    def onSliderChanged(self, value: int, coord: str, label: QLabel):
        float_value = value / 100.0
        label.setText(f'{float_value:.2f}')
        
        if coord == 'X':
            self.preview.setPosition(float_value, self.y_slider.value() / 100.0)
        else:
            self.preview.setPosition(self.x_slider.value() / 100.0, float_value)
            
    def onRealtimeChanged(self, state):
        if state == Qt.Checked:
            self.publish_timer.start(100)  # 10Hz
        else:
            self.publish_timer.stop()
            
    def publishMessage(self):
        msg = AffectiveState()
        msg.expression = self.expression_combo.currentText()
        msg.trigger_source = self.trigger_group.checkedButton().text()
        
        msg.gaze_target = Point()
        msg.gaze_target.x = self.x_slider.value() / 100.0
        msg.gaze_target.y = self.y_slider.value() / 100.0
        msg.gaze_target.z = 0.0
        
        msg.is_idle = self.idle_check.isChecked()
        
        self.publisher.publish(msg)
        self.status_label.setText(f'Status: Published at {self.node.get_clock().now().to_msg().sec}')
        
    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = ExpressionsTestUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
