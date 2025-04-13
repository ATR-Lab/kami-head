#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                            QVBoxLayout, QHBoxLayout, QLabel,
                            QPushButton, QCheckBox)
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QBrush
from PyQt5.QtCore import Qt, QPoint, QRect

class MotorControlWidget(QWidget):
    def __init__(self, motor_name="Motor", parent=None):
        super().__init__(parent)
        self.motor_name = motor_name
        self.angle = 0  # Angle in degrees
        self.radius = 100
        self.circle_center = QPoint(self.radius + 20, self.radius + 20)
        self.dragging = False
        self.torque_enabled = False
        self.setMinimumSize(2 * (self.radius + 40), 2 * (self.radius + 60))
        
        # Create layout for the control buttons
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Widget for drawing the circle
        self.circle_widget = QWidget()
        self.circle_widget.setMinimumSize(2 * (self.radius + 40), 2 * (self.radius + 40))
        self.circle_widget.paintEvent = self.paintCircleWidget
        self.circle_widget.mousePressEvent = self.circleMousePressEvent
        self.circle_widget.mouseMoveEvent = self.circleMouseMoveEvent
        self.circle_widget.mouseReleaseEvent = self.circleMouseReleaseEvent
        self.main_layout.addWidget(self.circle_widget)
        
        # Controls
        controls_layout = QHBoxLayout()
        
        # Torque toggle
        self.torque_checkbox = QCheckBox("Enable Torque")
        self.torque_checkbox.setChecked(self.torque_enabled)
        self.torque_checkbox.stateChanged.connect(self.toggleTorque)
        controls_layout.addWidget(self.torque_checkbox)
        
        # Reset position button
        self.reset_button = QPushButton("Reset Position")
        self.reset_button.clicked.connect(self.resetPosition)
        controls_layout.addWidget(self.reset_button)
        
        self.main_layout.addLayout(controls_layout)

    def paintCircleWidget(self, event):
        painter = QPainter(self.circle_widget)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw circle
        painter.setPen(QPen(Qt.black, 2))
        if self.torque_enabled:
            painter.setBrush(QBrush(QColor(230, 230, 255)))  # Light blue when torque enabled
        else:
            painter.setBrush(QBrush(QColor(240, 240, 240)))  # Light gray when disabled
        circle_rect = QRect(20, 20, 2 * self.radius, 2 * self.radius)
        painter.drawEllipse(circle_rect)
        
        # Draw motor name
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        name_rect = QRect(20, 2 * self.radius + 30, 2 * self.radius, 30)
        painter.drawText(name_rect, Qt.AlignCenter, self.motor_name)
        
        # Draw angle text
        angle_text = f"{self.angle:.1f}°"
        painter.drawText(name_rect.translated(0, 25), Qt.AlignCenter, angle_text)
        
        # Draw line from center to edge (like a clock hand)
        if self.torque_enabled:
            painter.setPen(QPen(Qt.red, 3))
        else:
            painter.setPen(QPen(Qt.gray, 3))
        line_end_x = self.circle_center.x() + self.radius * math.cos(math.radians(self.angle - 90))
        line_end_y = self.circle_center.y() + self.radius * math.sin(math.radians(self.angle - 90))
        painter.drawLine(self.circle_center, QPoint(int(line_end_x), int(line_end_y)))
        
        # Draw a small circle at the center
        painter.setPen(QPen(Qt.black, 1))
        painter.setBrush(QBrush(Qt.black))
        painter.drawEllipse(self.circle_center, 5, 5)

    def circleMousePressEvent(self, event):
        # Check if the click is within the circle
        dx = event.x() - self.circle_center.x()
        dy = event.y() - self.circle_center.y()
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist <= self.radius and self.torque_enabled:
            self.dragging = True
            self.updateAngle(event.x(), event.y())

    def circleMouseMoveEvent(self, event):
        if self.dragging and self.torque_enabled:
            self.updateAngle(event.x(), event.y())

    def circleMouseReleaseEvent(self, event):
        self.dragging = False

    def updateAngle(self, x, y):
        # Calculate angle from center to mouse point
        dx = x - self.circle_center.x()
        dy = y - self.circle_center.y()
        angle = math.degrees(math.atan2(dy, dx)) + 90
        
        # Normalize to 0-360 range
        self.angle = angle % 360
        
        # Update display
        self.circle_widget.update()
        
        # In a real application, you would send the angle to the Dynamixel motor
        print(f"{self.motor_name} angle set to: {self.angle:.1f}°")
    
    def toggleTorque(self, state):
        self.torque_enabled = bool(state)
        self.circle_widget.update()
        print(f"{self.motor_name} torque {'enabled' if self.torque_enabled else 'disabled'}")
    
    def resetPosition(self):
        self.angle = 0
        self.circle_widget.update()
        print(f"{self.motor_name} position reset to 0°")


class DynamixelControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('Dynamixel Motor Control')
        self.setGeometry(100, 100, 600, 500)  # Made the window taller
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Title label
        title_label = QLabel('Dynamixel Motor Control')
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont('Arial', 16, QFont.Bold))
        main_layout.addWidget(title_label)
        
        # Motor controls layout
        motors_layout = QHBoxLayout()
        
        # Pan motor control
        self.pan_motor = MotorControlWidget("Pan Motor")
        motors_layout.addWidget(self.pan_motor)
        
        # Tilt motor control
        self.tilt_motor = MotorControlWidget("Tilt Motor")
        motors_layout.addWidget(self.tilt_motor)
        
        main_layout.addLayout(motors_layout)


def main(args=None):
    rclpy.init(args=args)
    node = Node('dynamixel_ui')
    
    app = QApplication(sys.argv)
    window = DynamixelControlUI()
    window.show()
    
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
