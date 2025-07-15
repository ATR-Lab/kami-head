#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, 
                            QVBoxLayout, QHBoxLayout, QLabel,
                            QPushButton, QCheckBox)
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QBrush
from PyQt5.QtCore import Qt, QPoint, QRect, QTimer

# Import Dynamixel SDK interfaces
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

# Default settings from the read_write_node example
DXL_PAN_ID = 1   # Pan motor ID
DXL_TILT_ID = 9  # Tilt motor ID

# Constants for position conversion
# Dynamixel position range is typically 0-4095 for a full 360 degrees
# In the UI we use 0-360 degrees, so we need conversion
MIN_POSITION = 0
MAX_POSITION = 4095
POSITION_RANGE = MAX_POSITION - MIN_POSITION
DEGREES_PER_POSITION = 360.0 / POSITION_RANGE
POSITIONS_PER_DEGREE = POSITION_RANGE / 360.0

# Default angles for motors
DEFAULT_PAN_ANGLE = 180   # Pan motor default position (90 degrees)
DEFAULT_TILT_ANGLE = 180  # Tilt motor default position (180 degrees)

class MotorControlWidget(QWidget):
    def __init__(self, motor_id, motor_name="Motor", default_angle=0, parent=None):
        super().__init__(parent)
        self.motor_id = motor_id
        self.motor_name = motor_name
        self.default_angle = default_angle
        self.angle = default_angle  # Angle in degrees
        self.position = int(default_angle * POSITIONS_PER_DEGREE)  # Position in Dynamixel units
        self.radius = 100
        self.circle_center = QPoint(self.radius + 20, self.radius + 20)
        self.dragging = False
        self.torque_enabled = False
        self.motor_connected = False  # Track motor connection status
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
        self.torque_checkbox.setEnabled(False)  # Disabled until motor connects
        self.torque_checkbox.stateChanged.connect(self.toggleTorque)
        controls_layout.addWidget(self.torque_checkbox)
        
        # Reset position button
        self.reset_button = QPushButton("Reset Position")
        self.reset_button.setEnabled(False)  # Disabled until motor connects
        self.reset_button.clicked.connect(self.resetPosition)
        controls_layout.addWidget(self.reset_button)
        
        self.main_layout.addLayout(controls_layout)
        
        # Conversion helpers
        self.rosnode = None  # Will be set by parent

    def set_ros_node(self, node):
        self.rosnode = node

    def paintCircleWidget(self, event):
        painter = QPainter(self.circle_widget)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw circle with connection status
        if not self.motor_connected:
            painter.setPen(QPen(Qt.red, 3))  # Red border when disconnected
            painter.setBrush(QBrush(QColor(255, 220, 220)))  # Light red when disconnected
        elif self.torque_enabled:
            painter.setPen(QPen(Qt.black, 2))
            painter.setBrush(QBrush(QColor(230, 230, 255)))  # Light blue when torque enabled
        else:
            painter.setPen(QPen(Qt.black, 2))
            painter.setBrush(QBrush(QColor(240, 240, 240)))  # Light gray when disabled
        circle_rect = QRect(20, 20, 2 * self.radius, 2 * self.radius)
        painter.drawEllipse(circle_rect)
        
        # Draw motor name
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        name_rect = QRect(20, 2 * self.radius + 30, 2 * self.radius, 30)
        painter.drawText(name_rect, Qt.AlignCenter, self.motor_name)

        # Draw angle text with connection status
        if self.motor_connected:
            angle_text = f"{self.angle:.1f}°  (Pos: {self.position})"
        else:
            angle_text = f"{self.angle:.1f}° (DISCONNECTED)"
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
        if self.torque_enabled and self.rosnode:
            # Send final position to motor when releasing
            self.send_position_to_motor()

    def updateAngle(self, x, y):
        # Calculate angle from center to mouse point
        dx = x - self.circle_center.x()
        dy = y - self.circle_center.y()
        angle = math.degrees(math.atan2(dy, dx)) + 90
        
        # Normalize to 0-360 range
        self.angle = angle % 360
        
        # Convert angle to Dynamixel position
        self.position = int(self.angle * POSITIONS_PER_DEGREE)
        
        # Update display
        self.circle_widget.update()
        
        # Only send to motor if dragging stopped (to reduce traffic)
        # Full position will be sent when mouse is released

    def send_position_to_motor(self):
        if self.rosnode and self.torque_enabled:
            msg = SetPosition()
            msg.id = self.motor_id
            msg.position = self.position
            self.rosnode.publisher.publish(msg)
            self.rosnode.get_logger().info(f"{self.motor_name} (ID: {self.motor_id}) position set to: {self.position}")
    
    def toggleTorque(self, state):
        self.torque_enabled = bool(state)
        self.circle_widget.update()
        if self.rosnode:
            self.rosnode.get_logger().info(f"{self.motor_name} (ID: {self.motor_id}) torque {'enabled' if self.torque_enabled else 'disabled'}")
            # In a real implementation, you would send torque command to motor here
            # However, the example doesn't have a direct torque toggle topic
            # You would need to add this functionality to the read_write_node
    
    def resetPosition(self):
        self.angle = self.default_angle
        self.position = int(self.default_angle * POSITIONS_PER_DEGREE)
        self.circle_widget.update()
        
        if self.rosnode and self.torque_enabled:
            self.rosnode.get_logger().info(f"{self.motor_name} (ID: {self.motor_id}) position reset to {self.default_angle}°")
            self.send_position_to_motor()
    
    def set_position_from_motor(self, position):
        # Update UI from motor position
        self.position = position
        self.angle = (position * DEGREES_PER_POSITION) % 360
        self.motor_connected = True  # Mark motor as connected when we receive position
        # Enable controls when motor is connected
        self.torque_checkbox.setEnabled(True)
        self.reset_button.setEnabled(True)
        self.circle_widget.update()
    
    def set_motor_disconnected(self):
        """Mark motor as disconnected and disable controls"""
        self.motor_connected = False
        self.torque_checkbox.setEnabled(False)
        self.reset_button.setEnabled(False)
        self.circle_widget.update()


class DynamixelControlUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.service_connected = False  # Track overall service status
        self.initUI()
        
        # Initial motor position read
        self.read_motor_positions()
        
    def initUI(self):
        self.setWindowTitle('Dynamixel Motor Control')
        self.setGeometry(100, 100, 600, 500)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Title label
        title_label = QLabel('Dynamixel Motor Control')
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont('Arial', 16, QFont.Bold))
        main_layout.addWidget(title_label)
        
        # Service status indicator
        self.status_label = QLabel('Service Status: Checking connection...')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont('Arial', 11))
        self.update_service_status(False)  # Initialize as disconnected
        main_layout.addWidget(self.status_label)
        
        # Motor controls layout
        motors_layout = QHBoxLayout()
        
        # Pan motor control (default angle: 90 degrees)
        self.pan_motor = MotorControlWidget(DXL_PAN_ID, "Pan Motor", DEFAULT_PAN_ANGLE)
        self.pan_motor.set_ros_node(self.node)
        motors_layout.addWidget(self.pan_motor)
        
        # Tilt motor control (default angle: 180 degrees)
        self.tilt_motor = MotorControlWidget(DXL_TILT_ID, "Tilt Motor", DEFAULT_TILT_ANGLE)
        self.tilt_motor.set_ros_node(self.node)
        motors_layout.addWidget(self.tilt_motor)
        
        main_layout.addLayout(motors_layout)
        
        # Help instructions
        help_label = QLabel('Instructions: To control motors, run "ros2 run dynamixel_sdk_examples read_write_node" in another terminal')
        help_label.setAlignment(Qt.AlignCenter)
        help_label.setFont(QFont('Arial', 9))
        help_label.setStyleSheet("""
            QLabel {
                color: #666666;
                background-color: #f5f5f5;
                border: 1px solid #cccccc;
                border-radius: 3px;
                padding: 5px;
                margin: 5px;
            }
        """)
        main_layout.addWidget(help_label)
    
    def update_service_status(self, connected):
        """Update the service status indicator"""
        self.service_connected = connected
        if connected:
            self.status_label.setText('✓ Dynamixel Service: CONNECTED - Motors ready for control')
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #d4e8d4;
                    color: #2e7d2e;
                    border: 2px solid #4caf50;
                    border-radius: 5px;
                    padding: 8px;
                    font-weight: bold;
                }
            """)
        else:
            self.status_label.setText('⚠ Dynamixel Service: DISCONNECTED - Start "read_write_node" to control motors')
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #f8d7da;
                    color: #721c24;
                    border: 2px solid #f44336;
                    border-radius: 5px;
                    padding: 8px;
                    font-weight: bold;
                }
            """)
    
    def read_motor_positions(self):
        """Read current motor positions from the motors"""
        self.node.get_logger().info('Reading motor positions...')
        self.get_motor_position(DXL_PAN_ID, self.pan_motor)
        self.get_motor_position(DXL_TILT_ID, self.tilt_motor)
    
    def get_motor_position(self, motor_id, motor_widget):
        """Get position for a specific motor"""
        client = self.node.create_client(GetPosition, 'get_position')
        
        # Check if service is available with a short timeout (non-blocking)
        if not client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warning(f'get_position service not available for motor ID {motor_id}. UI will show default positions.')
            # Update service status to disconnected
            self.update_service_status(False)
            # Mark this motor as disconnected
            motor_widget.set_motor_disconnected()
            # Set a timer to retry later
            self.create_retry_timer(motor_id, motor_widget)
            return
        
        request = GetPosition.Request()
        request.id = motor_id
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.process_position_response(f, motor_widget)
        )
    
    def create_retry_timer(self, motor_id, motor_widget):
        """Create a timer to retry reading motor position later"""
        def retry_callback():
            self.node.get_logger().info(f'Retrying position read for motor ID {motor_id}...')
            # Cancel this timer first
            timer.cancel()
            # Then retry the position read
            self.get_motor_position(motor_id, motor_widget)
        
        # Create a one-shot timer to retry after 5 seconds
        timer = self.node.create_timer(5.0, retry_callback)
    
    def process_position_response(self, future, motor_widget):
        """Process the response from the get_position service"""
        try:
            response = future.result()
            self.node.get_logger().info(f'Received position {response.position} for motor ID {motor_widget.motor_id}')
            motor_widget.set_position_from_motor(response.position)
            # Update service status to connected when we get a successful response
            if not self.service_connected:
                self.update_service_status(True)
        except Exception as e:
            self.node.get_logger().error(f'Service call failed for motor ID {motor_widget.motor_id}: {e}')
            # Update service status to disconnected on failure
            self.update_service_status(False)
            # Mark this motor as disconnected
            motor_widget.set_motor_disconnected()
            # Retry after a delay if the service call failed
            self.create_retry_timer(motor_widget.motor_id, motor_widget)


class DynamixelUINode(Node):
    def __init__(self):
        super().__init__('dynamixel_ui_node')
        
        # Set up QoS profile
        qos = QoSProfile(depth=10)
        
        # Create publisher to send position commands
        self.publisher = self.create_publisher(
            SetPosition,
            'set_position',
            qos
        )
        
        self.get_logger().info('DynamixelUINode is running')
        
        # Start the UI
        app = QApplication(sys.argv)
        self.ui = DynamixelControlUI(self)
        self.ui.show()
        
        # Start a background thread for ROS spinning
        from threading import Thread
        self.ros_thread = Thread(target=self.spin_thread)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start Qt event loop
        app.exec_()
    
    def spin_thread(self):
        """Background thread for ROS spinning"""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DynamixelUINode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
