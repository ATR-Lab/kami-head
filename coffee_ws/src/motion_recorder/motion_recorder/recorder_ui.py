#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Motion Recorder Qt User Interface
Provides a GUI for controlling the motion recorder node
"""

import os
import json
import time
import threading
import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

# Import custom service types
from motion_recorder_msgs.srv import SaveMotion, LoadMotion, ListMotions

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QFileDialog, QListWidget,
    QGroupBox, QProgressBar, QSplitter, QTabWidget, QMessageBox,
    QCheckBox, QSlider, QComboBox, QGraphicsView, QGraphicsScene,
    QGraphicsItem, QGraphicsRectItem, QGraphicsLineItem
)
from PyQt5.QtGui import QColor, QPen, QBrush, QPainter, QFont
from PyQt5.QtCore import Qt, QTimer, QSize, QRectF, QPointF

class MotionRecorderUI(QMainWindow):
    """Qt-based user interface for the motion recorder"""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Create timer for UI updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # 10 Hz updates
        
        # Status variables
        self.is_recording = False
        self.is_playing = False
        self.motion_name = "unnamed_motion"
        self.frame_count = 0
        self.keyframe_count = 0
        self.duration = 0.0
        self.current_time = 0.0
        
        # Create the UI
        self.init_ui()
        
        # Connect to ROS status topic
        self.status_sub = self.node.create_subscription(
            String, 'motion_recorder/status', self.status_callback, 10)
        
        # Create service clients
        self.setup_service_clients()

        # Initialize motion list
        self.load_motion_list()
    
    def init_ui(self):
        """Initialize the UI layout and components"""
        self.setWindowTitle("Motion Recorder")
        self.setMinimumSize(800, 600)
        
        # Create main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Status bar at the top
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Status: Ready")
        self.status_label.setFont(QFont("Arial", 12, QFont.Bold))
        status_layout.addWidget(self.status_label)
        
        # Display current time/duration in status
        self.time_label = QLabel("Time: 0.0s / 0.0s")
        status_layout.addWidget(self.time_label)
        
        # Add status layout to main layout
        main_layout.addLayout(status_layout)
        
        # Create tab widget for main UI sections
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)
        
        # Create recorder tab
        recorder_tab = QWidget()
        recorder_layout = QVBoxLayout(recorder_tab)
        
        # Record controls
        record_group = QGroupBox("Recording Controls")
        record_layout = QHBoxLayout(record_group)
        
        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setFont(QFont("Arial", 12))
        self.record_btn.setMinimumHeight(50)
        self.record_btn.clicked.connect(self.toggle_recording)
        record_layout.addWidget(self.record_btn)
        
        self.keyframe_btn = QPushButton("Mark Keyframe")
        self.keyframe_btn.setFont(QFont("Arial", 12))
        self.keyframe_btn.setMinimumHeight(50)
        self.keyframe_btn.clicked.connect(self.mark_keyframe)
        self.keyframe_btn.setEnabled(False)
        record_layout.addWidget(self.keyframe_btn)
        
        self.torque_btn = QPushButton("Disable Torque")
        self.torque_btn.setFont(QFont("Arial", 12))
        self.torque_btn.setMinimumHeight(50)
        self.torque_btn.clicked.connect(self.toggle_torque)
        record_layout.addWidget(self.torque_btn)
        
        # Add record controls to recorder layout
        recorder_layout.addWidget(record_group)
        
        # Save/load controls
        save_group = QGroupBox("Save/Load Motion")
        save_layout = QHBoxLayout(save_group)
        
        self.name_label = QLabel("Motion Name:")
        save_layout.addWidget(self.name_label)
        
        self.name_edit = QLineEdit("unnamed_motion")
        save_layout.addWidget(self.name_edit)
        
        self.save_btn = QPushButton("Save Motion")
        self.save_btn.clicked.connect(self.save_motion)
        save_layout.addWidget(self.save_btn)
        
        # Add save controls to recorder layout
        recorder_layout.addWidget(save_group)
        
        # Playback controls
        playback_group = QGroupBox("Playback Controls")
        playback_layout = QHBoxLayout(playback_group)
        
        self.play_btn = QPushButton("Play Motion")
        self.play_btn.setFont(QFont("Arial", 12))
        self.play_btn.setMinimumHeight(50)
        self.play_btn.clicked.connect(self.play_motion)
        playback_layout.addWidget(self.play_btn)
        
        self.stop_btn = QPushButton("Stop Playback")
        self.stop_btn.setFont(QFont("Arial", 12))
        self.stop_btn.setMinimumHeight(50)
        self.stop_btn.clicked.connect(self.stop_playback)
        self.stop_btn.setEnabled(False)
        playback_layout.addWidget(self.stop_btn)
        
        # Add playback controls to recorder layout
        recorder_layout.addWidget(playback_group)
        
        # Timeline / keyframe viewer
        timeline_group = QGroupBox("Timeline")
        timeline_layout = QVBoxLayout(timeline_group)
        
        self.timeline_view = QGraphicsView()
        self.timeline_scene = QGraphicsScene()
        self.timeline_view.setScene(self.timeline_scene)
        self.timeline_view.setMinimumHeight(100)
        timeline_layout.addWidget(self.timeline_view)
        
        # Add timeline to recorder layout
        recorder_layout.addWidget(timeline_group)
        
        # Add recorder tab to tabs
        self.tabs.addTab(recorder_tab, "Recorder")
        
        # Create motion library tab
        library_tab = QWidget()
        library_layout = QVBoxLayout(library_tab)
        
        # Motion list
        self.motion_list = QListWidget()
        self.motion_list.setFont(QFont("Arial", 12))
        self.motion_list.setAlternatingRowColors(True)
        self.motion_list.itemDoubleClicked.connect(self.load_selected_motion)
        library_layout.addWidget(self.motion_list)
        
        # Library controls
        library_controls = QHBoxLayout()
        
        self.refresh_btn = QPushButton("Refresh List")
        self.refresh_btn.clicked.connect(self.load_motion_list)
        library_controls.addWidget(self.refresh_btn)
        
        self.load_btn = QPushButton("Load Selected")
        self.load_btn.clicked.connect(self.load_selected_motion)
        library_controls.addWidget(self.load_btn)
        
        self.delete_btn = QPushButton("Delete Selected")
        self.delete_btn.clicked.connect(self.delete_selected_motion)
        library_controls.addWidget(self.delete_btn)
        
        library_layout.addLayout(library_controls)
        
        # Add library tab to tabs
        self.tabs.addTab(library_tab, "Motion Library")
        
        # Initial UI update
        self.update_ui()
    
    def setup_service_clients(self):
        """Setup ROS service clients for interacting with the recorder node"""
        # Create callback group for service clients
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # Recording services
        self.start_recording_client = self.node.create_client(
            Trigger, 'motion_recorder/start_recording',
            callback_group=self.callback_group)
        
        self.stop_recording_client = self.node.create_client(
            Trigger, 'motion_recorder/stop_recording',
            callback_group=self.callback_group)
        
        self.mark_keyframe_client = self.node.create_client(
            Trigger, 'motion_recorder/mark_keyframe',
            callback_group=self.callback_group)
        
        self.toggle_torque_client = self.node.create_client(
            SetBool, 'motion_recorder/toggle_torque',
            callback_group=self.callback_group)
        
        # Playback services
        self.play_motion_client = self.node.create_client(
            Trigger, 'motion_recorder/play_motion',
            callback_group=self.callback_group)
        
        self.stop_playback_client = self.node.create_client(
            Trigger, 'motion_recorder/stop_playback',
            callback_group=self.callback_group)
        
        # File services
        self.save_motion_client = self.node.create_client(
            SaveMotion, 'motion_recorder/save_motion',
            callback_group=self.callback_group)
        
        self.load_motion_client = self.node.create_client(
            LoadMotion, 'motion_recorder/load_motion',
            callback_group=self.callback_group)
        
        self.list_motions_client = self.node.create_client(
            ListMotions, 'motion_recorder/list_motions',
            callback_group=self.callback_group)
        
        # Wait for services to be available
        self.wait_for_services()
    
    def wait_for_services(self):
        """Wait for required services to be available"""
        required_services = [
            ('start_recording', self.start_recording_client),
            ('stop_recording', self.stop_recording_client),
            ('mark_keyframe', self.mark_keyframe_client),
            ('toggle_torque', self.toggle_torque_client),
            ('play_motion', self.play_motion_client),
            ('stop_playback', self.stop_playback_client),
            ('save_motion', self.save_motion_client),
            ('load_motion', self.load_motion_client),
            ('list_motions', self.list_motions_client)
        ]
        
        for service_name, client in required_services:
            self.node.get_logger().info(f"Waiting for {service_name} service...")
            client.wait_for_service(timeout_sec=2.0)
    
    def status_callback(self, msg):
        """Process status updates from the recorder node"""
        try:
            status = json.loads(msg.data)
            
            # Update status variables
            self.is_recording = status.get("is_recording", False)
            self.is_playing = status.get("is_playing", False)
            self.motion_name = status.get("motion_name", "unnamed_motion")
            self.frame_count = status.get("frame_count", 0)
            self.keyframe_count = status.get("keyframe_count", 0)
            self.duration = status.get("duration", 0.0)
            
            # Update UI on next timer tick
        except Exception as e:
            self.node.get_logger().error(f"Error processing status: {e}")
    
    def update_ui(self):
        """Update UI based on current status"""
        # Update status label
        if self.is_recording:
            status_text = "Status: RECORDING"
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
        elif self.is_playing:
            status_text = "Status: PLAYING"
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            status_text = "Status: Ready"
            self.status_label.setStyleSheet("")
            
        # Add frame/keyframe info
        status_text += f" | Frames: {self.frame_count} | Keyframes: {self.keyframe_count}"
        self.status_label.setText(status_text)
        
        # Update time label
        self.time_label.setText(f"Time: {self.current_time:.1f}s / {self.duration:.1f}s")
        
        # Update buttons based on recording/playback state
        self.record_btn.setText("Stop Recording" if self.is_recording else "Start Recording")
        self.record_btn.setStyleSheet("background-color: #ff6666;" if self.is_recording else "")
        
        self.keyframe_btn.setEnabled(self.is_recording)
        
        self.play_btn.setEnabled(not self.is_recording and not self.is_playing and self.frame_count > 0)
        self.stop_btn.setEnabled(self.is_playing)
        
        self.torque_btn.setText("Enable Torque" if not self.is_recording and not self.is_playing else "Disable Torque")
        
        # Update save controls
        # Only update the name_edit when it's empty or when initially loading a motion
        # This prevents overwriting user input when they're trying to edit the name
        if not self.name_edit.hasFocus() and (self.name_edit.text() == "" or self.name_edit.text() == "unnamed_motion"):
            self.name_edit.setText(self.motion_name)
            
        self.save_btn.setEnabled(self.frame_count > 0 and not self.is_recording and not self.is_playing)
        
        # Update timeline
        self.update_timeline()
    
    def update_timeline(self):
        """Update the timeline visualization"""
        # Clear the scene
        self.timeline_scene.clear()
        
        # If no frames, nothing to display
        if self.frame_count == 0:
            return
        
        # Set scene rect
        timeline_width = 700
        timeline_height = 80
        self.timeline_scene.setSceneRect(0, 0, timeline_width, timeline_height)
        
        # Draw timeline background
        bg_rect = QGraphicsRectItem(0, 0, timeline_width, timeline_height)
        bg_rect.setBrush(QBrush(QColor(240, 240, 240)))
        bg_rect.setPen(QPen(Qt.NoPen))
        self.timeline_scene.addItem(bg_rect)
        
        # Draw timeline base line
        line_y = timeline_height * 0.7
        base_line = QGraphicsLineItem(0, line_y, timeline_width, line_y)
        base_line.setPen(QPen(QColor(100, 100, 100), 2))
        self.timeline_scene.addItem(base_line)
        
        # Draw time markers
        time_marker_count = 10
        for i in range(time_marker_count + 1):
            x = i * (timeline_width / time_marker_count)
            
            # Draw marker line
            marker_line = QGraphicsLineItem(x, line_y - 5, x, line_y + 5)
            marker_line.setPen(QPen(QColor(100, 100, 100), 1))
            self.timeline_scene.addItem(marker_line)
            
            # Draw time label
            time_val = (i / time_marker_count) * self.duration
            time_text = QLabel(f"{time_val:.1f}s")
            time_text.setFont(QFont("Arial", 8))
            # Use addWidget to add the QLabel to the scene
            self.timeline_scene.addWidget(time_text)
            # Position the label
            time_text.move(int(x - 15), int(line_y + 10))
        
        # Draw current time indicator if playing
        if self.is_playing and self.duration > 0:
            current_x = (self.current_time / self.duration) * timeline_width
            current_line = QGraphicsLineItem(current_x, 0, current_x, timeline_height)
            current_line.setPen(QPen(QColor(255, 0, 0), 2))
            self.timeline_scene.addItem(current_line)
        
        # Draw keyframe markers
        # For visualization purposes only - we don't have actual keyframe data here
        # In a real implementation, we would query keyframe data from the recorder node
        marker_size = 10
        for i in range(self.keyframe_count):
            # Evenly distribute keyframes for visualization
            # In reality, they would be at their actual timestamps
            x = (i + 1) * (timeline_width / (self.keyframe_count + 1))
            
            keyframe_marker = QGraphicsRectItem(
                x - marker_size/2, line_y - marker_size/2, 
                marker_size, marker_size)
            keyframe_marker.setBrush(QBrush(QColor(0, 120, 210)))
            keyframe_marker.setPen(QPen(Qt.NoPen))
            self.timeline_scene.addItem(keyframe_marker)
    
    def toggle_recording(self):
        """Start or stop recording"""
        if not self.is_recording:
            self.start_recording()
        else:
            self.stop_recording()
    
    def start_recording(self):
        """Start recording motion"""
        # Create a thread to call the service without blocking the UI
        threading.Thread(target=self._start_recording_thread, daemon=True).start()
    
    def _start_recording_thread(self):
        """Thread function for start recording service call"""
        try:
            request = Trigger.Request()
            future = self.start_recording_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            # Log result
            if response and response.success:
                self.node.get_logger().info(f"Started recording: {response.message}")
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to start recording: {error_msg}")
                # Show error dialog (from main thread)
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Error", f"Failed to start recording: {error_msg}"))
        except Exception as e:
            self.node.get_logger().error(f"Error in start recording: {e}")
    
    def stop_recording(self):
        """Stop recording motion"""
        threading.Thread(target=self._stop_recording_thread, daemon=True).start()
    
    def _stop_recording_thread(self):
        """Thread function for stop recording service call"""
        try:
            request = Trigger.Request()
            future = self.stop_recording_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Stopped recording: {response.message}")
                
                # Prompt user to save recording
                QTimer.singleShot(0, self.prompt_save_recording)
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to stop recording: {error_msg}")
        except Exception as e:
            self.node.get_logger().error(f"Error in stop recording: {e}")
    
    def prompt_save_recording(self):
        """Prompt user to save the recording"""
        reply = QMessageBox.question(
            self, "Save Recording",
            "Do you want to save the recorded motion?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            # Switch to the correct tab
            self.tabs.setCurrentIndex(0)
            
            # Focus the name edit
            self.name_edit.selectAll()
            self.name_edit.setFocus()
            
            # Optionally highlight the save button
            self.save_btn.setStyleSheet("background-color: #66cc66;")
            QTimer.singleShot(2000, lambda: self.save_btn.setStyleSheet(""))
    
    def mark_keyframe(self):
        """Mark the current position as a keyframe"""
        threading.Thread(target=self._mark_keyframe_thread, daemon=True).start()
    
    def _mark_keyframe_thread(self):
        """Thread function for mark keyframe service call"""
        try:
            request = Trigger.Request()
            future = self.mark_keyframe_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Marked keyframe: {response.message}")
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to mark keyframe: {error_msg}")
        except Exception as e:
            self.node.get_logger().error(f"Error in mark keyframe: {e}")
    
    def toggle_torque(self):
        """Toggle motor torque on/off"""
        # Determine desired torque state based on button text
        enable_torque = "Enable" in self.torque_btn.text()
        threading.Thread(target=lambda: self._toggle_torque_thread(enable_torque), daemon=True).start()
    
    def _toggle_torque_thread(self, enable):
        """Thread function for toggle torque service call"""
        try:
            request = SetBool.Request()
            request.data = enable
            future = self.toggle_torque_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Toggled torque: {response.message}")
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to toggle torque: {error_msg}")
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Error", f"Failed to toggle torque: {error_msg}"))
        except Exception as e:
            self.node.get_logger().error(f"Error in toggle torque: {e}")
    
    def play_motion(self):
        """Play the currently loaded motion"""
        threading.Thread(target=self._play_motion_thread, daemon=True).start()
    
    def _play_motion_thread(self):
        """Thread function for play motion service call"""
        try:
            request = Trigger.Request()
            future = self.play_motion_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Playing motion: {response.message}")
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to play motion: {error_msg}")
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Error", f"Failed to play motion: {error_msg}"))
        except Exception as e:
            self.node.get_logger().error(f"Error in play motion: {e}")
    
    def stop_playback(self):
        """Stop motion playback"""
        threading.Thread(target=self._stop_playback_thread, daemon=True).start()
    
    def _stop_playback_thread(self):
        """Thread function for stop playback service call"""
        try:
            request = Trigger.Request()
            future = self.stop_playback_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Stopped playback: {response.message}")
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to stop playback: {error_msg}")
        except Exception as e:
            self.node.get_logger().error(f"Error in stop playback: {e}")
    
    def save_motion(self):
        """Save the current motion"""
        motion_name = self.name_edit.text().strip()
        if not motion_name:
            QMessageBox.warning(self, "Error", "Please enter a name for the motion")
            return
        
        # Confirm overwrite if name already exists
        if motion_name in [self.motion_list.item(i).text() for i in range(self.motion_list.count())]:
            reply = QMessageBox.question(
                self, "Confirm Overwrite",
                f"Motion '{motion_name}' already exists. Overwrite?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
        
        threading.Thread(target=lambda: self._save_motion_thread(motion_name), daemon=True).start()
    
    def _save_motion_thread(self, motion_name):
        """Thread function for save motion service call"""
        try:
            request = SaveMotion.Request()
            request.name = motion_name
            future = self.save_motion_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Saved motion: {response.message}")
                # Refresh motion list
                QTimer.singleShot(0, self.load_motion_list)
                # Show success message
                QTimer.singleShot(0, lambda: QMessageBox.information(self, "Success", f"Saved motion: {motion_name}"))
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to save motion: {error_msg}")
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Error", f"Failed to save motion: {error_msg}"))
        except Exception as e:
            self.node.get_logger().error(f"Error in save motion: {e}")
    
    def load_motion_list(self):
        """Load the list of available motions"""
        threading.Thread(target=self._load_motion_list_thread, daemon=True).start()
    
    def _load_motion_list_thread(self):
        """Thread function for list motions service call"""
        try:
            request = ListMotions.Request()
            future = self.list_motions_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                # Use the motion_list directly from the response
                motion_list = response.motion_list
                # Update UI from main thread
                QTimer.singleShot(0, lambda: self._update_motion_list(motion_list))
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to get motion list: {error_msg}")
        except Exception as e:
            self.node.get_logger().error(f"Error in load motion list: {e}")
    
    def _update_motion_list(self, motion_list):
        """Update the motion list widget with available motions"""
        self.motion_list.clear()
        for motion_name in motion_list:
            self.motion_list.addItem(motion_name)
    
    def load_selected_motion(self):
        """Load the selected motion from the list"""
        selected_items = self.motion_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Error", "Please select a motion to load")
            return
        
        motion_name = selected_items[0].text()
        threading.Thread(target=lambda: self._load_motion_thread(motion_name), daemon=True).start()
    
    def _load_motion_thread(self, motion_name):
        """Thread function for load motion service call"""
        try:
            request = LoadMotion.Request()
            request.name = motion_name
            future = self.load_motion_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            response = future.result()
            
            if response and response.success:
                self.node.get_logger().info(f"Loaded motion: {response.message}")
                # Switch to recorder tab and update motion name directly
                QTimer.singleShot(0, lambda: self.tabs.setCurrentIndex(0))
                # Update motion name directly and in the model
                QTimer.singleShot(0, lambda: self._update_loaded_motion_name(motion_name))
            else:
                error_msg = response.message if response else "Timeout"
                self.node.get_logger().error(f"Failed to load motion: {error_msg}")
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Error", f"Failed to load motion: {error_msg}"))
        except Exception as e:
            self.node.get_logger().error(f"Error in load motion: {e}")
    
    def _update_loaded_motion_name(self, motion_name):
        """Helper method to update the motion name when loading a motion"""
        # Update the internal motion name (this will be used by status updates)
        self.motion_name = motion_name
        # Force update the text field regardless of focus state
        self.name_edit.setText(motion_name)
        # Give brief visual feedback
        self.name_edit.setStyleSheet("background-color: #e6ffe6;")
        QTimer.singleShot(1000, lambda: self.name_edit.setStyleSheet(""))
    
    def delete_selected_motion(self):
        """Delete the selected motion from the list"""
        selected_items = self.motion_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Error", "Please select a motion to delete")
            return
        
        motion_name = selected_items[0].text()
        
        # Confirm deletion
        reply = QMessageBox.question(
            self, "Confirm Deletion",
            f"Delete motion '{motion_name}'? This cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # In a real implementation, we would add a delete_motion service
            # For now, just handle it on the client side
            try:
                motion_file = os.path.expanduser(f"~/.ros/motion_files/{motion_name}.json")
                if os.path.exists(motion_file):
                    os.remove(motion_file)
                    self.load_motion_list()
                    QMessageBox.information(self, "Success", f"Deleted motion: {motion_name}")
                else:
                    QMessageBox.warning(self, "Error", f"Motion file not found: {motion_name}")
            except Exception as e:
                self.node.get_logger().error(f"Error deleting motion: {e}")
                QMessageBox.warning(self, "Error", f"Failed to delete motion: {e}")

class RecorderUINode(Node):
    """ROS2 node for the motion recorder UI"""
    
    def __init__(self):
        super().__init__('motion_recorder_ui')
        
        # Create QApplication
        self.app = QApplication(sys.argv)
        
        # Create UI
        self.ui = MotionRecorderUI(self)
        self.ui.show()
        
        # Create timer for ROS processing
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0))
        self.timer.start(10)  # 100 Hz processing

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RecorderUINode()
        exit_code = node.app.exec_()
    except Exception as e:
        print(f"Error in UI node: {e}")
        exit_code = 1
    finally:
        rclpy.shutdown()
    
    return exit_code

if __name__ == '__main__':
    sys.exit(main()) 