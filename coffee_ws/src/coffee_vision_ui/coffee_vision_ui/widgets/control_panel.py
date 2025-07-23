"""
Control Panel Widget

This module provides a Qt widget for camera control interface including:
- Camera device selection
- Quality settings (resolution)
- Face detection toggle
- Diagnostic controls
- Status display

The control panel emits signals for camera control actions and updates
its state based on camera status feedback.
"""

from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                       QLabel, QPushButton, QComboBox, 
                                       QCheckBox, QGroupBox, QMessageBox)
from python_qt_binding.QtCore import Qt, pyqtSignal, pyqtSlot


class ControlPanel(QWidget):
    """
    Widget providing camera control interface.
    
    This widget provides a complete control interface for camera operations,
    emitting signals for camera control actions and updating its display
    based on camera status feedback.
    
    Signals:
        camera_selected: Emitted when user selects a camera (int: camera_index)
        quality_changed: Emitted when user changes quality (bool: high_quality)
        face_detection_changed: Emitted when user toggles face detection (bool: enabled)
        refresh_cameras_requested: Emitted when user requests camera refresh
        diagnostics_requested: Emitted when user requests diagnostics
    """
    
    # Control signals
    camera_selected = pyqtSignal(int)
    quality_changed = pyqtSignal(bool)
    face_detection_changed = pyqtSignal(bool)
    refresh_cameras_requested = pyqtSignal()
    diagnostics_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        """
        Initialize control panel widget.
        
        Args:
            parent: Parent Qt widget (optional)
        """
        super().__init__(parent)
        
        # Control state
        self.available_cameras = []
        self.current_camera_index = -1
        self.high_quality = False
        self.face_detection_enabled = True
        
        # Block signals during programmatic updates
        self._updating_controls = False
        
        self._setup_ui()
        self._setup_connections()
    
    def _setup_ui(self):
        """Set up the user interface layout."""
        layout = QVBoxLayout(self)
        
        # Camera selection group
        camera_group = self._create_camera_selection_group()
        layout.addWidget(camera_group)
        
        # Settings group
        settings_group = self._create_settings_group()
        layout.addWidget(settings_group)
        
        # Diagnostics group
        diagnostics_group = self._create_diagnostics_group()
        layout.addWidget(diagnostics_group)
        
        # Status display
        self.status_label = QLabel("Status: Initializing...")
        self.status_label.setStyleSheet(
            "QLabel { "
            "color: #666; "
            "font-size: 11px; "
            "padding: 5px; "
            "background-color: #f9f9f9; "
            "border-radius: 3px; "
            "}"
        )
        layout.addWidget(self.status_label)
        
        # Add stretch to push everything to top
        layout.addStretch()
    
    def _create_camera_selection_group(self):
        """Create camera selection group box."""
        group = QGroupBox("Camera Selection")
        layout = QVBoxLayout(group)
        
        # Camera dropdown
        camera_layout = QHBoxLayout()
        camera_layout.addWidget(QLabel("Camera:"))
        
        self.camera_combo = QComboBox()
        self.camera_combo.setMinimumWidth(200)
        camera_layout.addWidget(self.camera_combo)
        
        layout.addLayout(camera_layout)
        
        # Refresh button
        self.refresh_button = QPushButton("Refresh Cameras")
        self.refresh_button.setToolTip("Scan for available camera devices")
        layout.addWidget(self.refresh_button)
        
        return group
    
    def _create_settings_group(self):
        """Create camera settings group box."""
        group = QGroupBox("Camera Settings")
        layout = QVBoxLayout(group)
        
        # Quality checkbox
        self.quality_checkbox = QCheckBox("High Quality (1080p)")
        self.quality_checkbox.setToolTip("Enable high resolution mode (1080p vs 480p)")
        self.quality_checkbox.setChecked(self.high_quality)
        layout.addWidget(self.quality_checkbox)
        
        # Face detection checkbox
        self.face_detection_checkbox = QCheckBox("Face Detection")
        self.face_detection_checkbox.setToolTip("Enable real-time face detection and tracking")
        self.face_detection_checkbox.setChecked(self.face_detection_enabled)
        layout.addWidget(self.face_detection_checkbox)
        
        return group
    
    def _create_diagnostics_group(self):
        """Create diagnostics group box."""
        group = QGroupBox("Diagnostics")
        layout = QVBoxLayout(group)
        
        self.diagnostics_button = QPushButton("Camera Diagnostics")
        self.diagnostics_button.setToolTip("Show camera and system diagnostic information")
        layout.addWidget(self.diagnostics_button)
        
        return group
    
    def _setup_connections(self):
        """Set up signal/slot connections."""
        self.camera_combo.currentIndexChanged.connect(self._on_camera_combo_changed)
        self.refresh_button.clicked.connect(self._on_refresh_clicked)
        self.quality_checkbox.stateChanged.connect(self._on_quality_changed)
        self.face_detection_checkbox.stateChanged.connect(self._on_face_detection_changed)
        self.diagnostics_button.clicked.connect(self._on_diagnostics_clicked)
    
    def _on_camera_combo_changed(self, index):
        """Handle camera combo box selection change."""
        if self._updating_controls or index < 0 or index >= len(self.available_cameras):
            return
        
        camera_index, camera_name = self.available_cameras[index]
        self.current_camera_index = camera_index
        self.camera_selected.emit(camera_index)
        self.update_status(f"Selected: {camera_name}")
    
    def _on_refresh_clicked(self):
        """Handle refresh button click."""
        self.refresh_cameras_requested.emit()
        self.update_status("Scanning for cameras...")
        self.refresh_button.setEnabled(False)  # Prevent spam clicking
    
    def _on_quality_changed(self, state):
        """Handle quality checkbox state change."""
        if self._updating_controls:
            return
        
        self.high_quality = bool(state)
        self.quality_changed.emit(self.high_quality)
        quality_text = "high quality (1080p)" if self.high_quality else "standard quality (480p)"
        self.update_status(f"Quality: {quality_text}")
    
    def _on_face_detection_changed(self, state):
        """Handle face detection checkbox state change."""
        if self._updating_controls:
            return
        
        self.face_detection_enabled = bool(state)
        self.face_detection_changed.emit(self.face_detection_enabled)
        detection_text = "enabled" if self.face_detection_enabled else "disabled"
        self.update_status(f"Face detection: {detection_text}")
    
    def _on_diagnostics_clicked(self):
        """Handle diagnostics button click."""
        self.diagnostics_requested.emit()
        self.update_status("Requesting diagnostics...")
    
    @pyqtSlot(list)
    def update_camera_list(self, cameras_list):
        """
        Update the camera selection dropdown.
        
        Args:
            cameras_list: List of (index, name) tuples for available cameras
        """
        self._updating_controls = True
        try:
            self.available_cameras = cameras_list
            self.camera_combo.clear()
            
            for index, name in cameras_list:
                self.camera_combo.addItem(name, index)
            
            # Re-enable refresh button
            self.refresh_button.setEnabled(True)
            
            if cameras_list:
                self.update_status(f"Found {len(cameras_list)} camera(s)")
                # Auto-select first camera if none selected
                if self.current_camera_index == -1:
                    self.camera_combo.setCurrentIndex(0)
            else:
                self.update_status("No cameras found")
                
        finally:
            self._updating_controls = False
    
    @pyqtSlot(str)
    def update_status(self, status_text):
        """
        Update the status display.
        
        Args:
            status_text: Status message to display
        """
        self.status_label.setText(f"Status: {status_text}")
    
    @pyqtSlot(str)
    def show_diagnostics(self, diagnostics_text):
        """
        Show diagnostics information in a popup dialog.
        
        Args:
            diagnostics_text: Diagnostic information to display
        """
        QMessageBox.information(self, "Camera Diagnostics", diagnostics_text)
    
    def set_camera_controls_enabled(self, enabled):
        """
        Enable or disable camera control widgets.
        
        Args:
            enabled: True to enable controls, False to disable
        """
        self.camera_combo.setEnabled(enabled)
        self.refresh_button.setEnabled(enabled)
        self.quality_checkbox.setEnabled(enabled)
        self.face_detection_checkbox.setEnabled(enabled)
    
    def get_current_settings(self):
        """
        Get current control panel settings.
        
        Returns:
            dict: Current settings (camera_index, quality, face_detection)
        """
        return {
            'camera_index': self.current_camera_index,
            'high_quality': self.high_quality,
            'face_detection_enabled': self.face_detection_enabled,
            'available_cameras': len(self.available_cameras)
        }
    
    def set_settings(self, settings):
        """
        Set control panel settings programmatically.
        
        Args:
            settings: Dictionary with settings to apply
        """
        self._updating_controls = True
        try:
            if 'high_quality' in settings:
                self.high_quality = settings['high_quality']
                self.quality_checkbox.setChecked(self.high_quality)
            
            if 'face_detection_enabled' in settings:
                self.face_detection_enabled = settings['face_detection_enabled']
                self.face_detection_checkbox.setChecked(self.face_detection_enabled)
            
            if 'camera_index' in settings:
                camera_index = settings['camera_index']
                # Find the combo box index for this camera index
                for i, (idx, name) in enumerate(self.available_cameras):
                    if idx == camera_index:
                        self.camera_combo.setCurrentIndex(i)
                        self.current_camera_index = camera_index
                        break
        finally:
            self._updating_controls = False 