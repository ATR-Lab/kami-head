#!/usr/bin/env python3

import cv2
import time
import threading
import numpy as np
from typing import Optional, List, Tuple, Callable, Any

class CameraManager:
    """Camera management class for video capture"""
    
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.camera = None
        self.running = False
        self.lock = threading.Lock()
        self.capture_thread = None
        
        # Camera settings
        self.frame_width = 1280  # Default to 720p (16:9)
        self.frame_height = 720
        self.high_quality = False
        self.backend = cv2.CAP_ANY  # Default backend
        
        # Frame callback
        self.frame_callback = None
        
        # Performance tracking
        self.fps = 0
        self.frame_count = 0
        self.start_time = 0
    
    def set_callback(self, callback: Callable[[np.ndarray, float], None]):
        """Set callback function for processed frames"""
        self.frame_callback = callback
    
    def set_camera_index(self, index: int) -> bool:
        """Change the camera index"""
        if self.running:
            return False
        self.camera_index = index
        return True
    
    def set_resolution(self, width: int, height: int) -> None:
        """Set camera resolution"""
        self.frame_width = width
        self.frame_height = height
    
    def set_high_quality(self, enabled: bool) -> None:
        """Enable or disable high quality mode"""
        self.high_quality = enabled
        if enabled:
            self.frame_width = 1920
            self.frame_height = 1080
        else:
            self.frame_width = 1280
            self.frame_height = 720
    
    def start(self) -> bool:
        """Start camera capture thread"""
        if self.running:
            return False
        
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        return True
    
    def stop(self) -> None:
        """Stop camera capture thread"""
        self.running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=1.0)
            self.capture_thread = None
        
        if self.camera:
            self.camera.release()
            self.camera = None
    
    def get_fps(self) -> float:
        """Get current FPS rate"""
        return self.fps
    
    def _capture_loop(self) -> None:
        """Main camera capture loop"""
        try:
            # Open camera
            self.camera = cv2.VideoCapture(self.camera_index, self.backend)
            
            if not self.camera.isOpened():
                print(f"Error: Could not open camera {self.camera_index}")
                self.running = False
                return
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Set camera buffer size to 1 for minimal latency
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try to set camera properties for better performance
            try:
                self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
                self.camera.set(cv2.CAP_PROP_FPS, 30)       # Request 30 FPS
            except:
                # Some cameras don't support these properties
                pass
            
            # FPS tracking
            self.frame_count = 0
            self.start_time = time.time()
            
            while self.running:
                # Read frame
                ret, frame = self.camera.read()
                
                if not ret:
                    print("Error: Could not read frame from camera")
                    continue
                
                # Update FPS calculation
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                
                if elapsed_time >= 1.0:
                    self.fps = self.frame_count / elapsed_time
                    self.frame_count = 0
                    self.start_time = time.time()
                
                # Process frame using callback if available
                if self.frame_callback:
                    try:
                        self.frame_callback(frame, time.time())
                    except Exception as e:
                        print(f"Error in frame callback: {e}")
            
        except Exception as e:
            print(f"Error in camera capture loop: {e}")
        finally:
            if self.camera:
                self.camera.release()
                self.camera = None
            
            self.running = False
    
    def scan_available_cameras(self, max_cameras=10) -> List[int]:
        """Scan for available cameras"""
        available_cameras = []
        
        for i in range(max_cameras):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                available_cameras.append(i)
                cap.release()
        
        return available_cameras 