#!/usr/bin/env python3

"""
Face detection module for coffee vision system.

This module provides face detection capabilities using OpenCV's DNN-based
face detector with temporal smoothing and visualization features.
"""

import cv2
import numpy as np
import os
import urllib.request
import time
from typing import List, Dict, Optional, Any


class FaceDetector:
    """
    OpenCV DNN-based face detector with temporal smoothing.
    
    Features:
    - DNN-based face detection using OpenCV
    - Automatic model downloading
    - Temporal smoothing to reduce detection flickering
    - Debug visualization with face overlays
    - CUDA acceleration when available
    """
    
    def __init__(
        self,
        confidence_threshold: float = 0.5,
        smoothing_factor: float = 0.4,
        models_dir: Optional[str] = None,
        logger: Optional[Any] = None
    ):
        """
        Initialize the face detector.
        
        Args:
            confidence_threshold: Minimum confidence for face detection (0.0-1.0)
            smoothing_factor: Temporal smoothing factor (0.0-1.0, higher = more smoothing)
            models_dir: Directory to store/load model files (default: ./models)
            logger: Optional logger for debug output
        """
        self.confidence_threshold = confidence_threshold
        self.smoothing_factor = smoothing_factor
        self.logger = logger
        
        # Set up models directory
        if models_dir is None:
            models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
        self.models_dir = models_dir
        os.makedirs(self.models_dir, exist_ok=True)
        
        # Face detection state
        self.face_net = None
        self.prev_faces = []
        
        # Initialize the detector
        self.init_face_detector()
    
    def init_face_detector(self):
        """Initialize the OpenCV DNN face detector"""
        try:
            # Try to get the models from disk, or download them if not present
            model_file = os.path.join(self.models_dir, "opencv_face_detector_uint8.pb")
            config_file = os.path.join(self.models_dir, "opencv_face_detector.pbtxt")
            
            # Download the model files if they don't exist
            if not os.path.exists(model_file) or not os.path.exists(config_file):
                self.download_face_model(model_file, config_file)
            
            # Load the DNN face detector
            self.face_net = cv2.dnn.readNet(model_file, config_file)
            
            # Switch to a more accurate backend if available
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                backend_info = "CUDA"
            else:
                self.face_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
                self.face_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                backend_info = "CPU"
            
            message = f"Face detector (OpenCV DNN) initialized successfully on {backend_info}"
            print(message)
            if self.logger:
                self.logger.info(message)
                
        except Exception as e:
            error_msg = f"Error initializing face detector: {e}"
            print(error_msg)
            if self.logger:
                self.logger.error(error_msg)
            self.face_net = None
    
    def download_face_model(self, model_file: str, config_file: str):
        """Download the face detection model if needed"""
        try:
            # Model URLs
            model_url = "https://github.com/spmallick/learnopencv/raw/refs/heads/master/AgeGender/opencv_face_detector_uint8.pb"
            config_url = "https://raw.githubusercontent.com/spmallick/learnopencv/refs/heads/master/AgeGender/opencv_face_detector.pbtxt"
            
            # Download the files
            message = "Downloading face detection model..."
            print(message)
            if self.logger:
                self.logger.info(message)
                
            urllib.request.urlretrieve(model_url, model_file)
            urllib.request.urlretrieve(config_url, config_file)
            
            success_msg = "Face detection model downloaded successfully"
            print(success_msg)
            if self.logger:
                self.logger.info(success_msg)
                
        except Exception as e:
            error_msg = f"Error downloading face model: {e}"
            print(error_msg)
            if self.logger:
                self.logger.error(error_msg)
            raise
    
    def detect_faces(self, frame: np.ndarray) -> List[Dict]:
        """
        Detect faces using OpenCV's DNN-based face detector.
        
        Args:
            frame: Input image as numpy array (BGR format)
            
        Returns:
            List of face dictionaries with keys: x1, y1, x2, y2, center_x, center_y, radius, confidence
        """
        if self.face_net is None:
            return []
        
        # Get frame dimensions
        h, w = frame.shape[:2]
        
        # Prepare input blob for the network
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
        self.face_net.setInput(blob)
        
        # Run forward pass
        detections = self.face_net.forward()
        
        # Parse detections
        faces = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.confidence_threshold:
                # Get face bounding box
                x1 = int(detections[0, 0, i, 3] * w)
                y1 = int(detections[0, 0, i, 4] * h)
                x2 = int(detections[0, 0, i, 5] * w)
                y2 = int(detections[0, 0, i, 6] * h)
                
                # Make sure the coordinates are within the frame
                x1 = max(0, min(x1, w-1))
                y1 = max(0, min(y1, h-1))
                x2 = max(0, min(x2, w-1))
                y2 = max(0, min(y2, h-1))
                
                if x2 > x1 and y2 > y1:  # Valid face
                    faces.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2,
                        'radius': max((x2 - x1), (y2 - y1)) // 2,
                        'confidence': confidence
                    })
        
        return faces
    
    def smooth_detections(self, faces: List[Dict]) -> List[Dict]:
        """
        Apply temporal smoothing to face detections to reduce flickering.
        
        Args:
            faces: List of face dictionaries from detect_faces()
            
        Returns:
            List of smoothed face dictionaries
        """
        if not faces:
            # If no faces detected in current frame but we have previous faces,
            # decay them but keep showing them for a while
            if self.prev_faces:
                # Slowly reduce confidence of previous faces
                for face in self.prev_faces:
                    face['confidence'] *= 0.8  # Decay factor
                
                # Remove faces with very low confidence
                self.prev_faces = [f for f in self.prev_faces if f['confidence'] > 0.2]
                return self.prev_faces
            return []
        
        # If we have new faces, smoothly transition to them
        if not self.prev_faces:
            # First detection, just use it
            self.prev_faces = faces
            return faces
        
        # Try to match new faces with previous faces
        new_faces = []
        for new_face in faces:
            # Find closest previous face
            best_match = None
            min_distance = float('inf')
            
            for i, prev_face in enumerate(self.prev_faces):
                # Calculate distance between centers
                dx = new_face['center_x'] - prev_face['center_x']
                dy = new_face['center_y'] - prev_face['center_y']
                distance = (dx*dx + dy*dy) ** 0.5
                
                if distance < min_distance:
                    min_distance = distance
                    best_match = i
            
            # If we found a close enough match, smooth the transition
            if best_match is not None and min_distance < 100:  # Threshold distance
                prev_face = self.prev_faces[best_match]
                
                # Smooth position and size
                smoothed_face = {
                    'center_x': int(self.smoothing_factor * prev_face['center_x'] + 
                                    (1 - self.smoothing_factor) * new_face['center_x']),
                    'center_y': int(self.smoothing_factor * prev_face['center_y'] + 
                                    (1 - self.smoothing_factor) * new_face['center_y']),
                    'radius': int(self.smoothing_factor * prev_face['radius'] + 
                                 (1 - self.smoothing_factor) * new_face['radius']),
                    'confidence': new_face['confidence']
                }
                
                # Calculate new bounding box from smoothed center and radius
                r = smoothed_face['radius']
                cx = smoothed_face['center_x']
                cy = smoothed_face['center_y']
                smoothed_face['x1'] = cx - r
                smoothed_face['y1'] = cy - r
                smoothed_face['x2'] = cx + r
                smoothed_face['y2'] = cy + r
                
                new_faces.append(smoothed_face)
                # Remove the matched face to prevent double matching
                self.prev_faces.pop(best_match)
            else:
                # No match, add as new face
                new_faces.append(new_face)
        
        # Add any remaining unmatched previous faces with decayed confidence
        for face in self.prev_faces:
            face['confidence'] *= 0.5  # Faster decay for unmatched faces
            if face['confidence'] > 0.3:  # Only keep if still confident enough
                new_faces.append(face)
        
        # Update previous faces for next frame
        self.prev_faces = new_faces
        return new_faces
    
    def draw_debug_overlay(self, frame: np.ndarray, faces: List[Dict]) -> np.ndarray:
        """
        Draw transparent circles and rectangles over detected faces with IDs.
        
        Args:
            frame: Input image as numpy array (BGR format)
            faces: List of face dictionaries
            
        Returns:
            Frame with face detection overlay drawn
        """
        if not faces:
            return frame
        
        # Create an overlay for transparency
        overlay = frame.copy()
        
        # Draw circles and face data on overlay
        for i, face in enumerate(faces):
            # Get face ID if available
            face_id = face.get('id', 'Unknown')
            
            # Choose color based on face ID
            if face_id != 'Unknown':
                # Use different color for recognized faces
                color = (0, 200, 255)  # Orange for recognized faces
            else:
                color = (0, 255, 0)  # Green for detected faces
            
            # Draw circle on overlay
            cv2.circle(overlay, 
                      (face['center_x'], face['center_y']), 
                      face['radius'], 
                      color, 
                      -1)
            
            # Draw rectangle around face
            cv2.rectangle(frame, (face['x1'], face['y1']), (face['x2'], face['y2']), color, 2)
            
            # Display face ID and confidence
            face_conf = face.get('confidence', 0.0)
            id_text = f"ID: {face_id}" if face_id != 'Unknown' else "Unknown"
            conf_text = f"Conf: {face_conf:.2f}"
            
            cv2.putText(frame, id_text, (face['x1'], face['y1'] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, conf_text, (face['x1'], face['y1'] + face['height'] if 'height' in face else (face['y2']-face['y1']) + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Blend the overlay with the original frame for transparency
        alpha = 0.3  # Transparency factor
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # Add indicator text if faces detected
        cv2.putText(frame, f"Faces: {len(faces)}", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display number of recognized faces
        recog_count = len([f for f in faces if f.get('id', 'Unknown') != 'Unknown'])
        if recog_count > 0:
            cv2.putText(frame, f"Recognized: {recog_count}/{len(faces)}", (10, 110), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return frame
    
    def reset_tracking(self):
        """Reset face tracking history"""
        self.prev_faces = []
    
    def is_initialized(self) -> bool:
        """Check if the face detector is properly initialized"""
        return self.face_net is not None
    
    def set_confidence_threshold(self, threshold: float):
        """Update the confidence threshold for face detection"""
        self.confidence_threshold = max(0.0, min(1.0, threshold))
    
    def set_smoothing_factor(self, factor: float):
        """Update the temporal smoothing factor"""
        self.smoothing_factor = max(0.0, min(1.0, factor)) 