#!/usr/bin/env python3

import cv2
import dlib
import numpy as np
import time
from typing import List, Dict, Tuple, Optional, Any

from ..common.data_types import FaceData, FrameData

class FaceDetector:
    """Face detection class using OpenCV or dlib"""
    
    def __init__(self, use_dlib=False):
        self.use_dlib = use_dlib
        self.face_detector = None
        self.face_net = None
        self.confidence_threshold = 0.5
        
        # Initialize detector
        self._init_detector()
        
        # Smoothing for face detection
        self.prev_faces = []
        self.smoothing_factor = 0.7  # Higher value = more smoothing
        self.smoothing_frames = 5    # Number of frames to average
    
    def _init_detector(self) -> None:
        """Initialize face detector based on selected method"""
        if self.use_dlib:
            # Use dlib's HOG-based face detector
            self.face_detector = dlib.get_frontal_face_detector()
        else:
            # Use OpenCV DNN face detector
            model_file = "face_detector/opencv_face_detector_uint8.pb"
            config_file = "face_detector/opencv_face_detector.pbtxt"
            
            try:
                self.face_net = cv2.dnn.readNetFromTensorflow(model_file, config_file)
            except Exception as e:
                print(f"Error loading face detection model: {e}")
                print("Falling back to Haar Cascade face detector")
                self.face_net = None
                self.face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 
                                                        'haarcascade_frontalface_default.xml')
    
    def set_confidence_threshold(self, threshold: float) -> None:
        """Set confidence threshold for face detection"""
        self.confidence_threshold = max(0.1, min(threshold, 0.99))
    
    def detect_faces(self, frame: np.ndarray) -> List[FaceData]:
        """Detect faces in the frame and return in standard format"""
        if self.use_dlib:
            return self._detect_faces_dlib(frame)
        elif self.face_net is not None:
            return self._detect_faces_dnn(frame)
        else:
            return self._detect_faces_haar(frame)
    
    def _detect_faces_dnn(self, frame: np.ndarray) -> List[FaceData]:
        """Detect faces using OpenCV DNN face detector"""
        if frame is None or self.face_net is None:
            return []
        
        faces = []
        frame_height, frame_width = frame.shape[:2]
        
        # Create a blob from the frame
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
        
        # Set the input and run forward pass
        self.face_net.setInput(blob)
        detections = self.face_net.forward()
        
        # Process detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            if confidence > self.confidence_threshold:
                # Get coordinates
                x1 = int(detections[0, 0, i, 3] * frame_width)
                y1 = int(detections[0, 0, i, 4] * frame_height)
                x2 = int(detections[0, 0, i, 5] * frame_width)
                y2 = int(detections[0, 0, i, 6] * frame_height)
                
                # Ensure coordinates are within frame bounds
                x1 = max(0, min(x1, frame_width - 1))
                y1 = max(0, min(y1, frame_height - 1))
                x2 = max(0, min(x2, frame_width - 1))
                y2 = max(0, min(y2, frame_height - 1))
                
                # Calculate center
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                
                # Create standard FaceData
                face = FaceData(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    center_x=center_x,
                    center_y=center_y,
                    confidence=float(confidence)
                )
                
                faces.append(face)
        
        return faces
    
    def _detect_faces_haar(self, frame: np.ndarray) -> List[FaceData]:
        """Detect faces using Haar Cascade classifier"""
        if frame is None or self.face_detector is None:
            return []
        
        # Convert to grayscale for Haar detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        detections = self.face_detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        faces = []
        for (x, y, w, h) in detections:
            # Calculate coordinates
            x1, y1 = x, y
            x2, y2 = x + w, y + h
            center_x = x1 + w // 2
            center_y = y1 + h // 2
            
            # Create standard FaceData
            face = FaceData(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                center_x=center_x,
                center_y=center_y,
                confidence=0.9  # Haar doesn't provide confidence
            )
            
            faces.append(face)
        
        return faces
    
    def _detect_faces_dlib(self, frame: np.ndarray) -> List[FaceData]:
        """Detect faces using dlib's HOG face detector"""
        if frame is None or self.face_detector is None:
            return []
        
        # Convert BGR to RGB (dlib expects RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect faces
        detections = self.face_detector(rgb_frame, 1)
        
        faces = []
        for detection in detections:
            # Get coordinates from rectangle
            x1 = detection.left()
            y1 = detection.top()
            x2 = detection.right()
            y2 = detection.bottom()
            
            # Calculate center
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Create standard FaceData
            face = FaceData(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                center_x=center_x,
                center_y=center_y,
                confidence=0.9  # dlib doesn't provide confidence
            )
            
            faces.append(face)
        
        return faces
    
    def apply_smoothing(self, faces: List[FaceData]) -> List[FaceData]:
        """Apply smoothing to face detections to reduce jitter"""
        if not faces:
            self.prev_faces = []
            return []
        
        # If we don't have previous faces, just return the current ones
        if not self.prev_faces:
            self.prev_faces = faces
            return faces
        
        # If counts don't match, just return the current faces
        if len(faces) != len(self.prev_faces):
            self.prev_faces = faces
            return faces
        
        # Apply smoothing to each face
        smoothed_faces = []
        for i, face in enumerate(faces):
            prev_face = self.prev_faces[i]
            
            # Smooth coordinates
            smoothed_x1 = int(prev_face.x1 * self.smoothing_factor + face.x1 * (1 - self.smoothing_factor))
            smoothed_y1 = int(prev_face.y1 * self.smoothing_factor + face.y1 * (1 - self.smoothing_factor))
            smoothed_x2 = int(prev_face.x2 * self.smoothing_factor + face.x2 * (1 - self.smoothing_factor))
            smoothed_y2 = int(prev_face.y2 * self.smoothing_factor + face.y2 * (1 - self.smoothing_factor))
            
            # Calculate center
            smoothed_center_x = (smoothed_x1 + smoothed_x2) // 2
            smoothed_center_y = (smoothed_y1 + smoothed_y2) // 2
            
            # Create smoothed face
            smoothed_face = FaceData(
                x1=smoothed_x1,
                y1=smoothed_y1,
                x2=smoothed_x2,
                y2=smoothed_y2,
                center_x=smoothed_center_x,
                center_y=smoothed_center_y,
                confidence=face.confidence
            )
            
            smoothed_faces.append(smoothed_face)
        
        # Update previous faces
        self.prev_faces = smoothed_faces
        
        return smoothed_faces
    
    def draw_faces(self, frame: np.ndarray, faces: List[FaceData], 
                   color=(0, 255, 0), thickness=2) -> np.ndarray:
        """Draw face bounding boxes on the frame"""
        for face in faces:
            # Draw rectangle
            cv2.rectangle(
                frame, 
                (face.x1, face.y1), 
                (face.x2, face.y2), 
                color, 
                thickness
            )
            
            # Draw center point
            cv2.circle(
                frame,
                (face.center_x, face.center_y),
                3,
                (0, 0, 255),
                -1
            )
            
            # Draw confidence if available
            cv2.putText(
                frame,
                f"{face.confidence:.2f}",
                (face.x1, face.y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1
            )
        
        return frame 