#!/usr/bin/env python3

import os
import sys
import time
import json
import numpy as np
import cv2
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QGridLayout, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer, Qt, QSize

# Import face recognition core class
from .recognition_memory import RecogniserBN

# Import data types
from .data_types import FaceData

# Add these imports at the top of the file
import psutil
import gc


class FaceMemory:
    """Manages face recognition and memory"""
    
    def __init__(self, data_dir):
        """Initialize the face recognition system"""
        self.data_dir = data_dir
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Sub-directories
        self.db_dir = os.path.join(self.data_dir, 'db')
        self.faces_dir = os.path.join(self.data_dir, 'faces')
        self.model_dir = os.path.join(self.data_dir, 'models')
        os.makedirs(self.db_dir, exist_ok=True)
        os.makedirs(self.faces_dir, exist_ok=True)
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Initialize RecogniserBN
        self.rb = RecogniserBN()
        self.rb.setFilePaths(self.data_dir)
        self.rb.isMultipleRecognitions = False
        self.rb.isMemoryRobot = False  # We're not using the robot features
        self.rb.isDBinCSV = True
        self.rb.setSilentMode()  # No speaking
        
        # Initialize face database
        self.db_file = os.path.join(self.data_dir, 'db.csv')
        self.face_db_file = 'faceDB'
        self.model_file = os.path.join(self.model_dir, 'face_model')
        
        # Resource limits and cleanup settings
        self.max_faces_to_track = 10  # Maximum number of faces to keep in memory
        self.max_stored_images = 1000  # Maximum images to store on disk before cleanup
        self.cleanup_interval = 600  # Cleanup old images every 10 minutes
        self.last_cleanup = time.time()
        self.max_memory_usage_mb = 500  # Maximum allowed memory usage in MB
        
        # Training parameters
        self.training_faces = {}  # id -> list of face images for training
        self.training_count_target = 250  # Target number of images per face
        self.is_training = False  # Flag to indicate active training
        self.last_model_save = 0  # Timestamp of last model save
        self.model_save_interval = 300  # Save model every 5 minutes
        
        # Check if model exists, otherwise create it
        if os.path.exists(self.model_file):
            print(f"Loading existing face model from {self.model_file}")
            self.rb.loadModel(self.model_file)
        
        # Check if face database exists, otherwise create it
        if os.path.isfile(os.path.join(self.data_dir, self.face_db_file)):
            self.rb.useFaceDetectionDB(self.face_db_file)
        else:
            self.rb.resetFaceDetectionDB()
            self.rb.setFaceDetectionDB(self.face_db_file)
        
        # Recognition parameters
        self.next_id = 1
        self.recognized_faces = {}  # id -> face data
        self.recognition_confidence_threshold = 0.5
        self.last_image = None
        self.current_frame = None
        self.processing_lock = threading.Lock()
        
        # Debug stats
        self.stats = {
            'total_faces_processed': 0,
            'total_recognitions': 0,
            'new_faces_learned': 0,
            'last_recognition_time': 0,
            'avg_recognition_time': 0,
            'learning_rate': 0.5,  # How quickly we adapt to new faces
            'last_process_timestamp': 0,
            'last_model_save': 0,
            'training_faces': {},
            'memory_usage_mb': 0,
            'last_cleanup_time': 0,
            'cleanup_count': 0
        }
        
        # Load existing database
        self.load_db()
        
        # Do an initial cleanup
        self.cleanup_old_files()
    
    def load_db(self):
        """Load the face database"""
        self.rb.loadDB(self.db_file)
        
        # Get the highest ID in the database to set next_id
        highest_id = 0
        if hasattr(self.rb, 'people'):
            for person in self.rb.people:
                if person and person[0]:
                    try:
                        person_id = int(person[0])
                        highest_id = max(highest_id, person_id)
                    except ValueError:
                        pass
        
        self.next_id = highest_id + 1
        
        # Update stats
        self.stats['total_faces_processed'] = len(self.rb.people) if hasattr(self.rb, 'people') else 0
        self.stats['new_faces_learned'] = self.stats['total_faces_processed']
    
    def save_model(self, force=False):
        """Save the face recognition model"""
        current_time = time.time()
        
        # Only save if enough time has passed or if forced
        if force or (current_time - self.last_model_save > self.model_save_interval):
            try:
                self.rb.saveModel(self.model_file)
                self.last_model_save = current_time
                self.stats['last_model_save'] = current_time
                print(f"Face model saved to {self.model_file}")
                return True
            except Exception as e:
                print(f"Error saving face model: {e}")
                return False
        return False
    
    def add_training_face(self, face_id, face_img):
        """Add a face image for training"""
        if face_id not in self.training_faces:
            self.training_faces[face_id] = []
            self.stats['training_faces'][face_id] = 0
        
        # Check memory usage before adding more images
        if self.check_memory_usage() > self.max_memory_usage_mb:
            print(f"WARNING: Memory usage too high ({self.check_memory_usage()} MB). Skipping training image.")
            return False
            
        # Convert to grayscale if not already
        if len(face_img.shape) == 3:
            gray = cv2.cvtColor(face_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = face_img
            
        # Resize to smaller resolution to save memory (e.g., 100x100)
        gray = cv2.resize(gray, (100, 100))
        
        # Only add every 5th image to avoid too many similar frames
        if len(self.training_faces[face_id]) % 5 == 0:
            # Add to training set
            self.training_faces[face_id].append(gray)
            self.stats['training_faces'][face_id] = len(self.training_faces[face_id])
            
            # Save training image
            training_dir = os.path.join(self.faces_dir, f'training_{face_id}')
            os.makedirs(training_dir, exist_ok=True)
            img_path = os.path.join(training_dir, f'train_{len(self.training_faces[face_id])}.jpg')
            cv2.imwrite(img_path, gray)
            
            # Check if it's time to cleanup
            current_time = time.time()
            if current_time - self.last_cleanup > self.cleanup_interval:
                self.cleanup_old_files()
        
        # Check if we have enough images for training
        if len(self.training_faces[face_id]) >= self.training_count_target:
            self.train_face(face_id)
            return True
        
        return False
    
    def train_face(self, face_id):
        """Train the model with collected face images"""
        if face_id not in self.training_faces or len(self.training_faces[face_id]) < 10:
            print(f"Not enough training images for face {face_id}")
            return False
        
        print(f"Training face {face_id} with {len(self.training_faces[face_id])} images")
        self.is_training = True
        
        try:
            # Create a person entry if it doesn't exist
            person_exists = False
            if hasattr(self.rb, 'people'):
                for person in self.rb.people:
                    if person and person[0] == face_id:
                        person_exists = True
                        break
            
            if not person_exists:
                # Add person to recognition database
                person = [face_id, "", "", 0, 0]
                self.rb.addPersonToBN(person)
            
            # Train with each image
            for img in self.training_faces[face_id]:
                # Save to a temporary file
                temp_path = os.path.join(self.faces_dir, f'temp_train_{face_id}.jpg')
                cv2.imwrite(temp_path, img)
                
                # Train the model
                self.rb.setImageToCopy(temp_path)
                self.rb.isRegistered = False
                self.rb.identity_est = face_id
                self.rb.confirmPersonIdentity(face_id)
            
            # Save model after training
            self.save_model(force=True)
            
            # Clear training data after successful training
            self.training_faces[face_id] = []
            
            print(f"Successfully trained face {face_id}")
            return True
            
        except Exception as e:
            print(f"Error training face {face_id}: {e}")
            return False
        finally:
            self.is_training = False
    
    def cleanup_old_files(self):
        """Cleanup old face images to prevent disk space issues"""
        try:
            self.last_cleanup = time.time()
            self.stats['last_cleanup_time'] = self.last_cleanup
            
            # Get list of all image files
            image_files = []
            for root, dirs, files in os.walk(self.faces_dir):
                for file in files:
                    if file.endswith('.jpg'):
                        full_path = os.path.join(root, file)
                        image_files.append((full_path, os.path.getmtime(full_path)))
            
            # If we have more than the maximum, delete the oldest
            if len(image_files) > self.max_stored_images:
                # Sort by modification time (oldest first)
                image_files.sort(key=lambda x: x[1])
                
                # Delete oldest files exceeding the limit
                files_to_delete = len(image_files) - self.max_stored_images
                for i in range(files_to_delete):
                    try:
                        os.remove(image_files[i][0])
                        self.stats['cleanup_count'] += 1
                    except:
                        pass
                
                print(f"Cleaned up {files_to_delete} old image files")
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    def check_memory_usage(self):
        """Check current memory usage of the process in MB"""
        try:
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024  # Convert to MB
            self.stats['memory_usage_mb'] = memory_mb
            return memory_mb
        except:
            # If psutil is not available, return a conservative estimate
            return 0  # Will assume we're under limit
    
    def manage_recognized_faces(self):
        """Limit the number of recognized faces in memory"""
        if len(self.recognized_faces) > self.max_faces_to_track:
            # Sort faces by last update time (oldest first)
            sorted_faces = sorted(
                self.recognized_faces.items(),
                key=lambda x: x[1].get('last_updated', 0)
            )
            
            # Keep only the most recently seen faces
            faces_to_remove = len(sorted_faces) - self.max_faces_to_track
            for i in range(faces_to_remove):
                face_id = sorted_faces[i][0]
                if face_id in self.recognized_faces:
                    del self.recognized_faces[face_id]
    
    def process_frame(self, frame, faces):
        """Process a frame with detected faces for recognition"""
        if not faces or frame is None:
            return {}
        
        start_time = time.time()
        self.stats['last_process_timestamp'] = start_time
        recognized_faces = {}
        
        # Check memory usage - skip processing if too high
        if self.check_memory_usage() > self.max_memory_usage_mb:
            print(f"WARNING: Memory usage too high ({self.stats['memory_usage_mb']} MB). Skipping face processing.")
            return self.recognized_faces
        
        with self.processing_lock:
            # Store a resized version of the frame to save memory
            h, w = frame.shape[:2]
            scale_factor = min(1.0, 640 / max(w, h))  # Limit to 640px in largest dimension
            if scale_factor < 1.0:
                self.current_frame = cv2.resize(frame, (int(w * scale_factor), int(h * scale_factor)))
            else:
                self.current_frame = frame.copy()
            
            for face_idx, face_data in enumerate(faces):
                # Process at most max_faces_to_track faces per frame
                if face_idx >= self.max_faces_to_track:
                    break
                    
                face_img = self.extract_face_image(frame, face_data)
                
                if face_img is not None:
                    self.stats['total_faces_processed'] += 1
                    
                    # Save the face image for recognition - use a consistent filename to avoid proliferation
                    face_image_path = os.path.join(self.faces_dir, f'temp_face_{face_idx}.jpg')
                    cv2.imwrite(face_image_path, face_img)
                    
                    # Set image for recognition
                    self.rb.setImageToCopy(face_image_path)
                    
                    # Perform recognition
                    recognition_start = time.time()
                    self.rb.num_recognitions += 1
                    self.stats['total_recognitions'] += 1
                    recog_results = self.rb.recognisePerson()
                    recognition_time = time.time() - recognition_start
                    
                    # Update timing stats
                    self.stats['last_recognition_time'] = recognition_time
                    if self.stats['avg_recognition_time'] == 0:
                        self.stats['avg_recognition_time'] = recognition_time
                    else:
                        self.stats['avg_recognition_time'] = (0.9 * self.stats['avg_recognition_time'] + 
                                                              0.1 * recognition_time)
                    
                    # Get the recognition results
                    identity = self.rb.identity_est
                    confidence = 0.0
                    is_new_face = False
                    
                    # Process recognition outcome
                    if identity and identity != "0":  # Known face
                        confidence = self.rb.quality_estimate
                        face_id = identity
                        
                        # Every time we recognize a face, update the model to improve recognition
                        # This is a form of continuous learning
                        self.rb.confirmPersonIdentity(face_id)
                        
                        # Add to training set for continuous improvement - but limit frequency
                        if self.stats['total_recognitions'] % 30 == 0:  # Only add every 30th recognition
                            training_complete = self.add_training_face(face_id, face_img)
                            if training_complete:
                                self.get_logger().info(f"Completed training for face ID: {face_id} with {self.training_count_target} images")
                                self.ui.add_log_message(f"Completed training for face ID: {face_id}")
                        
                        self.get_logger().info(f"Recognized existing face - ID: {face_id}, Confidence: {confidence:.2f}")
                    else:  # Unknown face, add to database if we're under the limit
                        # Only add new faces if we're under the limit
                        if len(self.recognized_faces) < self.max_faces_to_track:
                            # Assign a new ID
                            face_id = str(self.next_id)
                            self.next_id += 1
                            is_new_face = True
                            self.stats['new_faces_learned'] += 1
                            
                            # Create a person entry with just an ID
                            person = [face_id, "", "", 0, 0]
                            self.rb.addPersonToBN(person)
                            
                            # Now update the database with the face
                            self.rb.isRegistered = False
                            self.rb.identity_est = face_id
                            self.rb.confirmPersonIdentity(face_id)
                            
                            # Start collecting training images for this new face
                            self.add_training_face(face_id, face_img)
                        else:
                            # Skip new faces if we're already tracking too many
                            continue
                    
                    # Store additional debug info
                    face_shape = face_img.shape
                    face_quality = self._calculate_face_quality(face_img)
                    
                    # Store a downsampled version of the face image to save memory
                    face_img_small = cv2.resize(face_img, (100, 100))
                    
                    # Store the recognition result with debug info
                    recognized_faces[face_id] = {
                        'face_data': face_data,
                        'confidence': confidence,
                        'face_image': face_img_small,  # Use smaller face image
                        'is_new': is_new_face,
                        'recognition_time': recognition_time,
                        'face_size': f"{face_shape[1]}x{face_shape[0]}",
                        'face_quality': face_quality,
                        'learning_status': "Learned (New)" if is_new_face else "Recognized (Updated)",
                        'last_updated': time.time(),
                        'training_images': len(self.training_faces.get(face_id, [])),
                        'training_target': self.training_count_target
                    }
                    
                    # Only save face image to disk occasionally to avoid filling up storage
                    if self.stats['total_recognitions'] % 10 == 0:
                        timestamp = int(time.time())
                        face_filename = f"face_{face_id}_{timestamp}.jpg"
                        face_path = os.path.join(self.faces_dir, face_filename)
                        cv2.imwrite(face_path, face_img_small)
            
            # Update the recognized faces dictionary
            self.recognized_faces.update(recognized_faces)
            
            # Manage the number of faces we're tracking
            self.manage_recognized_faces()
            
            # Check if it's time to save the model
            current_time = time.time()
            if current_time - self.last_model_save > self.model_save_interval:
                self.save_model()
        
        self.stats['processing_time'] = time.time() - start_time
        return recognized_faces
    
    def _calculate_face_quality(self, face_img):
        """Calculate a quality metric for the face image (0-100)"""
        # Simple quality metric based on variance (sharpness)
        if face_img is None or face_img.size == 0:
            return 0
            
        # Convert to grayscale if needed
        if len(face_img.shape) == 3:
            gray = cv2.cvtColor(face_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = face_img
            
        # Laplacian variance as sharpness measure
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = laplacian.var()
        
        # Normalize to 0-100 range
        quality = min(100, max(0, int(sharpness / 500 * 100)))
        
        return quality
    
    def extract_face_image(self, frame, face_data):
        """Extract face image from the frame using face coordinates"""
        try:
            # Get face coordinates
            x1, y1, x2, y2 = face_data.x1, face_data.y1, face_data.x2, face_data.y2
            
            # Add some margin
            height, width = frame.shape[:2]
            margin = int(min(face_data.width, face_data.height) * 0.2)
            
            # Ensure coordinates are within bounds
            x1 = max(0, x1 - margin)
            y1 = max(0, y1 - margin)
            x2 = min(width, x2 + margin)
            y2 = min(height, y2 + margin)
            
            # Extract face image
            face_img = frame[y1:y2, x1:x2]
            
            if face_img.size == 0:
                return None
            
            return face_img
        except Exception as e:
            print(f"Error extracting face: {e}")
            return None


class FaceRecognitionUI(QMainWindow):
    """UI for displaying recognized faces"""
    
    def __init__(self, node, face_memory):
        super().__init__()
        self.node = node
        self.face_memory = face_memory
        self.bridge = CvBridge()
        
        # Setup UI
        self.setWindowTitle("Face Recognition Memory")
        self.setGeometry(100, 100, 1200, 800)  # Larger window for more debug information
        
        # Main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # Create horizontal layout for left and right panes
        self.h_layout = QHBoxLayout()
        self.main_layout.addLayout(self.h_layout, 1)
        
        # Left pane - Face recognition grid
        self.left_pane = QWidget()
        self.left_layout = QVBoxLayout(self.left_pane)
        
        # Title for faces grid
        self.faces_title = QLabel("Recognized Faces:")
        self.faces_title.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.left_layout.addWidget(self.faces_title)
        
        # Create grid for face display
        self.faces_scroll = QWidget()
        self.faces_grid = QGridLayout(self.faces_scroll)
        self.left_layout.addWidget(self.faces_scroll, 1)
        
        # Right pane - Debug information
        self.right_pane = QWidget()
        self.right_layout = QVBoxLayout(self.right_pane)
        
        # Current frame display
        self.frame_title = QLabel("Current Frame:")
        self.frame_title.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.right_layout.addWidget(self.frame_title)
        
        self.frame_label = QLabel()
        self.frame_label.setFixedSize(QSize(400, 300))
        self.frame_label.setScaledContents(True)
        self.frame_label.setStyleSheet("border: 1px solid gray;")
        self.right_layout.addWidget(self.frame_label)
        
        # Stats panel
        self.stats_title = QLabel("Recognition Stats:")
        self.stats_title.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.right_layout.addWidget(self.stats_title)
        
        self.stats_panel = QLabel()
        self.stats_panel.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        self.stats_panel.setWordWrap(True)
        self.right_layout.addWidget(self.stats_panel)
        
        # Debug log panel
        self.log_title = QLabel("Debug Log:")
        self.log_title.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.right_layout.addWidget(self.log_title)
        
        self.log_panel = QLabel()
        self.log_panel.setStyleSheet("background-color: #f0f0f0; padding: 10px; font-family: monospace; border-radius: 5px;")
        self.log_panel.setWordWrap(True)
        self.right_layout.addWidget(self.log_panel)
        
        # Add panes to horizontal layout
        self.h_layout.addWidget(self.left_pane, 1)
        self.h_layout.addWidget(self.right_pane, 1)
        
        # Status bar at bottom
        self.status_label = QLabel("Waiting for faces...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.status_label)
        
        # Memory grid for storing face widgets
        self.face_labels = {}
        self.face_info_labels = {}
        
        # Debug log storage
        self.log_messages = []
        self.max_log_messages = 10  # Maximum number of log messages to show
        
        # Timer for UI updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # Update 10 times per second
        
        # Recognized faces data
        self.recognized_faces = {}
        
        # Add initial log message
        self.add_log_message("Face recognition system initialized")
    
    def add_log_message(self, message):
        """Add a message to the debug log with timestamp"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[{timestamp}] {message}"
        self.log_messages.append(log_entry)
        
        # Keep log at max length
        if len(self.log_messages) > self.max_log_messages:
            self.log_messages = self.log_messages[-self.max_log_messages:]
        
        # Update log panel
        self.log_panel.setText("\n".join(self.log_messages))
    
    def update_ui(self):
        """Update the UI with recognized faces and debug info"""
        # Update connection status in the stats panel if available
        if hasattr(self.node, 'connection_status'):
            conn = self.node.connection_status
            camera_status = "Connected" if conn.get('camera_frame', False) else "Disconnected"
            face_status = "Connected" if conn.get('face_detection', False) else "Disconnected"
            
            frames_received = conn.get('frames_received', 0)
            face_data_received = conn.get('face_data_received', 0)
        else:
            camera_status = "Unknown"
            face_status = "Unknown"
            frames_received = 0
            face_data_received = 0
            
        # Update current frame if available
        if hasattr(self.face_memory, 'current_frame') and self.face_memory.current_frame is not None:
            frame = self.face_memory.current_frame
            
            # Draw face boxes on the frame
            if hasattr(self.face_memory, 'recognized_faces'):
                display_frame = frame.copy()
                for face_id, face_data in self.face_memory.recognized_faces.items():
                    fd = face_data['face_data']
                    # Draw face rectangle
                    cv2.rectangle(display_frame, (fd.x1, fd.y1), (fd.x2, fd.y2), (0, 255, 0), 2)
                    # Draw face ID
                    cv2.putText(display_frame, f"ID:{face_id}", (fd.x1, fd.y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                display_frame = frame
                
            # Convert to QImage and display
            h, w = display_frame.shape[:2]
            bytes_per_line = 3 * w
            q_img = QImage(display_frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.frame_label.setPixmap(QPixmap.fromImage(q_img))
        
        # Update stats panel
        if hasattr(self.face_memory, 'stats'):
            stats = self.face_memory.stats
            
            # Format training info
            training_info = ""
            for face_id, count in stats.get('training_faces', {}).items():
                if count > 0:
                    training_info += f"- Face ID {face_id}: {count}/{self.face_memory.training_count_target}\n"
            
            if not training_info:
                training_info = "- No faces in training\n"
            
            # Format model save info
            last_save = time.strftime('%H:%M:%S', time.localtime(stats.get('last_model_save', 0))) if stats.get('last_model_save', 0) else "Not saved yet"
            
            stats_text = (
                f"Connection Status:\n"
                f"- Camera Frame: {camera_status} (Frames: {frames_received})\n"
                f"- Face Detection: {face_status} (Updates: {face_data_received})\n\n"
                f"Recognition Stats:\n"
                f"- Total Faces Processed: {stats.get('total_faces_processed', 0)}\n"
                f"- Total Recognitions: {stats.get('total_recognitions', 0)}\n"
                f"- New Faces Learned: {stats.get('new_faces_learned', 0)}\n"
                f"- Recognition Time: {stats.get('last_recognition_time', 0):.3f}s\n"
                f"- Avg Recognition Time: {stats.get('avg_recognition_time', 0):.3f}s\n"
                f"- Processing Time: {stats.get('processing_time', 0):.3f}s\n"
                f"- Last Update: {time.strftime('%H:%M:%S', time.localtime(stats.get('last_process_timestamp', 0)))}\n\n"
                f"Model Status:\n"
                f"- Last Model Save: {last_save}\n\n"
                f"Training Progress:\n"
                f"{training_info}"
            )
            self.stats_panel.setText(stats_text)
            
            # Update status bar with connection information
            if not conn.get('camera_frame', False) or not conn.get('face_detection', False):
                self.status_label.setText("⚠️ Connection issue - check camera node")
                self.status_label.setStyleSheet("color: red; font-weight: bold;")
            elif not self.face_memory.recognized_faces:
                self.status_label.setText("Waiting for faces...")
                self.status_label.setStyleSheet("")
            else:
                self.status_label.setText(f"Recognized {len(self.face_memory.recognized_faces)} faces")
                self.status_label.setStyleSheet("")
        
        # Update faces grid
        if hasattr(self.face_memory, 'recognized_faces'):
            faces = self.face_memory.recognized_faces
            
            # Update status
            if not faces:
                self.status_label.setText("Waiting for faces...")
            else:
                self.status_label.setText(f"Recognized {len(faces)} faces")
                # Add log message if new faces are detected
                for face_id, face_data in faces.items():
                    if face_id not in self.face_labels and face_data.get('is_new', False):
                        self.add_log_message(f"New face learned: ID {face_id}")
            
            # Clear grid if face count changes
            if len(faces) != len(self.face_labels):
                self.clear_face_grid()
            
            # Update face grid
            row, col = 0, 0
            for face_id, face_data in faces.items():
                if face_id not in self.face_labels:
                    # Create new label for this face
                    face_img_label = QLabel()
                    face_img_label.setFixedSize(QSize(150, 150))
                    face_img_label.setScaledContents(True)
                    face_img_label.setStyleSheet("border: 2px solid " + 
                                                ("red" if face_data.get('is_new', False) else "green"))
                    
                    face_info_label = QLabel()
                    face_info_label.setAlignment(Qt.AlignCenter)
                    face_info_label.setWordWrap(True)
                    
                    # Add to grid
                    self.faces_grid.addWidget(face_img_label, row, col)
                    self.faces_grid.addWidget(face_info_label, row+1, col)
                    
                    # Store references
                    self.face_labels[face_id] = face_img_label
                    self.face_info_labels[face_id] = face_info_label
                    
                    # Move to next position
                    col += 1
                    if col >= 3:  # 3 faces per row
                        col = 0
                        row += 2  # Each face takes 2 rows (image + label)
                
                # Update image
                if 'face_image' in face_data:
                    face_img = face_data['face_image']
                    h, w = face_img.shape[:2]
                    
                    q_img = QImage(face_img.data, w, h, w*3, QImage.Format_RGB888).rgbSwapped()
                    pixmap = QPixmap.fromImage(q_img)
                    
                    self.face_labels[face_id].setPixmap(pixmap)
                    
                    # Update info with extended details
                    confidence = face_data.get('confidence', 0.0)
                    is_new = face_data.get('is_new', False)
                    learning_status = face_data.get('learning_status', "")
                    face_size = face_data.get('face_size', "")
                    face_quality = face_data.get('face_quality', 0)
                    
                    # Get training progress
                    training_count = len(self.face_memory.training_faces.get(face_id, []))
                    training_target = self.face_memory.training_count_target
                    training_progress = f"{training_count}/{training_target}"
                    
                    status = "New Face" if is_new else f"Confidence: {confidence:.2f}"
                    info_text = (
                        f"ID: {face_id}\n"
                        f"{status}\n"
                        f"Training: {training_progress}\n"
                        f"Quality: {face_quality}%\n"
                        f"Size: {face_size}"
                    )
                    self.face_info_labels[face_id].setText(info_text)
    
    def clear_face_grid(self):
        """Clear all faces from the grid"""
        for face_label in self.face_labels.values():
            face_label.setParent(None)
        
        for info_label in self.face_info_labels.values():
            info_label.setParent(None)
        
        self.face_labels = {}
        self.face_info_labels = {}
    
    def update_recognized_faces(self, faces):
        """Update the recognized faces data"""
        self.recognized_faces = faces


class FaceRecognitionNode(Node):
    """ROS node for face recognition and memory"""
    
    def __init__(self):
        super().__init__('face_recognition_node')
        self.get_logger().info('Face recognition node starting...')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Get data directory
        self.data_dir = os.path.join(os.path.expanduser('~'), '.coffee_head', 'face_recognition')
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Initialize face recognition memory
        self.face_memory = FaceMemory(self.data_dir)
        
        # State variables
        self.latest_frame = None
        self.latest_faces = []
        self.latest_face_images = {}  # face_id -> face_image
        self.has_new_data = False
        self.processing = False
        self.connection_status = {
            'camera_frame': False,
            'face_detection': False,
            'face_images': False,
            'last_frame_time': 0,
            'last_face_data_time': 0,
            'last_face_image_time': 0,
            'frames_received': 0,
            'face_data_received': 0,
            'face_images_received': 0
        }
        
        # Rate limiting
        self.process_every_n_frames = 3  # Only process every 3rd frame
        self.frame_counter = 0
        
        # Garbage collection timer
        self.gc_timer = self.create_timer(
            30.0,  # Run garbage collection every 30 seconds
            self.run_garbage_collection
        )
        
        # Resource monitoring timer
        self.resource_timer = self.create_timer(
            10.0,  # Check resources every 10 seconds
            self.monitor_resources
        )
        
        # Initialize ROS publishers and subscribers
        self.create_subscriptions()
        self.create_publishers()
        
        # Create timers
        self.create_timers()
        
        # Model save timer
        self.model_save_timer = self.create_timer(
            60.0,  # Check if model needs saving every minute
            self.check_model_save
        )
        
        # Initialize Qt Application
        self.app = QApplication(sys.argv)
        
        # Create UI
        self.ui = FaceRecognitionUI(self, self.face_memory)
        self.ui.show()
        
        # ROS thread for spinning
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Connection check timer
        self.connection_timer = self.create_timer(
            1.0,  # Check every second
            self.check_connections
        )
        
        # Log startup
        self.get_logger().info('Face recognition node initialized and waiting for camera data')
        
        # Start Qt application main loop
        self.app.exec()
    
    def shutdown_hook(self):
        """Handle shutdown gracefully"""
        self.get_logger().info('Shutting down face recognition node...')
        
        # Save model before shutdown
        try:
            if hasattr(self, 'face_memory'):
                self.face_memory.save_model(force=True)
                self.get_logger().info('Face model saved during shutdown')
        except Exception as e:
            self.get_logger().error(f'Error saving face model during shutdown: {e}')
        
        # Close UI
        if hasattr(self, 'ui') and self.ui:
            self.ui.close()
        
        # Quit application
        if hasattr(self, 'app') and self.app:
            self.app.quit()
    
    def create_subscriptions(self):
        """Create ROS subscriptions"""
        # Camera frame subscription with quality of service settings
        qos = QoSProfile(depth=10)
        self.frame_subscription = self.create_subscription(
            Image,
            'camera_frame',  # Topic from camera_node
            self.frame_callback,
            qos
        )
        self.get_logger().info('Subscribed to camera_frame topic')
        
        # Face detection subscription
        self.face_subscription = self.create_subscription(
            String,
            'face_detection_data',  # Topic from camera_node
            self.face_data_callback,
            10
        )
        self.get_logger().info('Subscribed to face_detection_data topic')
        
        # Face images subscription (new)
        self.face_image_subscription = self.create_subscription(
            Image,
            'face_images',  # Face images from camera_node
            self.face_image_callback,
            10
        )
        self.get_logger().info('Subscribed to face_images topic')
    
    def create_publishers(self):
        """Create ROS publishers"""
        # Face recognition results publisher
        self.recognition_publisher = self.create_publisher(
            String,
            'face_recognition_data',
            10
        )
    
    def create_timers(self):
        """Create ROS timers"""
        # Timer for processing recognition
        self.recognition_timer = self.create_timer(
            0.2,  # 5 Hz processing rate
            self.process_recognition
        )
    
    def check_connections(self):
        """Check if we're receiving data from camera and face detection"""
        now = time.time()
        
        # Check camera frame connection
        if now - self.connection_status['last_frame_time'] < 2.0:
            if not self.connection_status['camera_frame']:
                self.get_logger().info('Camera frame connection established')
                self.connection_status['camera_frame'] = True
                if self.ui:
                    self.ui.add_log_message('Camera frame connection established')
        else:
            if self.connection_status['camera_frame']:
                self.get_logger().warn('Camera frame connection lost')
                self.connection_status['camera_frame'] = False
                if self.ui:
                    self.ui.add_log_message('Camera frame connection lost')
        
        # Check face detection connection
        if now - self.connection_status['last_face_data_time'] < 2.0:
            if not self.connection_status['face_detection']:
                self.get_logger().info('Face detection connection established')
                self.connection_status['face_detection'] = True
                if self.ui:
                    self.ui.add_log_message('Face detection connection established')
        else:
            if self.connection_status['face_detection']:
                self.get_logger().warn('Face detection connection lost')
                self.connection_status['face_detection'] = False
                if self.ui:
                    self.ui.add_log_message('Face detection connection lost')
        
        # Check face images connection
        if now - self.connection_status['last_face_image_time'] < 2.0:
            if not self.connection_status['face_images']:
                self.get_logger().info('Face images connection established')
                self.connection_status['face_images'] = True
                if self.ui:
                    self.ui.add_log_message('Face images connection established')
        else:
            if self.connection_status['face_images']:
                self.get_logger().warn('Face images connection lost')
                self.connection_status['face_images'] = False
                if self.ui:
                    self.ui.add_log_message('Face images connection lost')
    
    def frame_callback(self, msg):
        """Process incoming camera frames with rate limiting"""
        try:
            # Update connection status
            self.connection_status['last_frame_time'] = time.time()
            self.connection_status['frames_received'] += 1
            
            # Rate limiting - only process every Nth frame
            self.frame_counter += 1
            if self.frame_counter % self.process_every_n_frames != 0:
                return
                
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Downscale large images to save memory
            h, w = frame.shape[:2]
            scale_factor = min(1.0, 640 / max(w, h))  # Limit to 640px in largest dimension
            if scale_factor < 1.0:
                frame = cv2.resize(frame, (int(w * scale_factor), int(h * scale_factor)))
                
            self.latest_frame = frame
            self.has_new_data = True
            
            # Log periodically (every 100 frames)
            if self.connection_status['frames_received'] % 100 == 0:
                self.get_logger().debug(f'Received {self.connection_status["frames_received"]} camera frames')
        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {e}')
    
    def face_data_callback(self, msg):
        """Process incoming face detection data"""
        try:
            # Update connection status
            self.connection_status['last_face_data_time'] = time.time()
            self.connection_status['face_data_received'] += 1
            
            # Parse face data JSON
            face_data = json.loads(msg.data)
            
            # Convert to FaceData objects
            faces = [FaceData.from_dict(face) for face in face_data.get('faces', [])]
            
            if faces:
                self.latest_faces = faces
                self.has_new_data = True
                self.get_logger().debug(f'Received face data with {len(faces)} faces')
            else:
                self.get_logger().debug('Received face data with no faces detected')
        except Exception as e:
            self.get_logger().error(f'Error processing face data: {e}')
    
    def face_image_callback(self, msg):
        """Process incoming face image"""
        try:
            # Update connection status
            self.connection_status['last_face_image_time'] = time.time()
            self.connection_status['face_images_received'] += 1
            
            # Convert ROS Image message to OpenCV image
            face_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Get face ID from header frame_id field (e.g., "face_0")
            face_id = msg.header.frame_id
            timestamp = time.time()
            
            if face_img is not None:
                # Save the face image to a temporary file for recognition
                temp_file = os.path.join(self.face_memory.faces_dir, f'temp_face_{face_id}.jpg')
                cv2.imwrite(temp_file, face_img)
                
                # Process this face image directly with the face recognition system
                self.process_face_image(face_img, temp_file, face_id, timestamp)
                
                # Log occasionally
                if self.connection_status['face_images_received'] % 30 == 0:
                    self.get_logger().debug(f"Received {self.connection_status['face_images_received']} face images")
                
        except Exception as e:
            self.get_logger().error(f'Error processing face image: {e}')
    
    def process_face_image(self, face_img, face_image_path, face_id, timestamp):
        """Process a single face image for recognition with memory optimization"""
        if self.processing:
            return
            
        self.processing = True
        
        try:
            # Check if memory usage is too high
            if hasattr(self.face_memory, 'check_memory_usage'):
                memory_usage = self.face_memory.check_memory_usage()
                if memory_usage > self.face_memory.max_memory_usage_mb:
                    self.get_logger().warn(f'Memory usage too high ({memory_usage:.1f} MB). Skipping face processing.')
                    return
            
            # Resize image to save memory if needed
            h, w = face_img.shape[:2]
            if h > 200 or w > 200:
                face_img = cv2.resize(face_img, (min(w, 200), min(h, 200)))
                cv2.imwrite(face_image_path, face_img)
                
            # Set the image for recognition
            self.face_memory.rb.setImageToCopy(face_image_path)
            
            # Perform recognition
            recognition_start = time.time()
            self.face_memory.rb.num_recognitions += 1
            self.face_memory.stats['total_recognitions'] += 1
            recog_results = self.face_memory.rb.recognisePerson()
            recognition_time = time.time() - recognition_start
            
            # Store a downsampled version of the face to save memory
            face_img_small = cv2.resize(face_img, (100, 100))
            
            # Get the recognition results
            identity = self.face_memory.rb.identity_est
            confidence = self.face_memory.rb.quality_estimate
            is_new_face = False
            recognized_id = "0"  # Default to unknown face ID
            
            # Create face data with minimal info for display
            h, w = face_img.shape[:2]
            face_data = FaceData(
                x1=0, 
                y1=0, 
                x2=w, 
                y2=h, 
                center_x=w//2, 
                center_y=h//2, 
                confidence=confidence
            )
            
            # Process recognition outcome
            if identity and identity != "0":  # Known face with good confidence
                recognized_id = identity
                
                # Every time we recognize a face, update the model to improve recognition
                self.face_memory.rb.confirmPersonIdentity(recognized_id)
                
                # Add to training set for continuous improvement - but limit frequency
                if self.face_memory.stats['total_recognitions'] % 30 == 0:  # Only add every 30th recognition
                    training_complete = self.face_memory.add_training_face(recognized_id, face_img_small)
                    if training_complete:
                        self.get_logger().info(f"Completed training for face ID: {recognized_id} with {self.face_memory.training_count_target} images")
                        self.ui.add_log_message(f"Completed training for face ID: {recognized_id}")
                
                self.get_logger().info(f"Recognized existing face - ID: {recognized_id}, Confidence: {confidence:.2f}")
            elif confidence > 0.3:  # Face with medium confidence
                # Use the existing ID but don't create a new face
                self.get_logger().info(f"Face detected with medium confidence: {confidence:.2f}")
                return  # Skip further processing
            else:  # New face or no face detected
                # Check if a face was actually detected
                if hasattr(self.face_memory.rb, 'is_face_detected') and not self.face_memory.rb.is_face_detected:
                    self.get_logger().info("No face detected in the image")
                    return  # Skip further processing if no face detected
                    
                # Only add new faces if we have room and quality is acceptable
                face_quality = self.face_memory._calculate_face_quality(face_img_small)
                if len(self.face_memory.recognized_faces) < self.face_memory.max_faces_to_track and face_quality > 30:
                    # Assign a new ID
                    recognized_id = str(self.face_memory.next_id)
                    self.face_memory.next_id += 1
                    is_new_face = True
                    self.face_memory.stats['new_faces_learned'] += 1
                    
                    # Create a person entry with just an ID
                    person = [recognized_id, "", "", 0, 0]
                    self.face_memory.rb.addPersonToBN(person)
                    
                    # Now update the database with the face
                    self.face_memory.rb.isRegistered = False
                    self.face_memory.rb.identity_est = recognized_id
                    self.face_memory.rb.confirmPersonIdentity(recognized_id)
                    
                    # Start collecting training images
                    self.face_memory.add_training_face(recognized_id, face_img_small)
                    
                    self.get_logger().info(f"Learned new face - ID: {recognized_id}")
                    self.ui.add_log_message(f"Started training new face - ID: {recognized_id}")
                else:
                    self.get_logger().info(f"Skipping new face creation - low quality ({face_quality}) or too many faces")
                    return  # Skip further processing
            
            # Only create recognized_faces entry if we have a valid ID
            if recognized_id != "0":
                # Store the recognition result with debug info
                recognized_faces = {
                    recognized_id: {
                        'face_data': face_data,
                        'confidence': confidence,
                        'face_image': face_img_small,  # Use smaller face image
                        'is_new': is_new_face,
                        'recognition_time': recognition_time,
                        'face_size': f"{face_img_small.shape[1]}x{face_img_small.shape[0]}",
                        'face_quality': self.face_memory._calculate_face_quality(face_img_small),
                        'learning_status': "Learned (New)" if is_new_face else "Recognized (Updated)",
                        'last_updated': timestamp
                    }
                }
                
                # Update the recognized faces dictionary
                self.face_memory.recognized_faces.update(recognized_faces)
                
                # Manage the number of faces we're tracking
                self.face_memory.manage_recognized_faces()
                
                # Check if it's time to save the model
                current_time = time.time()
                if current_time - self.face_memory.last_model_save > self.face_memory.model_save_interval:
                    self.face_memory.save_model()
                
                # Publish recognition results
                self.publish_recognition_results(self.face_memory.recognized_faces)
            
        except Exception as e:
            self.get_logger().error(f'Error in face recognition processing: {e}', exc_info=True)
        finally:
            self.processing = False
    
    def process_recognition(self):
        """
        Legacy method - no longer needed since we now process faces directly
        in the face_image_callback
        """
        pass
    
    def publish_recognition_results(self, recognized_faces):
        """Publish face recognition results"""
        try:
            # Create a message with recognized face IDs and positions
            result_data = {
                'timestamp': time.time(),
                'faces': []
            }
            
            for face_id, face_info in recognized_faces.items():
                face_data = face_info['face_data']
                
                result_data['faces'].append({
                    'id': face_id,
                    'confidence': face_info['confidence'],
                    'is_new': face_info['is_new'],
                    'position': {
                        'x1': face_data.x1,
                        'y1': face_data.y1,
                        'x2': face_data.x2,
                        'y2': face_data.y2,
                        'center_x': face_data.center_x,
                        'center_y': face_data.center_y
                    }
                })
            
            # Publish as JSON string
            msg = String()
            msg.data = json.dumps(result_data)
            self.recognition_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing recognition results: {e}')
    
    def ros_spin(self):
        """ROS spin in separate thread to keep Qt happy"""
        rclpy.spin(self)
    
    def check_model_save(self):
        """Check if it's time to save the model"""
        try:
            self.face_memory.save_model()
            self.get_logger().debug('Model save check completed')
        except Exception as e:
            self.get_logger().error(f'Error in model save check: {e}')
    
    def run_garbage_collection(self):
        """Explicitly run Python garbage collection"""
        try:
            gc.collect()
            self.get_logger().debug('Garbage collection completed')
        except Exception as e:
            self.get_logger().error(f'Error during garbage collection: {e}')
    
    def monitor_resources(self):
        """Monitor system resources and adjust behavior as needed"""
        try:
            # Check memory usage
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            
            # Check CPU usage
            cpu_percent = process.cpu_percent(interval=0.1)
            
            # Log resource usage
            self.get_logger().debug(f'Memory usage: {memory_mb:.1f} MB, CPU: {cpu_percent:.1f}%')
            
            # Adjust processing rate based on resource usage
            if memory_mb > 800:  # If memory usage is very high
                self.process_every_n_frames = 10  # Process every 10th frame
                self.get_logger().warn(f'High memory usage ({memory_mb:.1f} MB). Reducing processing rate.')
            elif memory_mb > 500:  # If memory usage is moderate
                self.process_every_n_frames = 5  # Process every 5th frame
            else:
                self.process_every_n_frames = 3  # Normal rate
                
        except Exception as e:
            self.get_logger().error(f'Error monitoring resources: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = FaceRecognitionNode()
    
    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main() 