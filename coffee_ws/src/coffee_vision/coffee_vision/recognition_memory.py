#!/usr/bin/env python3

import os
import numpy as np
import cv2
import time
import json
import pickle
import threading
from datetime import datetime
import logging
import hashlib

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("recognition_memory")

# Try to import cv2.face, but provide a fallback if not available
try:
    from cv2 import face as cv2_face
    FACE_RECOGNITION_AVAILABLE = True
    logger.info("OpenCV face recognition module available")
except ImportError:
    FACE_RECOGNITION_AVAILABLE = False
    logger.warning("OpenCV face module not available. Using basic recognition instead.")

try:
    import dlib
    DLIB_AVAILABLE = True
    logger.info("dlib available for face detection")
except ImportError:
    DLIB_AVAILABLE = False
    logger.warning("dlib not available. Using OpenCV for face detection.")

class RecogniserBN:
    """
    Simplified version of RecognitionMemory's RecogniserBN class
    using OpenCV's face recognition instead of the original implementation.
    """
    
    def __init__(self):
        # Face recognition parameters
        self.face_recognizer = None
        self.face_detector = None
        self.face_db = {}  # id -> face encoding
        self.init_face_recognition()
        
        # Recognition data
        self.identity_est = ""
        self.quality_estimate = 0.0
        self.num_recognitions = 0
        self.isRegistered = True
        
        # File paths
        self.recog_folder = ""
        self.image_save_dir = "faces/"
        self.db_file = "db.csv"
        self.face_db_file = "faceDB"
        self.image_to_copy = None
        
        # People database
        self.people = []
        
        # Settings
        self.isMemoryRobot = False
        self.isDBinCSV = True
        self.isMultipleRecognitions = False
        
        # Performance metrics
        self.recognition_time = 0.0
        
        # Face recognition thresholds - adjusted for better recognition
        self.recognition_threshold = 0.3  # Confidence threshold for recognition - lowered to recognize more faces
        self.new_face_threshold = 0.15    # Threshold to create a new face ID (must be below this)
        self.min_face_size = (60, 60)     # Minimum face size to detect
        
        # Learning parameters
        self.min_samples_for_learning = 5   # Minimum number of samples to start robust learning
        self.max_samples_per_identity = 100 # Maximum samples to store per identity
        self.face_counts = {}               # Count of how many times each face has been recognized
    
    def init_face_recognition(self):
        """Initialize OpenCV face recognition"""
        # First initialize face detector
        if DLIB_AVAILABLE:
            # Use dlib's face detector (better quality)
            self.face_detector = dlib.get_frontal_face_detector()
            try:
                # Try to load dlib's shape predictor for face landmarks
                model_path = os.path.join(os.path.dirname(__file__), 'shape_predictor_68_face_landmarks.dat')
                if os.path.exists(model_path):
                    self.shape_predictor = dlib.shape_predictor(model_path)
                    logger.info(f"Loaded dlib shape predictor from {model_path}")
                else:
                    logger.warning(f"dlib shape predictor model not found at {model_path}")
                    self.shape_predictor = None
            except Exception as e:
                logger.error(f"Error loading dlib shape predictor: {e}")
                self.shape_predictor = None
        else:
            # Use OpenCV's face detector as fallback
            try:
                self.face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
                logger.info("Using OpenCV Haar Cascade face detector")
            except Exception as e:
                logger.error(f"Error initializing OpenCV face detector: {e}")
                
        # Then initialize face recognition
        if FACE_RECOGNITION_AVAILABLE:
            try:
                # LBPH is more robust for different lighting conditions
                self.face_recognizer = cv2_face.LBPHFaceRecognizer_create(
                    radius=1,          # Radius for local binary pattern
                    neighbors=8,       # Number of neighbors
                    grid_x=8,          # Grid size X
                    grid_y=8,          # Grid size Y
                    threshold=100.0    # Default threshold (we'll adjust this)
                )
                logger.info("Using OpenCV LBPH face recognizer")
                
                # Try to load Eigenfaces and Fisherfaces as backups
                try:
                    self.eigen_recognizer = cv2_face.EigenFaceRecognizer_create()
                    self.fisher_recognizer = cv2_face.FisherFaceRecognizer_create()
                    logger.info("Loaded additional face recognizers")
                    self.use_multimodel = True
                except:
                    self.use_multimodel = False
            except Exception as e:
                logger.error(f"Error initializing face recognizer: {e}")
                self.fallback_recognizer_init()
        else:
            # Create a simple histogram-based recognizer as fallback
            self.fallback_recognizer_init()
            logger.info("Using fallback histogram-based recognizer")
    
    def fallback_recognizer_init(self):
        """Initialize a simple histogram-based face recognizer as fallback"""
        # We'll use a simple histogram comparison as fallback
        self.face_recognizer = SimpleFaceRecognizer()
        self.use_multimodel = False
    
    def setFilePaths(self, recog_folder):
        """Set paths for database and image storage"""
        self.recog_folder = recog_folder
        self.image_save_dir = os.path.join(recog_folder, "faces/")
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        self.db_file = os.path.join(recog_folder, "db.csv")
        self.face_db_file = os.path.join(recog_folder, "faceDB")
    
    def setImageToCopy(self, image_path):
        """Set the image to use for recognition"""
        self.image_to_copy = image_path
    
    def setSessionConstant(self, isMemoryRobot=True, isDBinCSV=True, 
                          isMultipleRecognitions=False, defNumMultRecog=3,
                          isSaveRecogFiles=True, isSpeak=True):
        """Set session constants"""
        self.isMemoryRobot = isMemoryRobot
        self.isDBinCSV = isDBinCSV
        self.isMultipleRecognitions = isMultipleRecognitions
        
    def setSilentMode(self):
        """Set silent mode (no speaking)"""
        pass  # We don't need to do anything here
    
    def loadDB(self, db_file=None):
        """Load the face database"""
        if db_file is None:
            db_file = self.db_file
        
        # Load people database from CSV
        if os.path.exists(db_file):
            try:
                logger.info(f"Loading database from {db_file}")
                # Load people data (simple format: ID,Name,Gender,Age,Height)
                with open(db_file, 'r') as f:
                    lines = f.readlines()
                    for line in lines[1:]:  # Skip header
                        parts = line.strip().split(',')
                        if len(parts) >= 5:
                            person_id = parts[0]
                            name = parts[1]
                            gender = parts[2]
                            try:
                                age = int(parts[3])
                                height = float(parts[4])
                            except ValueError:
                                age = 0
                                height = 0
                            self.people.append([person_id, name, gender, age, height])
                logger.info(f"Loaded {len(self.people)} people from database")
            except Exception as e:
                logger.error(f"Error loading DB from CSV: {e}")
        
        # Load face recognition database
        self.loadFaceRecognitionDB()
    
    def loadFaceRecognitionDB(self):
        """Load face recognition database"""
        if os.path.exists(self.face_db_file):
            try:
                logger.info(f"Loading face database from {self.face_db_file}")
                with open(self.face_db_file, 'rb') as f:
                    self.face_db = pickle.load(f)
                    
                # If we have labeled faces, train the recognizer
                if self.face_db:
                    self._train_recognizer_from_db()
                    logger.info(f"Face recognition model trained with {sum(len(faces) for faces in self.face_db.values())} faces")
            except Exception as e:
                logger.error(f"Error loading face recognition DB: {e}")
                # Initialize empty database
                self.face_db = {}
        else:
            logger.info(f"No face database found at {self.face_db_file}, initializing empty DB")
            self.face_db = {}
    
    def _train_recognizer_from_db(self):
        """Train face recognizer from the database"""
        if not self.face_db:
            return False
            
        try:
            faces = []
            labels = []
            label_map = {}
            next_label = 0
            
            # Filter out identities with too few samples
            filtered_face_db = {}
            for person_id, encodings in self.face_db.items():
                if len(encodings) >= self.min_samples_for_learning:
                    filtered_face_db[person_id] = encodings
                else:
                    logger.debug(f"Skipping person {person_id} for training - only {len(encodings)} samples")
            
            # If we don't have any identities with enough samples, use all samples
            if not filtered_face_db and self.face_db:
                logger.info("No identities have sufficient samples, using all available faces")
                filtered_face_db = self.face_db
            
            for person_id, encodings in filtered_face_db.items():
                if encodings:
                    label_map[next_label] = person_id
                    for encoding in encodings:
                        faces.append(encoding)
                        labels.append(next_label)
                    next_label += 1
            
            if faces and len(faces) >= 2:  # Need at least 2 faces to train
                # Train the main recognizer
                logger.info(f"Training face recognizer with {len(faces)} faces from {len(filtered_face_db)} people")
                self.face_recognizer.train(faces, np.array(labels))
                self.label_map = label_map
                
                # Train backup recognizers if available
                if hasattr(self, 'use_multimodel') and self.use_multimodel and len(set(labels)) > 1:
                    try:
                        # Resize faces to 100x100 for Eigenfaces and Fisherfaces
                        resized_faces = [cv2.resize(face, (100, 100)) for face in faces]
                        self.eigen_recognizer.train(resized_faces, np.array(labels))
                        self.fisher_recognizer.train(resized_faces, np.array(labels))
                    except Exception as e:
                        logger.error(f"Error training backup recognizers: {e}")
                
                return True
        except Exception as e:
            logger.error(f"Error training face recognizer: {e}")
            
        return False
    
    def saveFaceRecognitionDB(self):
        """Save face recognition database"""
        try:
            with open(self.face_db_file, 'wb') as f:
                pickle.dump(self.face_db, f)
            logger.info(f"Saved face database to {self.face_db_file}")
            return True
        except Exception as e:
            logger.error(f"Error saving face recognition DB: {e}")
            return False
    
    def resetFaceDetectionDB(self):
        """Reset face detection database"""
        self.face_db = {}
        self.saveFaceRecognitionDB()
        logger.info("Face detection database reset")
    
    def setFaceDetectionDB(self, facedb=None):
        """Set face detection database file"""
        if facedb:
            self.face_db_file = os.path.join(self.recog_folder, facedb)
            logger.info(f"Set face detection database to {self.face_db_file}")
    
    def useFaceDetectionDB(self, facedb=None):
        """Use existing face detection database"""
        if facedb:
            self.face_db_file = os.path.join(self.recog_folder, facedb)
            self.loadFaceRecognitionDB()
    
    def detectFaces(self, image):
        """Detect faces in image using the configured detector"""
        if image is None:
            return []
            
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        # Apply histogram equalization to improve detection in various lighting
        gray = cv2.equalizeHist(gray)
        
        faces = []
        
        if DLIB_AVAILABLE and isinstance(self.face_detector, dlib.fhog_object_detector):
            # Use dlib's detector
            try:
                # Detect faces
                dlib_faces = self.face_detector(gray, 1)
                
                for rect in dlib_faces:
                    # Convert dlib rectangle to OpenCV format
                    x, y = rect.left(), rect.top()
                    w, h = rect.width(), rect.height()
                    
                    # Skip faces smaller than minimum size
                    if w < self.min_face_size[0] or h < self.min_face_size[1]:
                        continue
                        
                    # Extract face ROI
                    face_roi = gray[y:y+h, x:x+w]
                    
                    # Use shape predictor if available for better face alignment
                    aligned_face = None
                    if self.shape_predictor:
                        try:
                            # Get landmarks
                            shape = self.shape_predictor(gray, rect)
                            # Convert to numpy array
                            landmarks = np.array([[p.x, p.y] for p in shape.parts()])
                            # Use landmarks for alignment (simple version)
                            aligned_face = self._align_face(gray, landmarks)
                        except Exception as e:
                            logger.warning(f"Error in face alignment: {e}")
                    
                    # If alignment failed, use the original face
                    if aligned_face is None or aligned_face.size == 0:
                        # Resize to standard size
                        face_roi = cv2.resize(face_roi, (100, 100))
                    else:
                        face_roi = aligned_face
                        
                    # Store face
                    faces.append(face_roi)
            except Exception as e:
                logger.error(f"Error in dlib face detection: {e}")
                
        elif isinstance(self.face_detector, cv2.CascadeClassifier):
            # Use OpenCV's cascade classifier
            try:
                # Detect faces
                opencv_faces = self.face_detector.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=self.min_face_size
                )
                
                for (x, y, w, h) in opencv_faces:
                    # Extract face ROI
                    face_roi = gray[y:y+h, x:x+w]
                    # Resize to standard size
                    face_roi = cv2.resize(face_roi, (100, 100))
                    # Store face
                    faces.append(face_roi)
            except Exception as e:
                logger.error(f"Error in OpenCV face detection: {e}")
                
        return faces
    
    def _align_face(self, image, landmarks, desired_size=100):
        """Align face using facial landmarks"""
        try:
            # Use eye centers for alignment
            left_eye = np.mean(landmarks[36:42], axis=0).astype(int)
            right_eye = np.mean(landmarks[42:48], axis=0).astype(int)
            
            # Calculate angle
            dx = right_eye[0] - left_eye[0]
            dy = right_eye[1] - left_eye[1]
            angle = np.degrees(np.arctan2(dy, dx))
            
            # Get center of eyes
            eye_center = ((left_eye[0] + right_eye[0]) // 2, (left_eye[1] + right_eye[1]) // 2)
            
            # Get rotation matrix
            M = cv2.getRotationMatrix2D(eye_center, angle, 1)
            
            # Apply rotation
            h, w = image.shape[:2]
            aligned = cv2.warpAffine(image, M, (w, h), flags=cv2.INTER_CUBIC)
            
            # Calculate bounding box around face
            face_height = int(desired_size * 1.5)  # Allow more space for face height
            face_width = desired_size
            
            # Extract face region
            y1 = max(0, eye_center[1] - face_height // 3)
            y2 = min(h, y1 + face_height)
            x1 = max(0, eye_center[0] - face_width // 2)
            x2 = min(w, x1 + face_width)
            
            # Extract aligned face
            aligned_face = aligned[y1:y2, x1:x2]
            
            # Resize to desired size
            aligned_face = cv2.resize(aligned_face, (desired_size, desired_size))
            
            return aligned_face
        except Exception as e:
            logger.warning(f"Face alignment failed: {e}")
            return None
    
    def extractFace(self, image_path):
        """Extract face from image"""
        try:
            # Read image
            img = cv2.imread(image_path)
            if img is None:
                logger.error(f"Could not read image: {image_path}")
                return None
            
            # Detect faces
            faces = self.detectFaces(img)
            
            if not faces:
                logger.warning("No face detected in image")
                return None
            
            # Use the first face (should be the best one from detectFaces)
            return faces[0]
            
        except Exception as e:
            logger.error(f"Error extracting face: {e}")
            return None
    
    def recognisePerson(self):
        """Recognize person from image"""
        start_time = time.time()
        
        if not self.image_to_copy:
            logger.error("No image set for recognition")
            self.identity_est = "0"  # Unknown
            self.quality_estimate = 0.0
            return []
        
        # Extract face from image
        face = self.extractFace(self.image_to_copy)
        if face is None:
            logger.warning("No face detected in image")
            self.identity_est = "0"  # Unknown
            self.quality_estimate = 0.0
            return []
        
        # If no faces in database or recognizer not initialized, return unknown
        if not self.face_db or not hasattr(self, 'label_map'):
            logger.warning("No face database or label map")
            self.identity_est = "0"  # Unknown
            self.quality_estimate = 0.0
            return []
        
        try:
            # Get predictions from multiple models if available
            predictions = []
            
            # Main recognizer prediction
            label, confidence = self.face_recognizer.predict(face)
            predictions.append((label, self._convert_confidence(confidence)))
            
            # Backup recognizers if available
            if hasattr(self, 'use_multimodel') and self.use_multimodel:
                try:
                    # Resize for Eigenfaces and Fisherfaces
                    resized_face = cv2.resize(face, (100, 100))
                    
                    # Eigen recognizer
                    eigen_label, eigen_confidence = self.eigen_recognizer.predict(resized_face)
                    predictions.append((eigen_label, self._convert_confidence(eigen_confidence)))
                    
                    # Fisher recognizer 
                    fisher_label, fisher_confidence = self.fisher_recognizer.predict(resized_face)
                    predictions.append((fisher_label, self._convert_confidence(fisher_confidence)))
                except Exception as e:
                    logger.warning(f"Error using backup recognizers: {e}")
            
            # Combine predictions (take the most confident prediction)
            best_label, best_confidence = max(predictions, key=lambda x: x[1])
            
            # Map label to person ID
            if best_label in self.label_map and best_confidence >= self.recognition_threshold:
                # Good match - use existing person
                person_id = self.label_map[best_label]
                self.identity_est = person_id
                self.quality_estimate = best_confidence
                
                # Update face count for this identity
                if person_id not in self.face_counts:
                    self.face_counts[person_id] = 0
                self.face_counts[person_id] += 1
                
                logger.info(f"Recognized person: {person_id} with confidence {best_confidence:.2f}")
            elif best_confidence > self.new_face_threshold:
                # Medium confidence - not good enough for a definite match, 
                # but not low enough to create a new face
                # Return unknown but with the confidence value
                self.identity_est = "0"  # Unknown
                self.quality_estimate = best_confidence
                logger.info(f"Person not recognized with high confidence. Best confidence was {best_confidence:.2f}")
            else:
                # Low confidence - create a new face entry
                self.identity_est = "0"  # Unknown
                self.quality_estimate = 0.0
                logger.info(f"Person not recognized. Best confidence was {best_confidence:.2f}")
                
            # Calculate recognition time
            self.recognition_time = time.time() - start_time
            logger.debug(f"Recognition took {self.recognition_time:.3f} seconds")
            
        except Exception as e:
            logger.error(f"Error in face recognition: {e}")
            self.identity_est = "0"  # Unknown
            self.quality_estimate = 0.0
        
        return []  # Empty result list (we use identity_est and quality_estimate)
    
    def _convert_confidence(self, raw_confidence):
        """Convert raw confidence score to normalized confidence (0-1, higher is better)"""
        # For LBPH and other OpenCV recognizers, lower value means better match
        # Convert to our format (0-1, higher is better)
        return max(0, min(1, (100 - min(raw_confidence, 100)) / 100))
    
    def addPersonToBN(self, person):
        """Add a new person to the Bayesian Network"""
        # Add to people list if not already there
        person_id = person[0]
        exists = False
        for p in self.people:
            if p[0] == person_id:
                exists = True
                break
                
        if not exists:
            self.people.append(person)
            logger.info(f"Added person {person_id} to database")
            
            # Save to database
            self.saveDBToCSV(self.db_file, person)
        
        return True
    
    def saveDBToCSV(self, db_file, person=None):
        """Save database to CSV file"""
        try:
            # Create header if file doesn't exist
            if not os.path.exists(db_file) or os.path.getsize(db_file) == 0:
                with open(db_file, 'w') as f:
                    f.write("ID,Name,Gender,Age,Height\n")
            
            # Append new person if provided
            if person:
                with open(db_file, 'a') as f:
                    f.write(f"{person[0]},{person[1]},{person[2]},{person[3]},{person[4]}\n")
                logger.info(f"Saved person {person[0]} to database")
            
            return True
        except Exception as e:
            logger.error(f"Error saving DB to CSV: {e}")
            return False
    
    def confirmPersonIdentity(self, p_id=None, recog_results_from_file=None, isRobotLearning=True):
        """Confirm identity of person and add face to database"""
        if not self.image_to_copy:
            logger.error("No image set for identity confirmation")
            return False
        
        # Extract face
        face = self.extractFace(self.image_to_copy)
        if face is None:
            logger.error("No face detected for identity confirmation")
            return False
        
        try:
            # Add face to database
            person_id = p_id if p_id else self.identity_est
            logger.info(f"Confirming identity for person {person_id}")
            
            # Initialize entry if not present
            if person_id not in self.face_db:
                self.face_db[person_id] = []
                self.face_counts[person_id] = 0
            
            # Calculate face hash for storage
            face_hash = hashlib.md5(face.tobytes()).hexdigest()
            
            # Check if we should add this face sample
            should_add = True
            max_samples = self.max_samples_per_identity
            
            # Apply different sampling strategies based on how many samples we already have
            current_count = len(self.face_db[person_id])
            
            if current_count >= max_samples:
                # If we already have max samples, replace oldest one
                # This keeps the model fresh with recent faces while maintaining a maximum size
                self.face_db[person_id].pop(0)  # Remove oldest sample
                logger.debug(f"Replaced oldest sample for person {person_id} (max {max_samples} reached)")
            elif current_count > self.min_samples_for_learning:
                # For faces we've seen many times, sample less frequently to avoid bias
                sampling_factor = min(25, self.face_counts[person_id]) / 25.0
                # Random sampling that decreases as we get more samples
                import random
                should_add = random.random() > sampling_factor
                
                if not should_add:
                    logger.debug(f"Skipped adding sample for well-known person {person_id}")
            
            if should_add:
                # Add face encoding
                self.face_db[person_id].append(face)
                logger.debug(f"Added face sample #{len(self.face_db[person_id])} for person {person_id}")
                
                # Save face image with a unique name
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                face_filename = f"face_{person_id}_{timestamp}_{face_hash[:8]}.jpg"
                face_path = os.path.join(self.image_save_dir, face_filename)
                
                cv2.imwrite(face_path, face)
                logger.info(f"Saved face image to {face_path}")
            
            # Increment face count regardless of whether we added the sample
            self.face_counts[person_id] = self.face_counts.get(person_id, 0) + 1
            
            # Retrain recognizer with all faces if we added a sample or if we have few samples
            if should_add or current_count < self.min_samples_for_learning:
                self._train_recognizer_from_db()
            
            # Save database
            self.saveFaceRecognitionDB()
            
            return True
        except Exception as e:
            logger.error(f"Error confirming person identity: {e}")
            return False


class SimpleFaceRecognizer:
    """Simple histogram-based face recognizer as fallback when cv2.face is not available"""
    
    def __init__(self):
        self.faces = []
        self.labels = []
        self.trained = False
    
    def train(self, faces, labels):
        """Train the recognizer with face images"""
        self.faces = []
        for face in faces:
            # Calculate histogram as feature
            hist = cv2.calcHist([face], [0], None, [64], [0, 256])
            hist = cv2.normalize(hist, hist).flatten()
            self.faces.append(hist)
        
        self.labels = labels.tolist() if isinstance(labels, np.ndarray) else labels
        self.trained = True
        logger.info(f"SimpleFaceRecognizer trained with {len(faces)} faces")
    
    def predict(self, face):
        """Predict the label for a face"""
        if not self.trained or not self.faces:
            return 0, 0.0
        
        # Calculate histogram of input face
        hist = cv2.calcHist([face], [0], None, [64], [0, 256])
        hist = cv2.normalize(hist, hist).flatten()
        
        # Compare with all faces
        best_match = 0
        best_score = 0
        
        for i, stored_hist in enumerate(self.faces):
            # Use correlation as similarity measure
            score = cv2.compareHist(hist, stored_hist, cv2.HISTCMP_CORREL)
            if score > best_score:
                best_score = score
                best_match = i
        
        if best_score > 0:
            # Return label and confidence
            # Convert score from [0,1] to [0,100] and invert (lower is better in OpenCV)
            confidence = 100 * (1 - best_score)
            return self.labels[best_match], confidence
        else:
            return 0, 100.0 