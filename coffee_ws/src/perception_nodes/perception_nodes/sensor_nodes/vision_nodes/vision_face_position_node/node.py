#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from coffee_buddy_msgs.msg import Face, Faces

from shared_configs import (
    VISION_TOPIC,
    FACE_POSITION_TOPIC,
    VISION_FACE_POSITION_NODE
)

class VisionFacePositionNode(Node):
    def __init__(self):
        super().__init__(VISION_FACE_POSITION_NODE)
        self.get_logger().info("Vision face position node initialized")
        
        # MediaPipe Face Mesh setup
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=10,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            refine_landmarks=True
        )
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Setup threading lock for thread safety
        self.lock = threading.Lock()
        
        # Create a publisher for the faces message
        self.faces_publisher = self.create_publisher(
            Faces, 
            FACE_POSITION_TOPIC, 
            10
        )
        
        # Create a subscriber for the camera frames
        self.camera_subscriber = self.create_subscription(
            Image,
            VISION_TOPIC,
            self.camera_callback,
            10
        )
        
        # Processing thread
        self.processing_thread = None
        self.running = True
        
        self.get_logger().info("Vision face position node ready")

    def camera_callback(self, msg):
        """Callback function for receiving camera frames"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Process frame in a separate thread to avoid blocking ROS callbacks
            if self.processing_thread is None or not self.processing_thread.is_alive():
                self.processing_thread = threading.Thread(
                    target=self.process_frame,
                    args=(cv_image,)
                )
                self.processing_thread.daemon = True
                self.processing_thread.start()
                
        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {e}")
    
    def process_frame(self, image):
        """Process camera frame to detect faces and estimate head pose"""
        try:
            with self.lock:
                # Convert the image to RGB for MediaPipe
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                # For better performance
                image_rgb.flags.writeable = False
                
                # Process the image with MediaPipe
                results = self.face_mesh.process(image_rgb)
                
                # For later drawing
                image_rgb.flags.writeable = True
                
                # If no faces are detected, return
                if not results.multi_face_landmarks:
                    return
                
                # Get image dimensions
                img_h, img_w, _ = image.shape
                
                # List to store Face messages
                face_msgs = []
                face_distances = []
                
                # Process each detected face
                for face_landmarks in results.multi_face_landmarks:
                    # Create a Face message
                    face_msg = Face()
                    
                    # Initialize face_landmarks with 468 Point objects
                    for i in range(468):
                        face_msg.face_landmarks.append(Point())
                    
                    # Populate face landmarks
                    for i, landmark in enumerate(face_landmarks.landmark):
                        if i < 468:  # Ensure we don't try to access beyond the array size
                            point = face_msg.face_landmarks[i]
                            point.x = landmark.x * img_w
                            point.y = landmark.y * img_h
                            point.z = landmark.z
                    
                    # Calculate head pose estimation
                    face_rotation, direction, distance = self.calculate_head_pose(face_landmarks, img_w, img_h)
                    
                    # Set face rotation
                    face_msg.face_rotation = face_rotation
                    face_msg.direction = direction
                    
                    # Add face to list
                    face_msgs.append(face_msg)
                    face_distances.append(distance)
                
                # Find the closest face (smallest z distance)
                if face_distances:
                    focused_face_index = face_distances.index(min(face_distances))
                    
                    # Create the Faces message
                    faces_msg = Faces()
                    faces_msg.faces = face_msgs
                    faces_msg.focused_face = focused_face_index
                    
                    # Publish the message
                    self.faces_publisher.publish(faces_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
    
    def calculate_head_pose(self, face_landmarks, img_w, img_h):
        """Calculate the head pose (rotation) from facial landmarks"""
        # Similar to the implementation in HeadPoseEstimation.py
        face_3d = []
        face_2d = []
        
        # Extract specific landmarks for pose estimation
        for idx, lm in enumerate(face_landmarks.landmark):
            if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:                
                x, y = lm.x * img_w, lm.y * img_h
                
                # Get the 2D coordinates
                face_2d.append([x, y])
                
                # Get the 3D coordinates
                face_3d.append([x, y, lm.z * 3000])
        
        # Convert to NumPy arrays
        face_2d = np.array(face_2d, dtype=np.float64)
        face_3d = np.array(face_3d, dtype=np.float64)
        
        # Camera matrix
        focal_length = 1 * img_w
        cam_matrix = np.array([
            [focal_length, 0, img_h / 2],
            [0, focal_length, img_w / 2],
            [0, 0, 1]
        ])
        
        # Distortion parameters
        dist_matrix = np.zeros((4, 1), dtype=np.float64)
        
        # Solve PnP to get rotation and translation vectors
        success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)
        
        # Get rotational matrix
        rmat, _ = cv2.Rodrigues(rot_vec)
        
        # Get angles
        angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
        
        # Get rotation in degrees
        x = angles[0] * 360
        y = angles[1] * 360
        z = angles[2] * 360
        
        # Determine direction based on angles
        if y < -10:
            direction = "Looking Left"
        elif y > 10:
            direction = "Looking Right"
        elif x < -10:
            direction = "Looking Down"
        elif x > 10:
            direction = "Looking Up"
        else:
            direction = "Forward"
        
        # Create Point message for rotation
        rotation = Point()
        rotation.x = x
        rotation.y = y
        rotation.z = z
        
        # Calculate distance - use the z component of translation vector as indicator
        # of distance from camera (depth)
        distance = float(trans_vec[2][0])
        
        return rotation, direction, distance
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.running = False
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=1.0)
        self.face_mesh.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    vision_face_position_node = VisionFacePositionNode()
    rclpy.spin(vision_face_position_node)
    vision_face_position_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()