Camera Node
===========

Overview
--------

The Camera Node is the primary perception system for the robot, handling video capture, face detection, and the initial analysis of visual data. It processes video frames in real-time to detect faces and extracts information such as position, size, and movement patterns, which forms the basis for the robot's reactive behaviors.

Classes
-------

CameraNode
~~~~~~~~~

The main ROS node responsible for camera operations:

- Manages video capture from connected cameras
- Processes frames for face detection
- Publishes face detection data for other nodes
- Handles camera parameter configuration
- Manages frame rate and processing load

FrameGrabber
~~~~~~~~~~~

Handles the acquisition of frames from camera devices:

- Manages camera connection and configuration
- Controls frame capture timing and buffers
- Provides thread-safe access to camera frames
- Handles camera switching and fallback mechanisms
- Supports various camera types and interfaces

FaceDetector
~~~~~~~~~~~

Performs face detection and feature extraction:

- Detects faces in camera frames using computer vision
- Extracts facial landmarks and features
- Calculates face position, size, and orientation
- Tracks faces across multiple frames
- Estimates face movement velocity and direction

ROS Interface
------------

Publishers:

- ``face_detection_data`` (String): Publishes detected face information in JSON format
- ``camera_frame`` (Image): Publishes the processed camera frame (when enabled)
- ``face_velocity`` (Vector3): Publishes the estimated velocity of detected faces
- ``camera_status`` (String): Publishes camera operational status

Parameters:

- ``camera_index`` (int): Index of the camera device to use (default: 0)
- ``resolution_width`` (int): Width of camera resolution (default: 640)
- ``resolution_height`` (int): Height of camera resolution (default: 480)
- ``frame_rate`` (float): Target frame rate for processing (default: 30.0)
- ``face_detection_frequency`` (float): How often to run face detection (default: 15.0)
- ``publish_images`` (bool): Whether to publish camera frames (default: false)
- ``detection_confidence`` (float): Minimum confidence for face detection (default: 0.5)

Key Features
-----------

- Real-time face detection and tracking
- Multi-face tracking capability
- Face position and velocity estimation
- Adaptive processing to maintain stable frame rate
- Configurable detection parameters
- Support for various camera hardware
- Fallback mechanisms for camera failures
- Debug visualization options for development 