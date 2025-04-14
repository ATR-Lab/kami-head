Eye Tracking Node
===============

Overview
--------

The Eye Tracking Node controls the robot's eye movements to create lifelike and responsive visual engagement. It works in conjunction with the Head Tracking Node to generate realistic eye behaviors, including saccades, blinking, pupil dilation, and responsive gaze patterns based on face tracking data.

Classes
-------

EyeTrackingNode
~~~~~~~~~~~~~~

The main ROS node class for eye tracking:

- Manages the eye movement system
- Subscribes to face tracking and head position data
- Coordinates eye movements with head position
- Controls eye state transitions (looking, blinking, etc.)
- Handles parameter updates and configurations

EyeController
~~~~~~~~~~~~

Core controller for eye movements:

- Implements different eye behavior patterns
- Handles saccade generation and timing
- Controls eye blinking and pupil dilation
- Calculates eye position based on face position and head angles
- Manages natural idle movements when no face is detected

EyeVisualization
~~~~~~~~~~~~~~

Visualization component for the eye tracking system:

- Renders eyes on display
- Animates eye movements, blinks, and pupil dilation
- Provides visual feedback of tracking status
- Adjusts appearance based on interaction state
- Supports customizable eye appearance parameters

ROS Interface
------------

Subscribers:

- ``face_detection`` (String): Receives face detection information
- ``face_velocity`` (Vector3): Receives velocity data of tracked faces
- ``motor_position`` (Float64MultiArray): Receives current head motor positions
- ``head_tracking_status`` (String): Receives tracking status from head tracking node

Publishers:

- ``eye_position`` (Point): Publishes current eye position
- ``eye_state`` (String): Publishes current state of the eyes (looking, blinking, etc.)
- ``attention_focus`` (Point): Publishes point of attention focus

Parameters:

- ``saccade_frequency`` (float): Frequency of small eye movements (default: 0.5 Hz)
- ``blink_rate`` (float): Average blinks per minute (default: 15)
- ``pupil_responsiveness`` (float): How quickly pupils dilate/contract (default: 0.3)
- ``gaze_randomness`` (float): Random factor in gaze direction (default: 0.2)
- ``use_head_compensation`` (bool): Compensate eye position for head movement (default: true)
- ``max_eye_angle`` (float): Maximum angle for eye movement (default: 20.0)
- ``idle_eye_movement`` (bool): Enable random movements when idle (default: true)

Key Features
-----------

- Realistic eye movements synchronized with head tracking
- Natural blinking patterns with adjustable frequency
- Pupil dilation based on lighting and attention
- Microsaccades for lifelike small movements
- Smooth transitions between different gaze targets
- Compensation for head movements in eye positioning
- Customizable eye behavior through parameters
- Visual feedback through eye display 