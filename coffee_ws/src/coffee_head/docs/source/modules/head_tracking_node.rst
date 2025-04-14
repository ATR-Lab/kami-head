Head Tracking Node
================

Overview
--------

The Head Tracking Node controls the physical movement of the robot's head based on face detection data. It processes face position information from the Camera Node and calculates appropriate motor commands to position the head servos, allowing the robot to follow and maintain engagement with detected faces. The node implements smooth motion control with configurable parameters for natural movement patterns.

Classes
-------

HeadTrackingNode
~~~~~~~~~~~~~~~

The primary ROS node for controlling head movements:

- Subscribes to face detection data from the Camera Node
- Calculates target head positions based on face locations
- Generates motor commands for the pan-tilt mechanism
- Maintains tracking state and engagement metrics
- Publishes tracking status and motor positions

HeadController
~~~~~~~~~~~~~

Responsible for movement calculations and motor control:

- Implements PID control for smooth position tracking
- Handles velocity ramping and motion smoothing
- Manages head movement limits and constraints
- Provides different tracking modes (smooth, responsive, etc.)
- Controls speed parameters for natural head movements

MotorInterface
~~~~~~~~~~~~~

Manages communication with the servo motors:

- Handles serial communication with motor controllers
- Formats and sends motor control commands
- Manages baud rate and communication settings
- Reads position feedback from motors
- Implements fault detection and recovery

ROS Interface
------------

Subscribers:

- ``face_detection_data`` (String): Receives face detection information
- ``tracking_mode`` (String): Receives tracking behavior commands
- ``manual_head_control`` (Vector3): Receives manual head position control signals

Publishers:

- ``head_position`` (Float64MultiArray): Publishes current head position
- ``motor_command`` (SetPosition): Publishes motor control commands
- ``tracking_status`` (String): Publishes the current tracking state
- ``head_velocity`` (Vector3): Publishes current head movement velocity

Parameters:

- ``update_rate_hz`` (float): Control loop update rate (default: 30.0)
- ``baud_rate`` (int): Baud rate for motor communication (default: 1000000)
- ``motor_ids`` (array): IDs of the motors used for head control
- ``max_speed`` (float): Maximum movement speed (default: 0.5)
- ``pid_p`` (float): Proportional gain for PID control (default: 1.0)
- ``pid_i`` (float): Integral gain for PID control (default: 0.1)
- ``pid_d`` (float): Derivative gain for PID control (default: 0.05)
- ``timeout`` (float): Tracking timeout in seconds (default: 3.0)
- ``smoothing_factor`` (float): Movement smoothing factor (default: 0.7)

Key Features
-----------

- Smooth face tracking with configurable parameters
- Multiple tracking behavior modes (responsive, relaxed, precise)
- PID-controlled movement for natural head motions
- Automatic recovery from lost tracking
- Configurable movement constraints and limits
- Manual override capability
- Adaptive tracking sensitivity based on face distance
- Configurable update rate and baud rate for performance tuning 