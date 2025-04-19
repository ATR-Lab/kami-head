#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Motion Recorder Node
Records and plays back motion sequences for Dynamixel servo motors
"""

import os
import json
import time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float32MultiArray, String, Bool
from std_srvs.srv import Trigger, SetBool

from motion_recorder.dynamixel_interface import DynamixelInterface

class MotionRecorder(Node):
    """
    ROS2 node for recording and playing back motion sequences
    for Dynamixel servo-controlled robot head
    """
    
    def __init__(self):
        super().__init__('motion_recorder')
        
        # Create callback groups for services to allow concurrent processing
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('sampling_rate', 50.0)  # Hz
        self.declare_parameter('motion_files_dir', os.path.expanduser('~/.ros/motion_files'))
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.motion_files_dir = self.get_parameter('motion_files_dir').value
        
        # Create motion files directory if it doesn't exist
        os.makedirs(self.motion_files_dir, exist_ok=True)
        
        # Initialize Dynamixel interface
        self.get_logger().info(f"Initializing Dynamixel interface on port {self.port}...")
        self.dxl = DynamixelInterface(self, port=self.port, baudrate=self.baudrate)
        
        # Motion data storage
        self.frames = []
        self.keyframes = []
        self.current_motion_name = "unnamed_motion"
        self.is_recording = False
        self.is_playing = False
        self.start_time = 0.0
        
        # Recording timer
        self.recording_timer = None
        self.playback_timer = None
        
        # Motor IDs (from dynamixel_interface)
        self.pan_id = self.dxl.pan_id
        self.tilt_id = self.dxl.tilt_id
        
        # Publishers
        self.position_pub = self.create_publisher(
            Float32MultiArray, 'head_position', 10)
        self.status_pub = self.create_publisher(
            String, 'motion_recorder/status', 10)
        
        # Services
        self.create_service(
            Trigger, 'motion_recorder/start_recording',
            self.start_recording_callback, callback_group=self.service_group)
        self.create_service(
            Trigger, 'motion_recorder/stop_recording',
            self.stop_recording_callback, callback_group=self.service_group)
        self.create_service(
            SetBool, 'motion_recorder/toggle_torque',
            self.toggle_torque_callback, callback_group=self.service_group)
        self.create_service(
            Trigger, 'motion_recorder/mark_keyframe',
            self.mark_keyframe_callback, callback_group=self.service_group)
        self.create_service(
            String, 'motion_recorder/save_motion',
            self.save_motion_callback, callback_group=self.service_group)
        self.create_service(
            String, 'motion_recorder/load_motion',
            self.load_motion_callback, callback_group=self.service_group)
        self.create_service(
            Trigger, 'motion_recorder/play_motion',
            self.play_motion_callback, callback_group=self.service_group)
        self.create_service(
            Trigger, 'motion_recorder/stop_playback',
            self.stop_playback_callback, callback_group=self.service_group)
        self.create_service(
            String, 'motion_recorder/list_motions',
            self.list_motions_callback, callback_group=self.service_group)
        
        # Status update timer (1Hz)
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=self.timer_group)
        
        self.get_logger().info("Motion recorder initialized and ready")
    
    def publish_status(self):
        """Publish current status information"""
        status = {
            "is_recording": self.is_recording,
            "is_playing": self.is_playing,
            "motion_name": self.current_motion_name,
            "frame_count": len(self.frames),
            "keyframe_count": len(self.keyframes),
            "duration": self.frames[-1]["timestamp"] if self.frames else 0.0
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def start_recording_callback(self, request, response):
        """Start recording motion frames"""
        if self.is_recording:
            response.success = False
            response.message = "Already recording"
            return response
        
        if self.is_playing:
            response.success = False
            response.message = "Cannot record while playing motion"
            return response
        
        # Turn off torque for manual positioning
        self.dxl.enable_torque(self.pan_id, False)
        self.dxl.enable_torque(self.tilt_id, False)
        
        # Turn on LEDs to indicate recording
        self.dxl.set_led(self.pan_id, True)
        self.dxl.set_led(self.tilt_id, True)
        
        # Reset recording data
        self.frames = []
        self.keyframes = []
        self.is_recording = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Create timer for recording at specified sampling rate
        period = 1.0 / self.sampling_rate
        self.recording_timer = self.create_timer(
            period, self.record_frame, callback_group=self.timer_group)
        
        self.get_logger().info(f"Started recording at {self.sampling_rate} Hz")
        response.success = True
        response.message = f"Started recording at {self.sampling_rate} Hz"
        return response
    
    def record_frame(self):
        """Record a single frame of motion data"""
        if not self.is_recording:
            return
        
        try:
            # Get current positions
            pan_pos = self.dxl.read_position(self.pan_id)
            tilt_pos = self.dxl.read_position(self.tilt_id)
            
            # Skip if position reading failed
            if pan_pos is None or tilt_pos is None:
                self.get_logger().warning("Failed to read position, skipping frame")
                return
            
            # Calculate timestamp
            current_time = self.get_clock().now().nanoseconds / 1e9
            timestamp = current_time - self.start_time
            
            # Create frame
            frame = {
                "timestamp": timestamp,
                "positions": [pan_pos, tilt_pos],
                "is_keyframe": False
            }
            
            # Store frame
            self.frames.append(frame)
            
            # Debug output every second
            if len(self.frames) % int(self.sampling_rate) == 0:
                self.get_logger().debug(
                    f"Recording: t={timestamp:.2f}s, pan={pan_pos:.2f}°, tilt={tilt_pos:.2f}°")
        
        except Exception as e:
            self.get_logger().error(f"Error recording frame: {e}")
    
    def stop_recording_callback(self, request, response):
        """Stop recording motion frames"""
        if not self.is_recording:
            response.success = False
            response.message = "Not recording"
            return response
        
        # Stop the recording timer
        if self.recording_timer:
            self.recording_timer.cancel()
            self.recording_timer = None
        
        # Turn off LEDs to indicate recording stopped
        self.dxl.set_led(self.pan_id, False)
        self.dxl.set_led(self.tilt_id, False)
        
        self.is_recording = False
        
        # Calculate duration
        duration = 0.0
        if self.frames:
            duration = self.frames[-1]["timestamp"]
        
        # Keep torque disabled to allow continued manual positioning
        frame_count = len(self.frames)
        keyframe_count = len(self.keyframes)
        
        self.get_logger().info(
            f"Stopped recording: {frame_count} frames, {keyframe_count} keyframes, {duration:.2f}s")
        
        response.success = True
        response.message = f"Stopped recording: {frame_count} frames, {duration:.2f}s"
        return response
    
    def toggle_torque_callback(self, request, response):
        """Toggle torque on/off for manual positioning"""
        enable = request.data
        
        if self.is_recording and enable:
            response.success = False
            response.message = "Cannot enable torque while recording"
            return response
        
        if self.is_playing and not enable:
            response.success = False
            response.message = "Cannot disable torque while playing motion"
            return response
        
        # Set torque for both motors
        success1 = self.dxl.enable_torque(self.pan_id, enable)
        success2 = self.dxl.enable_torque(self.tilt_id, enable)
        
        response.success = success1 and success2
        response.message = f"Torque {'enabled' if enable else 'disabled'}"
        return response
    
    def mark_keyframe_callback(self, request, response):
        """Mark current position as a keyframe"""
        if not self.is_recording:
            response.success = False
            response.message = "Not recording"
            return response
        
        # Get last recorded frame
        if not self.frames:
            response.success = False
            response.message = "No frames recorded yet"
            return response
        
        # Mark the last frame as a keyframe
        last_frame = self.frames[-1]
        last_frame["is_keyframe"] = True
        
        # Also add to keyframes list
        keyframe = last_frame.copy()
        self.keyframes.append(keyframe)
        
        # Flash LEDs to indicate keyframe marked
        self.dxl.set_led(self.pan_id, False)
        self.dxl.set_led(self.tilt_id, False)
        time.sleep(0.2)
        self.dxl.set_led(self.pan_id, True)
        self.dxl.set_led(self.tilt_id, True)
        
        self.get_logger().info(f"Marked keyframe at t={last_frame['timestamp']:.2f}s")
        response.success = True
        response.message = f"Marked keyframe at t={last_frame['timestamp']:.2f}s"
        return response
    
    def save_motion_callback(self, request, response):
        """Save recorded motion to file"""
        if not self.frames:
            response.success = False
            response.message = "No motion data to save"
            return response
        
        motion_name = request.data.strip()
        if not motion_name:
            motion_name = self.current_motion_name
        
        if not motion_name:
            response.success = False
            response.message = "No motion name provided"
            return response
        
        # Update current motion name
        self.current_motion_name = motion_name
        
        # Create motion data structure
        duration = self.frames[-1]["timestamp"] if self.frames else 0.0
        
        motion_data = {
            "metadata": {
                "name": motion_name,
                "duration": duration,
                "dof_count": 2,  # Currently pan and tilt
                "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "motor_limits": {
                    "pan": [self.dxl.pan_min_angle, self.dxl.pan_max_angle],
                    "tilt": [self.dxl.tilt_min_angle, self.dxl.tilt_max_angle]
                }
            },
            "keyframes": self.keyframes,
            "frames": self.frames
        }
        
        # Save to file
        filename = os.path.join(self.motion_files_dir, f"{motion_name}.json")
        try:
            with open(filename, 'w') as f:
                json.dump(motion_data, f, indent=2)
            
            self.get_logger().info(f"Saved motion to {filename}")
            response.success = True
            response.message = f"Saved motion '{motion_name}' ({duration:.2f}s)"
            return response
        except Exception as e:
            self.get_logger().error(f"Error saving motion: {e}")
            response.success = False
            response.message = f"Error saving motion: {e}"
            return response
    
    def load_motion_callback(self, request, response):
        """Load motion data from file"""
        motion_name = request.data.strip()
        
        if not motion_name:
            response.success = False
            response.message = "No motion name provided"
            return response
        
        # Check if adding .json is needed
        if not motion_name.endswith('.json'):
            filename = os.path.join(self.motion_files_dir, f"{motion_name}.json")
        else:
            filename = os.path.join(self.motion_files_dir, motion_name)
            motion_name = motion_name[:-5]  # Remove .json extension
        
        try:
            with open(filename, 'r') as f:
                motion_data = json.load(f)
            
            # Extract metadata
            metadata = motion_data.get("metadata", {})
            self.current_motion_name = metadata.get("name", motion_name)
            
            # Extract frames and keyframes
            self.frames = motion_data.get("frames", [])
            self.keyframes = motion_data.get("keyframes", [])
            
            duration = metadata.get("duration", 0.0)
            frame_count = len(self.frames)
            keyframe_count = len(self.keyframes)
            
            self.get_logger().info(
                f"Loaded motion '{self.current_motion_name}': {frame_count} frames, " +
                f"{keyframe_count} keyframes, {duration:.2f}s")
            
            response.success = True
            response.message = f"Loaded motion '{self.current_motion_name}'"
            return response
        except FileNotFoundError:
            self.get_logger().error(f"Motion file not found: {filename}")
            response.success = False
            response.message = f"Motion '{motion_name}' not found"
            return response
        except Exception as e:
            self.get_logger().error(f"Error loading motion: {e}")
            response.success = False
            response.message = f"Error loading motion: {e}"
            return response
    
    def play_motion_callback(self, request, response):
        """Play back the loaded motion"""
        if self.is_recording:
            response.success = False
            response.message = "Cannot play while recording"
            return response
        
        if self.is_playing:
            response.success = False
            response.message = "Already playing motion"
            return response
        
        if not self.frames:
            response.success = False
            response.message = "No motion data to play"
            return response
        
        # Enable torque for playback
        self.dxl.enable_torque(self.pan_id, True)
        self.dxl.enable_torque(self.tilt_id, True)
        
        # Set LEDs to indicate playback
        self.dxl.set_led(self.pan_id, True)
        self.dxl.set_led(self.tilt_id, True)
        
        # Start playback
        self.is_playing = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Create timer for playback
        # Use higher rate than recording for smoother playback
        playback_rate = 100.0  # Hz
        period = 1.0 / playback_rate
        self.playback_timer = self.create_timer(
            period, self.playback_tick, callback_group=self.timer_group)
        
        duration = self.frames[-1]["timestamp"] if self.frames else 0.0
        self.get_logger().info(f"Started playing motion '{self.current_motion_name}' ({duration:.2f}s)")
        
        response.success = True
        response.message = f"Playing motion '{self.current_motion_name}'"
        return response
    
    def stop_playback_callback(self, request, response):
        """Stop motion playback"""
        if not self.is_playing:
            response.success = False
            response.message = "Not playing motion"
            return response
        
        # Stop the playback timer
        if self.playback_timer:
            self.playback_timer.cancel()
            self.playback_timer = None
        
        # Turn off LEDs to indicate playback stopped
        self.dxl.set_led(self.pan_id, False)
        self.dxl.set_led(self.tilt_id, False)
        
        self.is_playing = False
        
        self.get_logger().info("Stopped motion playback")
        response.success = True
        response.message = "Stopped playback"
        return response
    
    def list_motions_callback(self, request, response):
        """List available motion files"""
        try:
            # Get all .json files in the motion files directory
            motion_files = [f for f in os.listdir(self.motion_files_dir) 
                          if f.endswith('.json')]
            
            # Sort alphabetically
            motion_files.sort()
            
            # Remove .json extension for display
            motion_names = [f[:-5] for f in motion_files]
            
            # Return as JSON list
            response.success = True
            response.message = json.dumps(motion_names)
            return response
        except Exception as e:
            self.get_logger().error(f"Error listing motion files: {e}")
            response.success = False
            response.message = f"Error listing motion files: {e}"
            return response
    
    def playback_tick(self):
        """Update playback for the current time"""
        if not self.is_playing:
            return
        
        try:
            # Calculate current playback time
            current_time = self.get_clock().now().nanoseconds / 1e9
            playback_time = current_time - self.start_time
            
            # Find frames that bracket the current time
            current_frame = None
            next_frame = None
            
            # Maximum time from the motion
            max_time = self.frames[-1]["timestamp"]
            
            # Check if playback is complete
            if playback_time > max_time:
                self.get_logger().info("Playback complete")
                self.stop_playback()
                return
            
            # Find appropriate frames
            for i, frame in enumerate(self.frames):
                if frame["timestamp"] > playback_time:
                    next_frame = frame
                    if i > 0:
                        current_frame = self.frames[i-1]
                    break
            
            # If we're at the beginning, use the first frame
            if current_frame is None and next_frame is not None:
                current_frame = next_frame
            
            # If we're at the end, use the last frame
            if next_frame is None and len(self.frames) > 0:
                current_frame = self.frames[-1]
            
            # If we couldn't find appropriate frames, exit
            if current_frame is None:
                return
            
            # Get positions
            if next_frame and current_frame != next_frame:
                # Interpolate between frames
                alpha = ((playback_time - current_frame["timestamp"]) / 
                        (next_frame["timestamp"] - current_frame["timestamp"]))
                
                pan_pos = self.interpolate(
                    current_frame["positions"][0],
                    next_frame["positions"][0],
                    alpha)
                
                tilt_pos = self.interpolate(
                    current_frame["positions"][1],
                    next_frame["positions"][1],
                    alpha)
            else:
                # Just use current frame
                pan_pos = current_frame["positions"][0]
                tilt_pos = current_frame["positions"][1]
            
            # Set motor positions
            # For smoother motion, also calculate velocities based on next frame
            if next_frame and current_frame != next_frame:
                # Time between frames
                dt = next_frame["timestamp"] - current_frame["timestamp"]
                if dt > 0:
                    # Calculate velocities in degrees/second
                    pan_vel = abs(next_frame["positions"][0] - current_frame["positions"][0]) / dt
                    tilt_vel = abs(next_frame["positions"][1] - current_frame["positions"][1]) / dt
                    
                    # Apply velocity to motor movement (optional)
                    self.dxl.set_position(self.pan_id, pan_pos, velocity=pan_vel)
                    self.dxl.set_position(self.tilt_id, tilt_pos, velocity=tilt_vel)
                else:
                    self.dxl.set_position(self.pan_id, pan_pos)
                    self.dxl.set_position(self.tilt_id, tilt_pos)
            else:
                self.dxl.set_position(self.pan_id, pan_pos)
                self.dxl.set_position(self.tilt_id, tilt_pos)
            
            # Publish current position
            pos_msg = Float32MultiArray()
            pos_msg.data = [pan_pos, tilt_pos]
            self.position_pub.publish(pos_msg)
            
            # Debug output for keyframes
            if current_frame.get("is_keyframe", False) and abs(playback_time - current_frame["timestamp"]) < 0.02:
                self.get_logger().debug(f"Keyframe at t={playback_time:.2f}s")
        
        except Exception as e:
            self.get_logger().error(f"Error in playback: {e}")
    
    def interpolate(self, val1, val2, alpha):
        """Linear interpolation between two values"""
        return val1 + alpha * (val2 - val1)
    
    def stop_playback(self):
        """Stop playback internally"""
        if self.playback_timer:
            self.playback_timer.cancel()
            self.playback_timer = None
        
        self.dxl.set_led(self.pan_id, False)
        self.dxl.set_led(self.tilt_id, False)
        self.is_playing = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotionRecorder()
        
        # Use a multithreaded executor for concurrent service handling
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error in motion recorder node: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 