#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from coffee_expressions_msgs.msg import AffectiveState
from head_control_interfaces.srv import RequestControl
from std_msgs.msg import String, Float32
from dataclasses import dataclass
from typing import Optional, Dict, Any
import os
import json
import time
import numpy as np

@dataclass
class MotionState:
    expression: str
    frames: list
    duration: float
    start_time: float
    blend_duration: float = 0.5  # seconds to blend between motions
    prev_pan: Optional[float] = None
    prev_tilt: Optional[float] = None

class HeadMotionServer(Node):
    """Server that plays pre-recorded head motions based on expression states.
    
    Features:
    - Smooth motion blending between expressions
    - Error handling for missing/invalid motion files
    - Progress feedback
    - Motion interruption handling
    """
    
    def __init__(self):
        super().__init__('head_motion_server')
        
        # Create callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize parameters
        self.declare_parameter('motion_files_dir', 
                             os.path.expanduser('~/.ros/motion_files'))
        self.declare_parameter('blend_duration', 0.5)
        
        self.motion_files_dir = self.get_parameter('motion_files_dir').value
        self.blend_duration = self.get_parameter('blend_duration').value
        
        # State management
        self.current_motion: Optional[MotionState] = None
        self.has_control = False
        self.interrupting = False
        
        # Cache loaded motions
        self.motion_cache: Dict[str, Any] = {}
        
        # Create subscribers
        self.create_subscription(
            AffectiveState,
            'affective_state',
            self.handle_affective_state,
            10
        )
        
        self.create_subscription(
            String,
            'head_control_status',
            self.handle_control_status,
            10
        )
        
        # Create publishers
        self.position_pub = self.create_publisher(
            SetPosition,
            'head_motion/set_position',
            10
        )
        
        self.progress_pub = self.create_publisher(
            Float32,
            'head_motion/progress',
            10
        )
        
        # Create service clients
        self.control_client = self.create_client(
            RequestControl,
            'request_head_control',
            callback_group=self.service_group
        )
        
        # Create playback timer
        self.create_timer(
            0.02,  # 50Hz
            self.playback_step,
            callback_group=self.timer_group
        )
        
        # Load motion mappings
        self.motion_mappings = self.load_motion_mappings()
        self.preload_motions()
        
        self.get_logger().info('Head Motion Server initialized')
    
    def load_motion_mappings(self):
        """Load the mapping of expressions to motion files."""
        mapping_file = os.path.join(self.motion_files_dir, 'expression_motions.json')
        
        default_mappings = {
            'Happy': 'happy_motion.json',
            'Angry': 'angry_motion.json',
            'Loving': 'loving_motion.json',
            'Sad': 'sad_motion.json',
            'Surprised': 'surprised_motion.json'
        }
        
        try:
            if os.path.exists(mapping_file):
                with open(mapping_file, 'r') as f:
                    return json.load(f)
            else:
                os.makedirs(self.motion_files_dir, exist_ok=True)
                with open(mapping_file, 'w') as f:
                    json.dump(default_mappings, f, indent=2)
                return default_mappings
        except Exception as e:
            self.get_logger().error(f'Error loading motion mappings: {e}')
            return default_mappings
    
    def preload_motions(self):
        """Preload all motion files into memory."""
        for expression, filename in self.motion_mappings.items():
            try:
                filepath = os.path.join(self.motion_files_dir, filename)
                if os.path.exists(filepath):
                    with open(filepath, 'r') as f:
                        motion_data = json.load(f)
                        # Validate motion data
                        if self.validate_motion_data(motion_data):
                            self.motion_cache[expression] = motion_data
                        else:
                            self.get_logger().error(
                                f'Invalid motion data in {filename}'
                            )
                else:
                    self.get_logger().warn(f'Motion file not found: {filepath}')
            except Exception as e:
                self.get_logger().error(
                    f'Error loading motion for {expression}: {e}'
                )
    
    def validate_motion_data(self, data: Dict) -> bool:
        """Validate motion data format."""
        required_fields = ['frames', 'duration']
        frame_fields = ['timestamp', 'pan_id', 'pan_position', 
                       'tilt_id', 'tilt_position']
        
        if not all(field in data for field in required_fields):
            return False
        
        if not isinstance(data['frames'], list):
            return False
        
        for frame in data['frames']:
            if not all(field in frame for field in frame_fields):
                return False
        
        return True
    
    def handle_affective_state(self, msg: AffectiveState):
        """Handle incoming affective state messages."""
        if (not self.current_motion or 
            msg.expression != self.current_motion.expression):
            
            if msg.expression in self.motion_cache:
                self.request_control(msg.expression)
            else:
                self.get_logger().warn(
                    f'No motion available for expression: {msg.expression}'
                )
    
    def handle_control_status(self, msg: String):
        """Handle control status updates."""
        owner = msg.data.split(':')[1]
        had_control = self.has_control
        self.has_control = (owner == "head_motion_server")
        
        if had_control and not self.has_control:
            # Lost control
            self.interrupting = True
            self.current_motion = None
    
    async def request_control(self, expression: str):
        """Request control from the control manager."""
        if not self.control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Control service not available')
            return
        
        request = RequestControl.Request()
        request.controller_id = "head_motion_server"
        request.priority = 2.0  # Higher than tracking
        request.timeout = self.motion_cache[expression]['duration'] + 1.0
        
        try:
            response = await self.control_client.call_async(request)
            if response.success:
                self.get_logger().info(
                    f'Control granted for {response.granted_timeout}s'
                )
                self.start_motion(expression)
            else:
                self.get_logger().warn(
                    f'Control request denied: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(f'Error requesting control: {e}')
    
    def start_motion(self, expression: str):
        """Start playing a new motion with blending."""
        if not self.has_control:
            return
        
        motion_data = self.motion_cache[expression]
        
        # Store previous positions for blending if available
        prev_pan = None
        prev_tilt = None
        if self.current_motion:
            prev_pan = self.current_motion.prev_pan
            prev_tilt = self.current_motion.prev_tilt
        
        self.current_motion = MotionState(
            expression=expression,
            frames=motion_data['frames'],
            duration=motion_data['duration'],
            start_time=time.time(),
            blend_duration=self.blend_duration,
            prev_pan=prev_pan,
            prev_tilt=prev_tilt
        )
        
        self.get_logger().info(f'Starting motion for {expression}')
    
    def playback_step(self):
        """Execute one step of motion playback with blending."""
        if not self.current_motion or not self.has_control:
            return
        
        now = time.time()
        elapsed = now - self.current_motion.start_time
        
        # Publish progress
        progress = min(1.0, elapsed / self.current_motion.duration)
        self.progress_pub.publish(Float32(data=progress))
        
        if elapsed > self.current_motion.duration:
            self.current_motion = None
            self.get_logger().info('Motion complete')
            return
        
        # Find current and next frames
        current_frame = None
        next_frame = None
        for i, frame in enumerate(self.current_motion.frames):
            if frame['timestamp'] > elapsed:
                if i > 0:
                    current_frame = self.current_motion.frames[i-1]
                    next_frame = frame
                break
            elif i == len(self.current_motion.frames) - 1:
                current_frame = frame
        
        if not current_frame:
            return
        
        # Calculate interpolated positions
        if next_frame:
            # Interpolate between frames
            alpha = ((elapsed - current_frame['timestamp']) /
                    (next_frame['timestamp'] - current_frame['timestamp']))
            
            pan_pos = np.interp(alpha, [0, 1],
                               [current_frame['pan_position'],
                                next_frame['pan_position']])
            tilt_pos = np.interp(alpha, [0, 1],
                                [current_frame['tilt_position'],
                                 next_frame['tilt_position']])
        else:
            pan_pos = current_frame['pan_position']
            tilt_pos = current_frame['tilt_position']
        
        # Apply blending if we have previous positions
        if self.current_motion.prev_pan is not None:
            blend_alpha = min(1.0, elapsed / self.current_motion.blend_duration)
            pan_pos = np.interp(blend_alpha, [0, 1],
                               [self.current_motion.prev_pan, pan_pos])
            tilt_pos = np.interp(blend_alpha, [0, 1],
                                [self.current_motion.prev_tilt, tilt_pos])
        
        # Update previous positions
        self.current_motion.prev_pan = pan_pos
        self.current_motion.prev_tilt = tilt_pos
        
        # Publish positions
        msg = SetPosition()
        msg.id = current_frame['pan_id']
        msg.position = int(pan_pos)
        self.position_pub.publish(msg)
        
        msg.id = current_frame['tilt_id']
        msg.position = int(tilt_pos)
        self.position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadMotionServer()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
