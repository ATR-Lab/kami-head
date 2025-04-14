#!/usr/bin/env python3

import time
import json
import threading
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image, Joy

from .eye_controller import EyeController
from .eye_renderer import EyeRenderer
from ..common.data_types import FaceData, VelocityVector, MotorPosition

class EyeTrackingNode(Node):
    """ROS node for eye tracking visualization"""
    
    def __init__(self):
        super().__init__('eye_tracking')
        
        # Create callback groups for threading
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create core components
        self.eye_controller = EyeController()
        self.eye_renderer = EyeRenderer()
        
        # Set up component connections
        self.eye_controller.set_status_callback(self._on_status_update)
        self.eye_renderer.set_exit_callback(self._on_renderer_exit)
        
        # Declare parameters
        self._declare_parameters()
        
        # Create subscriptions
        self._create_subscriptions()
        
        # Create publishers
        self._create_publishers()
        
        # Create timers
        self.update_timer = self.create_timer(
            1.0/60.0,  # 60 Hz
            self._update_callback,
            callback_group=self.timer_callback_group
        )
        
        # Threading for renderer
        self.renderer_thread = None
        self.running = True
        
        # Start renderer
        self._start_renderer()
        
        self.get_logger().info('Eye tracking node started')
    
    def _declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('fullscreen', False)
        self.declare_parameter('eye_radius', 120)
        self.declare_parameter('pupil_radius', 40)
        self.declare_parameter('eye_spacing', 200)
        self.declare_parameter('enable_debug', False)
        self.declare_parameter('center_threshold_x', 40)
        self.declare_parameter('center_threshold_y', 25)
        
        # Apply parameters
        width = self.get_parameter('window_width').value
        height = self.get_parameter('window_height').value
        fullscreen = self.get_parameter('fullscreen').value
        eye_radius = self.get_parameter('eye_radius').value
        pupil_radius = self.get_parameter('pupil_radius').value
        eye_spacing = self.get_parameter('eye_spacing').value
        enable_debug = self.get_parameter('enable_debug').value
        center_threshold_x = self.get_parameter('center_threshold_x').value
        center_threshold_y = self.get_parameter('center_threshold_y').value
        
        # Configure components
        self.eye_renderer.set_window_size(width, height)
        self.eye_renderer.set_fullscreen(fullscreen)
        self.eye_renderer.set_eye_properties(eye_radius, pupil_radius, eye_spacing)
        self.eye_renderer.set_debug(enable_debug)
        
        self.eye_controller.set_thresholds(center_threshold_x, center_threshold_y)
    
    def _create_subscriptions(self):
        """Create ROS subscriptions"""
        # Subscribe to face detection data
        self.face_sub = self.create_subscription(
            String,
            'face_detection',
            self._face_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Subscribe to face velocity
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            'face_velocity',
            self._velocity_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Subscribe to head position
        self.head_pos_sub = self.create_subscription(
            Float64MultiArray,
            'motor_position',
            self._head_position_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Subscribe to camera parameters
        self.camera_param_sub = self.create_subscription(
            String,
            'camera_parameters',
            self._camera_param_callback,
            10,
            callback_group=self.subscription_callback_group
        )
    
    def _create_publishers(self):
        """Create ROS publishers"""
        # Publish eye status updates
        self.status_pub = self.create_publisher(
            String,
            'eye_status',
            10
        )
    
    def _start_renderer(self):
        """Start the renderer in a separate thread"""
        if self.renderer_thread is None:
            self.renderer_thread = threading.Thread(
                target=self._renderer_thread_func,
                daemon=True
            )
            self.renderer_thread.start()
    
    def _renderer_thread_func(self):
        """Thread function for running the renderer"""
        try:
            self.eye_renderer.initialize()
            
            while self.running:
                self.eye_renderer.handle_events()
                self.eye_renderer.draw()
                
                # Small sleep to reduce CPU usage
                time.sleep(0.001)
                
        except Exception as e:
            self.get_logger().error(f'Renderer error: {str(e)}')
        finally:
            self.eye_renderer.shutdown()
    
    def _update_callback(self):
        """Timer callback for updating eye tracking"""
        if not self.running:
            return
            
        # Update eye controller and get pupil positions
        left_target, right_target = self.eye_controller.update(1.0/60.0)
        
        # Update renderer with new targets
        self.eye_renderer.update_pupils(left_target, right_target)
    
    def _face_callback(self, msg):
        """Callback for face detection data"""
        try:
            # Parse JSON data
            data = json.loads(msg.data)
            
            # Extract face data if available
            face = None
            if data.get('faces') and len(data['faces']) > 0:
                face_data = data['faces'][0]  # Use the first face
                face = FaceData(
                    center_x=face_data['center_x'],
                    center_y=face_data['center_y'],
                    width=face_data['width'],
                    height=face_data['height']
                )
            
            # Update eye controller
            self.eye_controller.update_face(face, time.time())
            
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in face detection message')
        except KeyError as e:
            self.get_logger().error(f'Missing key in face data: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing face data: {str(e)}')
    
    def _velocity_callback(self, msg):
        """Callback for face velocity data"""
        if len(msg.data) >= 3:
            vx, vy, magnitude = msg.data[:3]
            velocity = VelocityVector(x=vx, y=vy, magnitude=magnitude)
            self.eye_controller.update_face_velocity(velocity)
    
    def _head_position_callback(self, msg):
        """Callback for head position data"""
        if len(msg.data) >= 2:
            pan, tilt = msg.data[:2]
            position = MotorPosition(pan_angle=pan, tilt_angle=tilt)
            self.eye_controller.update_head_position(position)
    
    def _camera_param_callback(self, msg):
        """Callback for camera parameters"""
        try:
            params = json.loads(msg.data)
            width = params.get('width', 640)
            height = params.get('height', 480)
            
            # Update eye controller with frame size
            self.eye_controller.set_frame_size(width, height)
            
            # Update renderer with camera aspect
            aspect = width / height if height > 0 else 1.0
            self.eye_renderer.set_camera_parameters(aspect)
            
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in camera parameters')
        except Exception as e:
            self.get_logger().error(f'Error processing camera parameters: {str(e)}')
    
    def _on_status_update(self, status):
        """Callback for eye tracking status updates"""
        # Publish status update
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def _on_renderer_exit(self):
        """Callback for renderer exit event"""
        self.get_logger().info('Renderer exit requested')
        self.running = False
        
        # Create one-shot timer to handle shutdown
        self.create_timer(0.1, self._shutdown)
    
    def _shutdown(self):
        """Shutdown the node"""
        self.running = False
        self.get_logger().info('Shutting down eye tracking node')
        
        # Request ROS node shutdown
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    eye_tracking_node = EyeTrackingNode()
    
    try:
        # Use MultiThreadedExecutor for better performance
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(eye_tracking_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            eye_tracking_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        eye_tracking_node.get_logger().error(f'Error: {str(e)}')
    finally:
        # Ensure clean shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 