#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import math
from rclpy.qos import QoSProfile

class CoffeeRobotTFPublisher(Node):
    """
    TF Publisher for Coffee Robot Description
    
    Integrates with existing head control system by subscribing to head angles
    and publishing joint states for robot_state_publisher.
    """
    
    def __init__(self):
        super().__init__('coffee_robot_tf_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Current joint states (in radians, centered at 0)
        self.neck_yaw_angle = 0.0      # Pan motor (ID:1)
        self.neck_pitch_angle = 0.0    # Tilt motor (ID:9)  
        self.neck_roll_angle = 0.0     # Future roll motor
        self.left_ear_angle = 0.0      # Future left ear motor
        self.right_ear_angle = 0.0     # Future right ear motor
        
        # Track if we have received any data
        self.has_pan_data = False
        self.has_tilt_data = False
        
        # Joint state publisher for robot_state_publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            QoSProfile(depth=10)
        )
        
        # Subscribe to existing head control topics
        self.create_subscription(
            Float32, 
            '/head_pan_angle', 
            self.pan_angle_callback, 
            10
        )
        
        self.create_subscription(
            Float32, 
            '/head_tilt_angle', 
            self.tilt_angle_callback, 
            10
        )
        
        # Timer for publishing joint states at 30Hz
        self.joint_timer = self.create_timer(1.0/30.0, self.publish_joint_states)
        
        # Timer for additional TF frames (if needed)
        self.tf_timer = self.create_timer(1.0/30.0, self.publish_additional_transforms)
        
        self.get_logger().info('Coffee Robot TF Publisher started')
        self.get_logger().info('Subscribing to /head_pan_angle and /head_tilt_angle')
        
    def pan_angle_callback(self, msg):
        """
        Callback for pan angle from existing head control system
        Convert from motor coordinate system (143-210°, center=180°) to URDF (-37° to +30°, center=0°)
        """
        # Motor coordinate: 143° to 210°, center at 180°
        # URDF coordinate: -37° to +30°, center at 0°
        motor_angle = msg.data
        urdf_angle = math.radians(motor_angle - 180.0)  # Convert to radians, center at 0
        
        # Clamp to joint limits
        urdf_angle = max(math.radians(-37), min(math.radians(30), urdf_angle))
        
        self.neck_yaw_angle = urdf_angle
        self.has_pan_data = True
        
        self.get_logger().debug(f'Pan: {motor_angle:.1f}° -> {math.degrees(urdf_angle):.1f}°')
        
    def tilt_angle_callback(self, msg):
        """
        Callback for tilt angle from existing head control system
        Convert from motor coordinate system (169-206°, center=180°) to URDF (-11° to +26°, center=0°)
        """
        # Motor coordinate: 169° to 206°, center at 180°  
        # URDF coordinate: -11° to +26°, center at 0°
        motor_angle = msg.data
        urdf_angle = math.radians(motor_angle - 180.0)  # Convert to radians, center at 0
        
        # Clamp to joint limits
        urdf_angle = max(math.radians(-11), min(math.radians(26), urdf_angle))
        
        self.neck_pitch_angle = urdf_angle
        self.has_tilt_data = True
        
        self.get_logger().debug(f'Tilt: {motor_angle:.1f}° -> {math.degrees(urdf_angle):.1f}°')
        
    def publish_joint_states(self):
        """
        Publish joint states for robot_state_publisher
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Joint names matching URDF
        joint_state.name = [
            'neck_yaw_joint',      # Pan motor (ID:1)
            'neck_pitch_joint',    # Tilt motor (ID:9)
            'neck_roll_joint',     # Future roll motor
            'left_ear_joint',      # Future left ear motor
            'right_ear_joint'      # Future right ear motor
        ]
        
        # Joint positions (in radians)
        joint_state.position = [
            self.neck_yaw_angle,
            self.neck_pitch_angle,
            self.neck_roll_angle,
            self.left_ear_angle,
            self.right_ear_angle
        ]
        
        # Velocities (zero for now)
        joint_state.velocity = [0.0] * len(joint_state.name)
        
        # Efforts (zero for now)
        joint_state.effort = [0.0] * len(joint_state.name)
        
        self.joint_state_publisher.publish(joint_state)
        
        # Log status periodically
        if hasattr(self, '_last_log_time'):
            if (self.get_clock().now().nanoseconds - self._last_log_time) > 5e9:  # Every 5 seconds
                self._log_status()
        else:
            self._last_log_time = self.get_clock().now().nanoseconds
            self._log_status()
            
    def _log_status(self):
        """Log current status"""
        self._last_log_time = self.get_clock().now().nanoseconds
        
        pan_status = "✓" if self.has_pan_data else "✗"
        tilt_status = "✓" if self.has_tilt_data else "✗"
        
        self.get_logger().info(
            f'TF Status - Pan: {pan_status} ({math.degrees(self.neck_yaw_angle):.1f}°), '
            f'Tilt: {tilt_status} ({math.degrees(self.neck_pitch_angle):.1f}°)'
        )
        
    def publish_additional_transforms(self):
        """
        Publish additional TF frames if needed
        (robot_state_publisher handles most transforms)
        """
        # Currently, robot_state_publisher handles all our transforms
        # This method is available for future custom transforms
        pass
        
    def set_ear_angles(self, left_angle, right_angle):
        """
        Set ear angles (for future use when ear motors are added)
        
        Args:
            left_angle: Left ear angle in radians
            right_angle: Right ear angle in radians
        """
        self.left_ear_angle = max(math.radians(-45), min(math.radians(45), left_angle))
        self.right_ear_angle = max(math.radians(-45), min(math.radians(45), right_angle))
        
    def set_roll_angle(self, roll_angle):
        """
        Set roll angle (for future use when roll motor is added)
        
        Args:
            roll_angle: Roll angle in radians
        """
        self.neck_roll_angle = max(math.radians(-30), min(math.radians(30), roll_angle))


def main(args=None):
    rclpy.init(args=args)
    
    tf_publisher = CoffeeRobotTFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 