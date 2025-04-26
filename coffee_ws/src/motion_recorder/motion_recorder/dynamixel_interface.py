#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dynamixel interface for Motion Recorder
Handles communication with Dynamixel XM540-W270 servo motors
"""

import os
import time
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Import Dynamixel SDK

class DynamixelInterface:
    """Handles direct communication with Dynamixel XM540-W270 servo motors"""
    
    def __init__(self, node, port='/dev/ttyUSB0', baudrate=1000000):
        """Initialize Dynamixel interface"""
        self.node = node
        
        # XM540-W270 specific settings
        self.PROTOCOL_VERSION = 2.0
        self.BAUDRATE = baudrate
        self.DEVICENAME = port
        
        # Control table addresses for XM540-W270
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_LED = 65  # LED address to provide user feedback
        
        # Operating modes
        self.POSITION_CONTROL_MODE = 3
        
        # Motor IDs (currently using 2 motors)
        self.pan_id = 1   # Pan motor
        self.tilt_id = 9  # Tilt motor
        
        # Motor angle limits (in degrees)
        self.pan_min_angle = 143.0 # 0.01   # Min pan angle (right)
        self.pan_max_angle = 210.0 # 180.0  # Max pan angle (left)
        self.tilt_min_angle = 169.0 # 135.0 # Min tilt angle (up)
        self.tilt_max_angle = 206.0 # 225.0 # Max tilt angle (down)
        
        # Initialize port handler and packet handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        # Connect to device
        self.connect()
    
    def connect(self):
        """Connect to Dynamixel device"""
        try:
            if self.portHandler.openPort():
                self.node.get_logger().info(f"Succeeded to open port {self.DEVICENAME}")
            else:
                self.node.get_logger().error(f"Failed to open port {self.DEVICENAME}")
                return False
                
            if self.portHandler.setBaudRate(self.BAUDRATE):
                self.node.get_logger().info(f"Succeeded to set baudrate to {self.BAUDRATE}")
            else:
                self.node.get_logger().error(f"Failed to change baudrate to {self.BAUDRATE}")
                return False
            
            # Check connection to motors
            if self.ping(self.pan_id) and self.ping(self.tilt_id):
                self.node.get_logger().info("Successfully connected to all motors")
                return True
            else:
                self.node.get_logger().warning("Could not ping all motors")
                return False
        except Exception as e:
            self.node.get_logger().error(f"Exception during connection: {e}")
            return False
    
    def ping(self, motor_id):
        """Ping a specific motor"""
        try:
            model_number, comm_result, error = self.packetHandler.ping(self.portHandler, motor_id)
            if comm_result != COMM_SUCCESS:
                self.node.get_logger().warning(f"Failed to ping motor {motor_id}: {self.packetHandler.getTxRxResult(comm_result)}")
                return False
            if error != 0:
                self.node.get_logger().warning(f"Error from motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
                return False
            self.node.get_logger().info(f"Motor {motor_id} is connected (Model: {model_number})")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Exception pinging motor {motor_id}: {e}")
            return False
    
    def enable_torque(self, motor_id, enable=True):
        """Toggle torque on/off for manual positioning"""
        try:
            comm_result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.ADDR_TORQUE_ENABLE, 
                1 if enable else 0)
            
            if comm_result != COMM_SUCCESS:
                self.node.get_logger().error(f"Failed to set torque for motor {motor_id}: {self.packetHandler.getTxRxResult(comm_result)}")
                return False
            if error != 0:
                self.node.get_logger().warning(f"Error from motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
                return False
            
            self.node.get_logger().info(f"Motor {motor_id} torque {'enabled' if enable else 'disabled'}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Exception setting torque: {e}")
            return False
    
    def set_led(self, motor_id, on=True):
        """Set LED on/off for user feedback during recording"""
        try:
            comm_result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.ADDR_LED, 
                1 if on else 0)
            
            if comm_result != COMM_SUCCESS or error != 0:
                self.node.get_logger().warning(f"Failed to set LED for motor {motor_id}")
                return False
            return True
        except Exception as e:
            self.node.get_logger().error(f"Exception setting LED: {e}")
            return False
    
    def convert_to_position_value(self, angle, motor_id):
        """Convert from angle (degrees) to position value"""
        # XM540-W270 has 4096 steps over 360 degrees
        position_value = int((angle / 360.0) * 4096)
        return position_value
    
    def convert_to_angle(self, position_value, motor_id):
        """Convert from position value to angle (degrees)"""
        # XM540-W270 has 4096 steps over 360 degrees
        angle = (position_value * 360.0) / 4096.0
        
        # Apply joint-specific limits
        if motor_id == self.pan_id:  # Pan motor
            # Map to range 0.01-180.0
            return max(self.pan_min_angle, min(self.pan_max_angle, angle))
        else:  # Tilt motor
            # Map to range 135.0-225.0
            return max(self.tilt_min_angle, min(self.tilt_max_angle, angle))
    
    def read_position(self, motor_id):
        """Read current position in degrees"""
        try:
            position, result, error = self.packetHandler.read4ByteTxRx(
                self.portHandler, motor_id, self.ADDR_PRESENT_POSITION)
            
            if result != COMM_SUCCESS or error != 0:
                self.node.get_logger().warning(f"Failed to read position from motor {motor_id}")
                return None
            
            # Convert to degrees
            angle = self.convert_to_angle(position, motor_id)
            return angle
        except Exception as e:
            self.node.get_logger().error(f"Exception reading position: {e}")
            return None
    
    def set_position(self, motor_id, angle, velocity=None):
        """Set position in degrees with optional velocity profile"""
        try:
            # Enforce angle limits based on motor
            if motor_id == self.pan_id:
                angle = max(self.pan_min_angle, min(self.pan_max_angle, angle))
            else:
                angle = max(self.tilt_min_angle, min(self.tilt_max_angle, angle))
            
            # Convert to position value
            position_value = self.convert_to_position_value(angle, motor_id)
            
            # Set velocity profile if specified
            if velocity is not None:
                # Convert velocity from degrees/second to Dynamixel units
                # For XM540-W270, velocity units are roughly 0.229 rpm per unit
                vel_value = int(velocity / (0.229 * 6.0))  # Convert degrees/sec to units
                self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_id, self.ADDR_PROFILE_VELOCITY, vel_value)
            
            # Set position
            result, error = self.packetHandler.write4ByteTxRx(
                self.portHandler, motor_id, self.ADDR_GOAL_POSITION, position_value)
            
            if result != COMM_SUCCESS or error != 0:
                self.node.get_logger().warning(f"Failed to set position for motor {motor_id}")
                return False
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Exception setting position: {e}")
            return False
    
    def close(self):
        """Close the port"""
        try:
            # Disable torque on all motors
            self.enable_torque(self.pan_id, False)
            self.enable_torque(self.tilt_id, False)
            
            # Close port
            self.portHandler.closePort()
            self.node.get_logger().info("Dynamixel interface closed")
        except Exception as e:
            self.node.get_logger().error(f"Exception during close: {e}") 