#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file for the motion recorder package.
Launches both the recorder node and the UI node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for motion recorder."""
    
    # Declare launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default='1000000')
    sampling_rate = LaunchConfiguration('sampling_rate', default='50.0')
    motion_files_dir = LaunchConfiguration(
        'motion_files_dir', 
        default=PathJoinSubstitution([EnvironmentVariable('HOME'), '.ros/motion_files'])
    )
    
    # Create list of declared arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for connecting to Dynamixel servos'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Baudrate for Dynamixel communication'
        ),
        DeclareLaunchArgument(
            'sampling_rate',
            default_value='50.0',
            description='Sampling rate in Hz for motion recording'
        ),
        DeclareLaunchArgument(
            'motion_files_dir',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), '.ros/motion_files']),
            description='Directory to store motion files'
        ),
    ]
    
    # Configure nodes
    recorder_node = Node(
        package='motion_recorder',
        executable='recorder_node',
        name='motion_recorder',
        output='screen',
        parameters=[{
            'port': serial_port,
            'baudrate': baudrate,
            'sampling_rate': sampling_rate,
            'motion_files_dir': motion_files_dir,
        }],
        emulate_tty=True
    )
    
    ui_node = Node(
        package='motion_recorder',
        executable='recorder_ui',
        name='motion_recorder_ui',
        output='screen',
        emulate_tty=True
    )
    
    # Return launch description
    return LaunchDescription(
        declared_arguments + [recorder_node, ui_node]
    ) 