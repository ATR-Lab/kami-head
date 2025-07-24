#!/usr/bin/env python3
"""
Launch file for complete Coffee Ears ESP32 System

This launch file starts the micro-ROS agent and provides a complete
interface for controlling the ESP32-based ear motion system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete ear system"""
    
    # Declare launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyUSB1',
        description='Serial device path for ESP32 connection'
    )
    
    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='6',
        description='micro-ROS agent verbosity level (0-6)'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coffee_ears',
        description='ROS namespace for ear topics'
    )
    
    # Include micro-ROS agent launch
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coffee_ears_esp32'),
                'launch',
                'microros_agent.launch.py'
            ])
        ]),
        launch_arguments={
            'device_path': LaunchConfiguration('device_path'),
            'verbosity': LaunchConfiguration('verbosity'),
            'auto_start': 'true',
            'restart_on_failure': 'true',
        }.items()
    )
    
    # Info message
    info_msg = LogInfo(
        msg=[
            'Starting Coffee Ears ESP32 System:\n',
            '  Device: ', LaunchConfiguration('device_path'), '\n',
            '  Namespace: ', LaunchConfiguration('namespace'), '\n',
            '  Available topics:\n',
            '    - /ear_motion_command (std_msgs/Int32) - Send motion commands (1-12)\n',
            '    - /microros_agent/status (std_msgs/Bool) - Agent status\n',
            '    - /microros_agent/diagnostics (std_msgs/String) - Diagnostics\n',
            '    - /microros_agent/control (std_msgs/Bool) - Start/stop agent\n',
            '  Test command:\n',
            '    ros2 topic pub /ear_motion_command std_msgs/Int32 "data: 1"'
        ]
    )
    
    return LaunchDescription([
        device_path_arg,
        verbosity_arg,
        namespace_arg,
        info_msg,
        microros_launch,
    ]) 