#!/usr/bin/env python3
"""
Launch file for Coffee Voice Agent - Bridge Only Mode

This launch file runs only the ROS2 bridge node, assuming the LiveKit voice agent
is already running separately (e.g., started manually or by another process).

The bridge connects to the voice agent via WebSocket and provides ROS2 integration.

Usage:
    ros2 launch coffee_voice_agent voice_agent_bridge.launch.py
    
    # With custom parameters:
    ros2 launch coffee_voice_agent voice_agent_bridge.launch.py \
        voice_agent_host:=192.168.1.100 \
        voice_agent_port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for bridge-only mode"""
    
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'voice_agent_host',
        default_value='localhost',
        description='Host address of the voice agent WebSocket server'
    )
    
    port_arg = DeclareLaunchArgument(
        'voice_agent_port',
        default_value='8080',
        description='Port of the voice agent WebSocket server'
    )
    
    reconnect_interval_arg = DeclareLaunchArgument(
        'reconnect_interval',
        default_value='5.0',
        description='Reconnection interval in seconds'
    )
    
    # Create the bridge node
    bridge_node = Node(
        package='coffee_voice_agent',
        executable='voice_agent_bridge',
        name='voice_agent_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voice_agent_host': LaunchConfiguration('voice_agent_host'),
            'voice_agent_port': LaunchConfiguration('voice_agent_port'),
            'reconnect_interval': LaunchConfiguration('reconnect_interval'),
        }],
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        reconnect_interval_arg,
        bridge_node,
    ]) 