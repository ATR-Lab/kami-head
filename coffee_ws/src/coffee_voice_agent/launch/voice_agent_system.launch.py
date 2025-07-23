#!/usr/bin/env python3
"""
Launch file for Coffee Voice Agent - Complete System Mode

This launch file runs both the LiveKit voice agent script and the ROS2 bridge node
together for a complete production system. The voice agent runs in 'start' mode
(production) and the bridge provides ROS2 integration.

Usage:
    ros2 launch coffee_voice_agent voice_agent_system.launch.py
    
    # With custom parameters:
    ros2 launch coffee_voice_agent voice_agent_system.launch.py \
        voice_agent_mode:=dev \
        voice_agent_port:=8080 \
        bridge_reconnect_interval:=3.0
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete system"""
    
    # Declare launch arguments
    voice_agent_mode_arg = DeclareLaunchArgument(
        'voice_agent_mode',
        default_value='start',
        description='Voice agent mode: console, dev, or start (production)'
    )
    
    voice_agent_port_arg = DeclareLaunchArgument(
        'voice_agent_port',
        default_value='8080',
        description='WebSocket port for voice agent server'
    )
    
    bridge_reconnect_interval_arg = DeclareLaunchArgument(
        'bridge_reconnect_interval',
        default_value='5.0',
        description='Bridge reconnection interval in seconds'
    )
    
    voice_agent_host_arg = DeclareLaunchArgument(
        'voice_agent_host',
        default_value='localhost',
        description='Host address of the voice agent (for bridge connection)'
    )
    
    # Get package share directory
    package_share = FindPackageShare('coffee_voice_agent')
    
    # Path to the voice agent script  
    script_path = os.path.join(package_share.find('coffee_voice_agent'), 'scripts', 'livekit_voice_agent.py')
    
    # Create the voice agent process
    voice_agent_process = ExecuteProcess(
        cmd=[
            'python3', 
            script_path,
            LaunchConfiguration('voice_agent_mode')
        ],
        output='screen',
        name='livekit_voice_agent',
        emulate_tty=True,
        shell=False,
        respawn=True,  # Auto-restart in production mode
        respawn_delay=5.0,
        # Set environment variables for the voice agent
        additional_env={
            'WEBSOCKET_PORT': LaunchConfiguration('voice_agent_port'),
            'WEBSOCKET_HOST': LaunchConfiguration('voice_agent_host'),
        }
    )
    
    # Create the bridge node (with delay to let voice agent start first)
    bridge_node = TimerAction(
        period=3.0,  # Wait 3 seconds for voice agent to start
        actions=[
            Node(
                package='coffee_voice_agent',
                executable='voice_agent_bridge',
                name='voice_agent_bridge',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'voice_agent_host': LaunchConfiguration('voice_agent_host'),
                    'voice_agent_port': LaunchConfiguration('voice_agent_port'),
                    'reconnect_interval': LaunchConfiguration('bridge_reconnect_interval'),
                }],
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )
    
    return LaunchDescription([
        voice_agent_mode_arg,
        voice_agent_port_arg,
        bridge_reconnect_interval_arg,
        voice_agent_host_arg,
        voice_agent_process,
        bridge_node,
    ]) 