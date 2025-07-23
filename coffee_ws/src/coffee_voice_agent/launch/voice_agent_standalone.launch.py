#!/usr/bin/env python3
"""
Launch file for Coffee Voice Agent - Standalone Mode

This launch file runs only the LiveKit voice agent script in console mode.
Console mode provides local CLI functionality with interactive controls.

For console mode to work properly, run this in a proper terminal:
    ros2 launch coffee_voice_agent voice_agent_standalone.launch.py

Usage:
    ros2 launch coffee_voice_agent voice_agent_standalone.launch.py
    ros2 launch coffee_voice_agent voice_agent_standalone.launch.py mode:=dev    # Development mode  
    ros2 launch coffee_voice_agent voice_agent_standalone.launch.py mode:=start  # Production mode
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for standalone voice agent"""
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='console',
        description='Voice agent mode: console (interactive CLI), dev, or start (production)'
    )
    
    # Get package share directory
    package_share = FindPackageShare('coffee_voice_agent')
    
    # Path to the voice agent script
    script_path = os.path.join(package_share.find('coffee_voice_agent'), 'scripts', 'livekit_voice_agent.py')
    
    # Create the process to run the voice agent
    voice_agent_process = ExecuteProcess(
        cmd=[
            'python3', 
            script_path,
            LaunchConfiguration('mode')
        ],
        output='screen',
        name='livekit_voice_agent',
        emulate_tty=True,
        shell=False,
        respawn=False,  # Don't auto-restart in console mode
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        mode_arg,
        voice_agent_process,
    ]) 