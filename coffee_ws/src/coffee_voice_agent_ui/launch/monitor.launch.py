#!/usr/bin/env python3
"""
Launch file for Coffee Voice Agent Monitor UI.

This launch file starts only the voice agent monitor UI, which connects to an
existing voice agent bridge running elsewhere. This enables:

- Remote voice agent monitoring and control
- UI development independent of voice agent processing
- Multiple UI instances monitoring the same voice agent
- Flexible deployment scenarios

The UI connects via voice_agent/* topics to communicate with
the voice agent bridge node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for voice agent monitor UI."""
    
    # Declare launch arguments
    ui_node_name_arg = DeclareLaunchArgument(
        'ui_node_name',
        default_value='voice_agent_monitor',
        description='Name for the UI node (default: voice_agent_monitor)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the UI node (debug, info, warn, error)'
    )
    
    window_title_arg = DeclareLaunchArgument(
        'window_title',
        default_value='Coffee Voice Agent Monitor',
        description='Title for the monitor window'
    )
    
    # Voice Agent Monitor UI node
    monitor_ui_node = Node(
        package='coffee_voice_agent_ui',
        executable='voice_agent_monitor',
        name=LaunchConfiguration('ui_node_name'),
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'window_title': LaunchConfiguration('window_title')
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        ui_node_name_arg,
        log_level_arg,
        window_title_arg,
        monitor_ui_node
    ]) 