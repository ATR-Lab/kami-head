#!/usr/bin/env python3
"""
Launch file for complete Coffee Voice Agent Monitor System.

This launch file starts both the voice agent bridge and the monitor UI,
providing a complete system for monitoring the voice agent system.

Components launched:
1. voice_agent_bridge (from coffee_voice_agent package) - ROS2 bridge to voice agent
2. voice_agent_monitor (from coffee_voice_agent_ui package) - Monitoring interface

This enables testing of the complete monitoring system and provides
a one-command solution for operators.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete voice agent monitor system."""
    
    # Declare launch arguments
    voice_agent_host_arg = DeclareLaunchArgument(
        'voice_agent_host',
        default_value='localhost',
        description='Host address of the voice agent WebSocket server'
    )
    
    voice_agent_port_arg = DeclareLaunchArgument(
        'voice_agent_port',
        default_value='8080',
        description='Port of the voice agent WebSocket server'
    )
    
    enable_ui_arg = DeclareLaunchArgument(
        'enable_ui',
        default_value='true',
        description='Enable monitor UI (true/false)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes (debug, info, warn, error)'
    )
    
    # Voice Agent Bridge node (from coffee_voice_agent package)
    voice_agent_bridge_node = Node(
        package='coffee_voice_agent',
        executable='voice_agent_bridge',
        name='voice_agent_bridge',
        output='screen',
        parameters=[
            {
                'voice_agent_host': LaunchConfiguration('voice_agent_host'),
                'voice_agent_port': LaunchConfiguration('voice_agent_port'),
                'reconnect_interval': 5.0
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Voice Agent Monitor UI (from coffee_voice_agent_ui package)
    # Use IncludeLaunchDescription to include the monitor.launch.py
    monitor_ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coffee_voice_agent_ui'),
                'launch',
                'monitor.launch.py'
            ])
        ]),
        launch_arguments={
            'ui_node_name': 'voice_agent_monitor_ui',
            'log_level': LaunchConfiguration('log_level'),
            'window_title': 'Coffee Voice Agent Monitor - Full System'
        }.items(),
        condition=None  # Always launch UI for now
    )
    
    # Alternative: Direct node launch (if we don't want to use IncludeLaunchDescription)
    # monitor_ui_node = Node(
    #     package='coffee_voice_agent_ui',
    #     executable='voice_agent_monitor',
    #     name='voice_agent_monitor_ui',
    #     output='screen',
    #     parameters=[
    #         {
    #             'use_sim_time': False,
    #             'window_title': 'Coffee Voice Agent Monitor - Full System'
    #         }
    #     ],
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    #     condition=IfCondition(LaunchConfiguration('enable_ui'))
    # )
    
    return LaunchDescription([
        voice_agent_host_arg,
        voice_agent_port_arg,
        enable_ui_arg,
        log_level_arg,
        voice_agent_bridge_node,
        monitor_ui_launch
    ]) 