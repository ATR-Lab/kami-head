#!/usr/bin/env python3
"""
Launch file for complete Coffee Camera System.

This launch file starts both the camera processing node and the separated UI,
providing a complete system for testing the separated architecture.

Components launched:
1. camera_node (from coffee_vision package) - Camera processing and ROS publishing
2. camera_ui (from coffee_vision_ui package) - Separated control interface

This enables testing of the full separated architecture while maintaining
backwards compatibility with the integrated approach.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete camera system."""
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index to use'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='coffee_bot',
        description='Robot namespace for ROS topics'
    )
    
    enable_ui_arg = DeclareLaunchArgument(
        'enable_ui',
        default_value='true',
        description='Enable separated UI (true/false)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes (debug, info, warn, error)'
    )
    
    # Camera processing node (from coffee_vision package)
    camera_node = Node(
        package='coffee_vision',
        executable='camera_node',
        name='camera_processing',
        output='screen',
        parameters=[
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'frame_width': 640,
                'frame_height': 480,
                'target_fps': 30
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Separated camera UI (from coffee_vision_ui package)
    # Use IncludeLaunchDescription to include the camera_ui.launch.py
    camera_ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coffee_vision_ui'),
                'launch',
                'camera_ui.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'ui_node_name': 'camera_ui_separated',
            'log_level': LaunchConfiguration('log_level')
        }.items(),
        condition=None  # Always launch UI for now
    )
    
    # Alternative: Direct node launch (if we don't want to use IncludeLaunchDescription)
    # camera_ui_node = Node(
    #     package='coffee_vision_ui',
    #     executable='camera_ui',
    #     name='camera_ui_separated',
    #     output='screen',
    #     parameters=[
    #         {
    #             'robot_namespace': LaunchConfiguration('robot_namespace'),
    #             'use_sim_time': False
    #         }
    #     ],
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    #     condition=IfCondition(LaunchConfiguration('enable_ui'))
    # )
    
    return LaunchDescription([
        camera_index_arg,
        robot_namespace_arg,
        enable_ui_arg,
        log_level_arg,
        camera_node,
        camera_ui_launch
    ]) 