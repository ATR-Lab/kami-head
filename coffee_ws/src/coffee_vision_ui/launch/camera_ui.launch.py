#!/usr/bin/env python3
"""
Launch file for separated Coffee Camera UI.

This launch file starts only the camera UI node, which connects to an
existing camera_node running elsewhere. This enables:

- Remote camera monitoring and control
- UI development independent of camera processing
- Multiple UI instances monitoring the same camera
- Flexible deployment scenarios

The UI connects via /coffee_bot/ namespace topics to communicate with
the camera processing node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for separated camera UI."""
    
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='coffee_bot',
        description='Robot namespace for ROS topics (default: coffee_bot)'
    )
    
    ui_node_name_arg = DeclareLaunchArgument(
        'ui_node_name',
        default_value='camera_ui',
        description='Name for the UI node (default: camera_ui)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the UI node (debug, info, warn, error)'
    )
    
    # Camera UI node
    camera_ui_node = Node(
        package='coffee_vision_ui',
        executable='camera_ui',
        name=LaunchConfiguration('ui_node_name'),
        output='screen',
        parameters=[
            {
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'use_sim_time': False
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        robot_namespace_arg,
        ui_node_name_arg,
        log_level_arg,
        camera_ui_node
    ]) 