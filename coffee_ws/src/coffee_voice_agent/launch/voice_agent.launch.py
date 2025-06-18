#!/usr/bin/env python3
"""
Launch file for Coffee Voice Agent ROS2 Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coffee_voice_agent',
            executable='voice_agent_node',
            name='coffee_voice_agent',
            output='screen',
            emulate_tty=True,
            parameters=[
                # Add parameters here as needed
            ]
        )
    ]) 