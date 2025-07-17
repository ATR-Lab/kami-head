#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import conditions
import os

def generate_launch_description():
    """
    Launch file for RViz visualization of the coffee robot
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('coffee_robot_description'),
            'rviz',
            'coffee_robot.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Include robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coffee_robot_description'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True,
    )
    
    # Launch argument for choosing control mode
    use_manual_control_arg = DeclareLaunchArgument(
        'use_manual_control',
        default_value='false',
        description='Use manual joint control instead of hardware integration'
    )
    
    # Joint State Publisher GUI (for manual joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=conditions.IfCondition(LaunchConfiguration('use_manual_control')),
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        use_manual_control_arg,
        robot_state_publisher_launch,
        rviz_node,
        joint_state_publisher_gui_node,
    ]) 