#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import conditions
from launch import substitutions
import os

def generate_launch_description():
    """
    Launch file for robot state publisher and TF publisher
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_manual_control_arg = DeclareLaunchArgument(
        'use_manual_control',
        default_value='false',
        description='Use manual joint control instead of hardware integration'
    )
    
    # Get the URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('coffee_robot_description'),
        'urdf',
        'coffee_robot.urdf.xacro'
    ])
    
    # Process the URDF file with xacro
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Robot description parameter
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        emulate_tty=True,
    )
    
    # Coffee Robot TF Publisher (integrates with existing head control)
    # Only runs in hardware integration mode (when use_manual_control is false)
    coffee_tf_publisher_node = Node(
        package='coffee_robot_description',
        executable='tf_publisher',
        name='coffee_robot_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=conditions.UnlessCondition(LaunchConfiguration('use_manual_control')),
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_manual_control_arg,
        robot_state_publisher_node,
        coffee_tf_publisher_node,
    ]) 