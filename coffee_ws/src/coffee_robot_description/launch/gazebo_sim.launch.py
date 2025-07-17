#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    ExecuteProcess,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution, 
    Command
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    """
    Launch file for Gazebo simulation of the coffee robot
    """
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Gazebo world file'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot X position in world'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot Y position in world'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Robot Z position in world'
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
        'use_sim_time': True
    }
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Robot State Publisher node (for simulation)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        emulate_tty=True,
    )
    
    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_coffee_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'coffee_robot',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose')
        ],
        emulate_tty=True,
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        emulate_tty=True,
    )
    
    # Coffee Head Controller (for the neck joints)
    coffee_head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["coffee_head_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        emulate_tty=True,
    )
    
    # Coffee Ear Controller (for the ear joints)  
    coffee_ear_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["coffee_ear_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        emulate_tty=True,
    )
    
    # Register event handlers to spawn controllers after robot is spawned
    joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    coffee_head_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[coffee_head_controller_spawner],
        )
    )
    
    coffee_ear_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=coffee_head_controller_spawner,
            on_exit=[coffee_ear_controller_spawner],
        )
    )
    
    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot_node,
        joint_state_broadcaster_event,
        coffee_head_controller_event,
        coffee_ear_controller_event,
    ]) 