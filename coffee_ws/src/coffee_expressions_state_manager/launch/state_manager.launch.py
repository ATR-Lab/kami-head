from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'idle_timeout',
            default_value='5.0',
            description='Timeout in seconds before considering the robot idle'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='0.1',
            description='Rate in seconds at which to publish state updates'
        ),
        DeclareLaunchArgument(
            'default_expression',
            default_value='Neutral',
            description='Default expression when no other inputs are active'
        ),

        # State manager node
        Node(
            package='coffee_expressions_state_manager',
            executable='state_manager_node',
            name='state_manager',
            parameters=[{
                'idle_timeout': LaunchConfiguration('idle_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'default_expression': LaunchConfiguration('default_expression'),
            }],
            output='screen',
        )
    ])
