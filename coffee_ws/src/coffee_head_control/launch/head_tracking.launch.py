from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for head tracking node"""
    
    # Declare launch arguments
    enable_tracking_arg = DeclareLaunchArgument(
        'enable_tracking',
        default_value='false',
        description='Start with head tracking enabled'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Motor update rate in Hz'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Dynamixel motor baud rate'
    )
    
    # Head tracking node
    head_tracking_node = Node(
        package='coffee_head_control',
        executable='head_tracking',
        name='head_tracking_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'enable_tracking': LaunchConfiguration('enable_tracking'),
                'update_rate': LaunchConfiguration('update_rate'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }
        ]
    )
    
    return LaunchDescription([
        enable_tracking_arg,
        update_rate_arg,
        baud_rate_arg,
        head_tracking_node
    ]) 