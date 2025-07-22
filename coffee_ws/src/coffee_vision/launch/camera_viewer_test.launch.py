from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for testing ROS camera transport performance.
    
    This launches both:
    1. camera_node (publisher) - captures and publishes camera frames
    2. camera_viewer_test (subscriber) - receives and displays frames via ROS
    
    Use this to test the latency and performance of UI separation via ROS transport.
    """
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index to use'
    )
    
    # Camera node - the publisher
    camera_node = Node(
        package='coffee_vision',
        executable='camera_node',
        name='camera_publisher',
        output='screen',
        parameters=[
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'frame_width': 640,
                'frame_height': 480,
                'target_fps': 30
            }
        ]
    )
    
    # Camera viewer test - the subscriber
    camera_viewer_test = Node(
        package='coffee_vision',
        executable='camera_viewer_test',
        name='camera_viewer_test',
        output='screen'
    )
    
    return LaunchDescription([
        camera_index_arg,
        camera_node,
        camera_viewer_test
    ]) 