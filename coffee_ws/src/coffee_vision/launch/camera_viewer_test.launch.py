from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for separated camera UI architecture.
    
    This launches both:
    1. camera_node (publisher) - captures and publishes camera frames + handles commands
    2. camera_viewer_test (subscriber/UI) - full-featured UI that controls camera via ROS
    
    The UI communicates via /coffee_bot/ namespace topics for camera control,
    status updates, and video streaming. This tests the feasibility of complete
    UI separation from camera processing.
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
    
    # Camera UI - the separated interface
    camera_viewer_test = Node(
        package='coffee_vision',
        executable='camera_viewer_test',
        name='camera_ui',
        output='screen'
    )
    
    return LaunchDescription([
        camera_index_arg,
        camera_node,
        camera_viewer_test
    ]) 