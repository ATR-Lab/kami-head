from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for coffee_vision nodes"""
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Face detection confidence threshold'
    )
    
    # Camera node
    camera_node = Node(
        package='coffee_vision',
        executable='camera_node',
        name='camera_node',
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
    
    # Face detection node
    face_detection_node = Node(
        package='coffee_vision',
        executable='face_detection_node',
        name='face_detection_node',
        output='screen',
        parameters=[
            {
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'nms_threshold': 0.4,
                'enable_smoothing': True
            }
        ]
    )
    
    return LaunchDescription([
        camera_index_arg,
        confidence_threshold_arg,
        camera_node,
        # face_detection_node
    ]) 