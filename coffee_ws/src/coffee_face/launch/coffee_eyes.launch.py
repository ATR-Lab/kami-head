from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='coffee_face',
            executable='coffee_eyes',
            name='coffee_eyes',
            output='screen',
            parameters=[
                {'screen_width': 1080},
                {'screen_height': 600},
                {'movement_speed': 1.0},
                {'face_tracking_enabled': True},
                {'frame_width': 640},
                {'frame_height': 480},
                {'invert_x': False},
                {'invert_y': False},
                {'eye_range': 3.0}
            ]
        )
    ]) 