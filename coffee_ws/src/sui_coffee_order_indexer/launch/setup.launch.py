from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('sui_indexer')
    
    return LaunchDescription([
        # First run Prisma DB setup
        ExecuteProcess(
            cmd=['ros2', 'run', 'sui_indexer', 'prisma_setup'],
            name='prisma_setup',
            output='screen'
        ),
    ]) 