from setuptools import find_packages, setup
import glob
import os

package_name = 'coffee_voice_agent'

def get_data_files():
    """Get all data files for the scripts directory including subdirectories"""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ]
    
    # Add top-level script files
    script_files = glob.glob('scripts/*.py') + glob.glob('scripts/*.sh')
    if script_files:
        data_files.append(('share/' + package_name + '/scripts', script_files))
    
    # Add subdirectory files recursively
    for root, dirs, files in os.walk('scripts'):
        if root == 'scripts':
            continue  # Skip root, already handled above
            
        # Get Python files in this directory
        py_files = [os.path.join(root, f) for f in files if f.endswith('.py')]
        if py_files:
            # Convert path for installation
            rel_path = os.path.relpath(root, 'scripts')
            install_path = f'share/{package_name}/scripts/{rel_path}'
            data_files.append((install_path, py_files))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'typeguard',
        'python-dotenv',
        'pvporcupine==3.0.5',
        'pvrecorder==1.2.7',
        'livekit',
        'livekit-agents[openai,deepgram,silero,turn-detector]',
        'livekit-plugins-noise-cancellation',
        'empy==3.3.4',  # Required for ROS2 compatibility
        'websockets',
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='Coffee barista voice agent ROS2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_agent_bridge = coffee_voice_agent.voice_agent_bridge:main',
        ],
    },
)
