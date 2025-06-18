from setuptools import find_packages, setup

package_name = 'coffee_voice_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'typeguard',
        'python-dotenv',
        'pvporcupine==3.0.5',
        'pvrecorder==1.2.7',
        'livekit',
        'livekit-agents[openai,deepgram,silero,turn-detector]',
        'livekit-plugins-noise-cancellation',
        'empy==3.3.4',  # Required for ROS2 compatibility
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='Coffee barista voice agent ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_agent_node = coffee_voice_agent.voice_agent_node:main',
        ],
    },
)
