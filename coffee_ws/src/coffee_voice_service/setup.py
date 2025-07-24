from setuptools import find_packages, setup

package_name = 'coffee_voice_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tts_node.launch.py']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='vr-workstation-2',
    maintainer_email='marcus@opendive.io',
    description='ROS2 package providing Text-to-Speech (TTS) service for the Coffee Buddy robot system using ElevenLabs API',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = coffee_voice_service.tts_node:main',
        ],
    },
)
