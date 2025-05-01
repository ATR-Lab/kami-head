from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'silero-vad',
    ],
    zip_safe=True,
    maintainer='opendive-technologies',
    maintainer_email='Marcus@opendive.io',
    description='Perception input nodes for Coffee-Buddy',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'voice_intent_node = perception_nodes.sensor_nodes.voice_nodes.voice_intent_node.node:main',
            'vision_node = perception_nodes.sensor_nodes.vision_nodes.vision_node:main',
            'vision_face_position_node = perception_nodes.sensor_nodes.vision_nodes.vision_face_position_node.node:main',
        ],
    },
)