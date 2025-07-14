from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_head_motion_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='Marcus.Arnett10@gmail.com',
    description='Motion recorder and playback for Dynamixel servo-controlled robot head',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = coffee_head_motion_recorder.recorder_node:main',
            'recorder_ui = coffee_head_motion_recorder.recorder_ui:main',
        ],
    },
)
