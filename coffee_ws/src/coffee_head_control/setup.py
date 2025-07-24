from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_head_control'

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
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='Head control package for Coffee Buddy robot with face tracking and motor control',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'head_tracking = coffee_head_control.head_tracking:main',
        ],
    },
)
