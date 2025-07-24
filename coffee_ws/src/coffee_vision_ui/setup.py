from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_vision_ui'

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
    maintainer_email='todo@todo.com',
    description='Separated UI package for coffee vision camera control and monitoring',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_ui = coffee_vision_ui.camera_ui_node:main',
        ],
    },
)
