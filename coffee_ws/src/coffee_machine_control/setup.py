from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'coffee_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['coffee_control', 'coffee_control.*']),
    data_files=[
        ('lib/coffee_control', ['scripts/coffee_control_node']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools', 'bleak', 'rclpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 node for controlling the Delonghi coffee machine via Bluetooth',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coffee_control_node = coffee_control.coffee_control_node:main',
        ],
    },
)
