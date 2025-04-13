from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'python_dynamixel_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UI for controlling Dynamixel motors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_ui = python_dynamixel_ui.dynamixel_ui_node:main',
        ],
    },
)
