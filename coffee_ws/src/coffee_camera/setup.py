from setuptools import setup
import os
from glob import glob

package_name = 'coffee_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='kpatch@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = coffee_camera.camera_node:main',
            'head_tracking = coffee_camera.head_tracking:main',
            'integrated_camera = coffee_camera.integrated_camera_node:main',
        ],
    },
)
