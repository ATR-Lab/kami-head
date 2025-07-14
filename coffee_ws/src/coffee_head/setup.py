from setuptools import setup
import os
from glob import glob

package_name = 'coffee_head'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'camera_node = coffee_head.camera_node:main',
            'head_tracking = coffee_head.head_tracking:main',
            'eye_tracking = coffee_head.eye_tracking:main',
            'face_recognition = coffee_head.face_recognition_node:main',
            'read_write_coffee = coffee_head.read_write_coffee:main',
            'coffee_dispenser_node = coffee_head.coffee_dispenser_node:main',
        ],
    },
)
