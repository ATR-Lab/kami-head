from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_vision'

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
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='Computer vision package for camera capture, face detection, and basic visual processing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = coffee_vision.camera_node:main',
            'face_detection_node = coffee_vision.face_detection_node:main',
        ],
    },
)
