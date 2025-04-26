from setuptools import find_packages, setup

package_name = 'head_motion_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='Server for playing expression-based head motions',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_server = head_motion_server.motion_server_node:main',
        ],
    },
)
