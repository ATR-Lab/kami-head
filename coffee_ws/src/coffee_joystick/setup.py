from setuptools import setup

package_name = 'coffee_joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Joystick control for Coffee Buddy robot head',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'ros2launch.launch_file_data': [],
        'console_scripts': [
            'joystick_control = coffee_joystick.joystick_control_node:main'
        ],
    },
    python_executable='/home/kpatch/Github/coffee-budy/coffee_buddy_venv/bin/python3'
)
