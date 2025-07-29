from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_voice_agent_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*.ui')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='PyQt-based monitoring dashboard for Coffee Voice Agent system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_agent_monitor = coffee_voice_agent_ui.voice_agent_monitor_app:main',
            'voice_agent_rqt = coffee_voice_agent_ui.voice_agent_monitor:main',
        ],
    },
)
