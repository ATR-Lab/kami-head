from setuptools import setup
import os
from glob import glob

package_name = 'coffee_expressions_test_ui'

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
    maintainer_email='irvsteve@gmail.com',
    description='Qt-based UI for testing coffee expressions',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'expressions_test_ui = coffee_expressions_test_ui.expressions_test_ui:main',
        ],
    },
)
