from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coffee_expressions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['coffee_expressions', 'coffee_expressions.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'expressive_eyes = coffee_expressions.expressive_eyes:main',
            'plaipin_expressive_eyes = coffee_expressions.plaipin_expressive_eyes:main',
        ],
    },
)
