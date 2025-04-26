from setuptools import find_packages, setup

package_name = 'behavior_nodes'

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
    maintainer='vr-workstation-2',
    maintainer_email='Marcus.Arnett10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'language_model_processor_node = behavior_nodes.language_model_processor_node.node:main',
        ],
    },
)