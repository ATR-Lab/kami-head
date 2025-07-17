from setuptools import find_packages, setup

package_name = 'coffee_llm_processor'

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
    maintainer_email='marcus@opendive.io',
    description='LLM processing package for Coffee Buddy robot system, providing conversational AI and response generation capabilities',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'language_model_processor_node = coffee_llm_processor.language_model_processor_node.node:main',
        ],
    },
)