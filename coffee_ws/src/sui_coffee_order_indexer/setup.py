from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sui_coffee_order_indexer'
package_dir = os.path.dirname(os.path.abspath(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join(package_dir, 'resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(package_dir, 'launch', '*.launch.py'))),
        # Include Prisma schema
        (os.path.join('share', package_name, 'prisma'),
            glob(os.path.join(package_dir, 'prisma', '*.prisma'))),
    ],
    install_requires=[
        'setuptools',
        'prisma',  # Prisma Client Python
        'sui-py>=0.1.0',  # New Sui Python SDK
        'python-dotenv',  # For environment configuration
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='A ROS2 package for indexing coffee club events from the Sui blockchain',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'indexer_node = sui_coffee_order_indexer.indexer_node:main',
            'prisma_setup = scripts.setup_prisma:main',
        ],
    },
)
