from setuptools import setup
import os
from glob import glob

package_name = 'pull_wire_encoder'

setup(
    name=package_name,
    version='0.0.0',
    # This automatically finds "pull_wire_encoder" directory as the package
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install the config file to share/pull_wire_encoder/config/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install the message definition if needed (optional based on your structure)
        # (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='user@todo.todo',
    description='Pull wire encoder driver and publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Maps "ros2 run pull_wire_encoder fork_publisher" 
            # to src/pull_wire_encoder/pull_wire_encoder/fork_publisher.py
            'fork_publisher = pull_wire_encoder.fork_publisher:main',
        ],
    },
)