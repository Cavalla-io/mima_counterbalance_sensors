from setuptools import find_packages, setup

package_name = 'turn_assist'

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
    maintainer='cavallatestbench3',
    maintainer_email='cavallatestbench3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'assister = turn_assist.assister:main',
            'wheel_tracker = turn_assist.wheel_tracker:main'
        ],
    },
)
