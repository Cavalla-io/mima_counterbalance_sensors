from setuptools import find_packages, setup

package_name = 'threat_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/threat_detector.launch.py']),
        ('share/' + package_name + '/models', ['models/yolov8m.pt']),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'torchvision',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'threat_detection_node = threat_detector.threat_detection_node:main',
        ],
    },
)
