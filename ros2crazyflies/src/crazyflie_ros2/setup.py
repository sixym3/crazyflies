from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crazyflie_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eric',
    maintainer_email='xiao.yuan0217@gmail.com',
    description='ROS2 package for Crazyflie drone control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_node = crazyflie_ros2.crazyflie_node:main',
            'octomap_mapper_node = crazyflie_ros2.octomap_mapper_node:main',
            'autonomous_navigation_node = crazyflie_ros2.autonomous_navigation_node:main'
        ],
    },
)
