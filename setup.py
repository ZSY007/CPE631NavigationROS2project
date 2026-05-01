from setuptools import find_packages, setup
from glob import glob
import os
from pathlib import Path

package_name = 'cpe631_ros2'

def package_files(base_dir, package_name):
    data_files = []
    base_path = Path(base_dir)
    for path in base_path.rglob('*'):
        if path.is_file():
            rel_dir = path.parent.relative_to(base_path).as_posix()
            install_dir = os.path.join('share', package_name, base_dir, rel_dir)
            data_files.append((install_dir, [str(path)]))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    + [(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))]
    + [(os.path.join('share', package_name, 'maps'), glob('maps/*'))]
    + [(os.path.join('share', package_name, 'worlds'), glob('worlds/*'))]
    + [(os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))]
    + [(os.path.join('share', package_name, 'param'), glob('param/*.yaml'))]
    + [(os.path.join('share', package_name, 'param'), glob('param/*.xml'))]
    + package_files('models', package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='ROS2 port of the CPE631 cafe simulation using Gazebo Sim (gz).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpe631_teleop = cpe631_ros2.teleop:main',
            'cpe631_peds = cpe631_ros2.peds:main',
            'cpe631_map_republisher = cpe631_ros2.map_republisher:main',
            'ped_pose_extractor = cpe631_ros2.ped_pose_extractor:main',
            'social_nav_node = cpe631_ros2.social_nav_node:main',
            'data_collector = cpe631_ros2.data_collector:main',
            'goal_sender = cpe631_ros2.goal_sender:main',
        ],
    },
)
