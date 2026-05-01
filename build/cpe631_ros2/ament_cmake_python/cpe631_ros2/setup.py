from setuptools import find_packages
from setuptools import setup

setup(
    name='cpe631_ros2',
    version='0.1.0',
    packages=find_packages(
        include=('cpe631_ros2', 'cpe631_ros2.*')),
)
