import os
from glob import glob
from setuptools import setup

package_name = 'cfe_plugin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swhart',
    maintainer_email='swhart@traclabs.com',
    description='BRASH cFS to ROS2 Python Bridge using Juicer-generated message structures',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={},
)
