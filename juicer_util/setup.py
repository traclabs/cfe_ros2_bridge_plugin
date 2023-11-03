import os
from glob import glob
from setuptools import setup

package_name = 'juicer_util'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmilam',
    maintainer_email='tmilam@traclabs.com',
    description='Juicer utility used by other packages to parse sqlite files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'generate_juicer_database = juicer_util.generate_juicer_database:main',
        ],
    },
)
