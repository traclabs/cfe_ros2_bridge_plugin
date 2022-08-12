from setuptools import setup

package_name = 'juicer_util'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
