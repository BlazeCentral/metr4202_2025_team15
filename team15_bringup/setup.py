import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'team15_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This is the crucial part: install launch and config files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 15',
    maintainer_email='blaise.delforce@outlook.com',
    description='Launch and configuration package for the METR4202 project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # This package has no executable nodes, so entry_points is empty
    entry_points={
        'console_scripts': [],
    },
)

