import os
from glob import glob
from setuptools import setup

package_name = 'metr4202_2025_team15'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This is the crucial part for installing your launch and config files.
        # It tells colcon to copy all .py files from the 'launch' directory,
        # and all .yaml files from the 'config' directory.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 15',
    maintainer_email='team15@student.uq.edu.au',
    description='METR4202 Team 15 project package for autonomous exploration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # This is where you make your Python scripts executable.
    # This is the step from Prac 4 (g).
    # The format is 'executable_name = package_name.python_file_name:main'
    entry_points={
        'console_scripts': [
            'explore_nav = metr4202_2025_team15.explore_nav:main',
            # 'aruco_detect_publish = metr4202_2025_team15.aruco_detect_publish:main', # Add this when your aruco node is ready
        ],
    },
)


