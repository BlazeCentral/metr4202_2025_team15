from setuptools import find_packages, setup

package_name = 'team15_exploration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 15',
    maintainer_email='blaise.delforce@outlook.com',
    description='Contains the frontier exploration and navigation node for the METR4202 project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This line makes your Python script an executable
            'explore_nav = team15_exploration.explore_nav:main',
        ],
    },
)
