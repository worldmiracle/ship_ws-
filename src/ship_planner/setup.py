import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ship_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('ship_planner/launch_*.py')),
        # RViz2 config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='douhouqi',
    maintainer_email='douhouqi@todo.todo',
    description='Ship autonomous driving path planning package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner = ship_planner.path_planner_node:main',
            'map_generator = ship_planner.map_generator:main',
            'simple_map_server = ship_planner.simple_map_server:main',
        ],
    },
)
