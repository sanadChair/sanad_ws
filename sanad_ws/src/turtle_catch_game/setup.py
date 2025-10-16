import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_catch_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This line is crucial for the package to be found by ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # This line includes the package.xml file
        ('share/' + package_name, ['package.xml']),
        # This line includes the launch file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bod',
    maintainer_email='bod@todo.todo',
    description='The turtle catch game package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawner = turtle_catch_game.turtle_spawner:main",
            "turtle_controller = turtle_catch_game.turtle_controller:main"
        ],
    },
)