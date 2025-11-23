from setuptools import setup
import os
from glob import glob

package_name = 'sanad_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mazen',
    maintainer_email='mazen@example.com',
    description='SLAM automation node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = sanad_slam.slam_node:main',
        ],
    },
)
