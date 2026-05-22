from setuptools import find_packages, setup

package_name = "sanad_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="youssef",
    maintainer_email="yousefkamel282@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            'perception_node   = sanad_perception.human_follower.perception_node:main',
            'follow_node   = sanad_perception.human_follower.follow_node:main',
            'lidar_camera_fusion_2d    = sanad_perception.sensor_fusion.lidar_camera_fusion_2d:main',
            'calibration_node    = sanad_perception.sensor_fusion.calibration_node:main',
            'fusion_node    = sanad_perception.sensor_fusion.fusion_node:main',
            'fusion_bringup    = sanad_perception.sensor_fusion.fusion_bringup:main',
        ],
    },
)
