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
            'lidar_camera_fusion_3d    = sanad_perception.sensor_fusion.lidar_camera_fusion_3d:main',
            'lidar_camera_fusion_2d    = sanad_perception.sensor_fusion.lidar_camera_fusion_2d:main',
            'detector_tracker_node   = sanad_perception.human_follower.detector_tracker_node:main',
            'pose_estimator_node   = sanad_perception.human_follower.pose_estimator_node:main',
        ],
    },
)
