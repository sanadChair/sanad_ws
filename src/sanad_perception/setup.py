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
            # "lidar_merger    = sanad_perception.lidar_merger:main",
            'lidar_detector  = sanad_perception.human_follower.lidar_cluster_detector:main',
            'yolo_detector   = sanad_perception.human_follower.yolo_depth_detector:main',
            'tracker         = sanad_perception.human_follower.tracker_node:main',
            'lidar_camera_fusion_3d    = sanad_perception.sensor_fusion.lidar_camera_fusion_3d:main',
            'lidar_camera_fusion_2d    = sanad_perception.sensor_fusion.lidar_camera_fusion_2d:main',
        ],
    },
)
