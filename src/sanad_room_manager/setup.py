from glob import glob
from setuptools import find_packages, setup

package_name = "sanad_room_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/rooms.yaml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="zero",
    maintainer_email="mohmed.ayman11@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "room_manager_node = sanad_room_manager.room_manager_node:main",
            "room_nav = sanad_room_manager.room_nav:main",
            "ir_manager_node = sanad_room_manager.ir_manager_node:main",
        ],
    },
)
