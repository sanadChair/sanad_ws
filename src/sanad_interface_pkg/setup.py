from glob import glob
from setuptools import find_packages, setup

package_name = "sanad_interface_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/srv", glob("srv/*.srv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="youssef, zero",
    maintainer_email="yousefkamel282@gmail.com, mohmed.ayman11@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "keyboard = sanad_interface_pkg.keyboard:main",
            "keyboard_twist = sanad_interface_pkg.keyboard_twist:main",
            "ps3_twist = sanad_interface_pkg.ps3_twist:main",
            "human_follower= sanad_interface_pkg.human_follower:main",
            "imu_to_cmdvel = sanad_interface_pkg.imu_twist:main",
        ],
    },
)
