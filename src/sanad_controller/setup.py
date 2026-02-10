from setuptools import find_packages, setup

package_name = "sanad_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/controller.launch.py"]),
        ("share/" + package_name + "/config", ["config/sanad_controllers.yaml",
                                                 "config/joy_teleop.yaml",
                                                 "config/joy_config.yaml",
                                                 "config/twist_mux_joy.yaml",
                                                 "config/twist_mux_locks.yaml",
                                                 "config/twist_mux_topics.yaml",
                                                 ]),
        ("share/" + package_name + "/launch", ["launch/controller.launch.py",
                                                 "launch/joystick_teleop.launch.py"]),
    ],
    install_requires=[
        "setuptools",
        # Runtime deps used by the node (available via apt in ROS, but keep for completeness)
        # "numpy",
        # "tf-transformations",
    ],
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
            "noisy = sanad_controller.noisy_controller:main",
            "noisy_controller = sanad_controller.noisy_controller:main",
            "twist_relay = sanad_controller.twist_relay:main",
        ],
    },
)
