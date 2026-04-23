from setuptools import find_packages, setup

package_name = "sanad_eye_tracking"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/eye_tracking.launch.py"]),
        ("share/" + package_name + "/config", ["config/eye_tracking_params.yaml",
                                                  "config/face_landmarker.task"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="zero",
    maintainer_email="mohmed.ayman11@gmail.com",
    description="Eye-tracking gaze navigation for SANAD wheelchair",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "eye_tracker = sanad_eye_tracking.eye_tracker_node:main",
            "gaze_controller = sanad_eye_tracking.gaze_controller_node:main",
        ],
    },
)
