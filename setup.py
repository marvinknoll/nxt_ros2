import os
from glob import glob
from setuptools import setup

package_name = "nxt_ros2"
nxt_ros2_util = "nxt_ros2/util"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, nxt_ros2_util],
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            glob(os.path.join("resource", package_name)),
        ),
        (
            os.path.join("share", package_name),
            glob(os.path.join("package.xml")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marvin",
    maintainer_email="marvin.knoll@protonmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nxt_ros = nxt_ros2.nxt_ros:main",
            "js_aggregator = nxt_ros2.joint_state_aggregator:main",
            "diff_drive_controller = nxt_ros2.diff_drive_controller:main",
            "nxt_teleop = nxt_ros2.nxt_teleop:main",
            "odometry = nxt_ros2.odometry:main",
        ],
    },
)
