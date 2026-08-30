import os
from glob import glob

from setuptools import find_packages, setup

package_name = "core_damiao_imu"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="curious",
    maintainer_email="curious.ks.jp@gmail.com",
    description="ROS 2 USB serial driver for the Damiao DM-IMU-L1.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "damiao_imu_node = core_damiao_imu.node:main",
        ],
    },
)
