from setuptools import find_packages, setup

package_name = "core_status_gui"

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
    maintainer="ytk-wsl",
    maintainer_email="tyousinnsei.zangai01@gmail.com",
    description="Fullscreen ROS status display GUI.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "status_display_gui = core_status_gui.status_display_gui:main",
        ],
    },
)
