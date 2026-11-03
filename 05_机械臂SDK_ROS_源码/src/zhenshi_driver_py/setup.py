from glob import glob
import os

from setuptools import find_packages, setup


package_name = "zhenshi_driver_py"

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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="XuK",
    maintainer_email="todo@example.com",
    description="ROS 2 Python driver for the Zhenshi controller SDK",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "driver_node = zhenshi_driver_py.driver_node:main",
        ],
    },
)

