import os
from glob import glob

from setuptools import find_packages, setup

package_name = "compressed_pointcloud"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "DracoPy"],
    zip_safe=True,
    maintainer="ubicomp",
    maintainer_email="ubicomp@todo.todo",
    description="Draco-compress PointCloud2 into foxglove_msgs/CompressedPointCloud.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "compress_node = compressed_pointcloud.compress_node:main",
        ],
    },
)
