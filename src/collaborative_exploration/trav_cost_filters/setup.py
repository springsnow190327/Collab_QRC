from setuptools import setup
import os
from glob import glob

package_name = "trav_cost_filters"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hanshang Zhu",
    maintainer_email="zhuhanshan12@outlook.com",
    description="grid_map cost layer → OccupancyGrid adapter for Nav2 + CFPA2.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grid_map_to_occupancy_grid = trav_cost_filters.grid_map_to_occupancy_grid:main",
        ],
    },
)
