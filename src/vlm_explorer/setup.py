import os
from glob import glob

from setuptools import setup

package_name = "vlm_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.lua")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hz",
    maintainer_email="hz@example.com",
    description="VLM-in-the-loop dual-robot exploration system.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "skeleton_extractor_node = vlm_explorer.skeleton_extractor_node:main",
            "map_renderer_node = vlm_explorer.map_renderer_node:main",
            "green_marker_detector_node = vlm_explorer.green_marker_detector_node:main",
            "artifact_detector_node = vlm_explorer.artifact_detector_node:main",
            "interaction_tool_node = vlm_explorer.interaction_tool_node:main",
            "vlm_coordinator_node = vlm_explorer.vlm_coordinator_node:main",
            "florence2_detector_node = vlm_explorer.florence2_detector_node:main",
            "red_block_detector_node = vlm_explorer.red_block_detector_node:main",
            "yolo_detector_node = vlm_explorer.yolo_detector_node:main",
        ],
    },
)
