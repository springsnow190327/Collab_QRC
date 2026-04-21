from setuptools import find_packages, setup

package_name = "door_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/door_task.yaml", "config/scene.yaml"]),
        (
            "share/" + package_name + "/prompts",
            ["door_task/prompts/planner.md", "door_task/prompts/executer.md"],
        ),
    ],
    include_package_data=True,
    package_data={"door_task.prompts": ["*.md"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hz",
    maintainer_email="hz@example.com",
    description="Dual-robot door task (VLM controller + archived FSM path).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vlm_controller_node = door_task.ros.controller:main",
            "perception_node = door_task.ros.perception_node:main",
            "door_monitor_node = door_task.door_monitor_node:main",
            "button_monitor_node = door_task.button_monitor_node:main",
            "door_lock_from_button_node = door_task.door_lock_from_button_node:main",
        ],
    },
)
