from setuptools import setup

package_name = "cfpa2_collaborative_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/cfpa2_coordinator.yaml"]),
        ("share/" + package_name + "/config", ["config/cfpa2_single_robot.yaml"]),
    ],
    install_requires=["setuptools"],
    package_data={package_name: ["*.so"]},
    zip_safe=True,
    maintainer="hz",
    maintainer_email="hz@example.com",
    description="ROS2 CFPA2 coordinator node.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "cfpa2_coordinator_node = "
                "cfpa2_collaborative_autonomy.cfpa2_coordinator_node:main"
            ),
            (
                "cfpa2_single_robot_node = "
                "cfpa2_collaborative_autonomy.cfpa2_single_robot_node:main"
            ),
        ],
    },
)
