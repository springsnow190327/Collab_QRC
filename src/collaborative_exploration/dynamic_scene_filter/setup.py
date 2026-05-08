from setuptools import find_packages, setup

package_name = "dynamic_scene_filter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/dynamic_filter.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ylhc",
    maintainer_email="ylhc@example.com",
    description="Temporal voxel dynamic object filtering for LiDAR team-SLAM.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dynamic_obstacle_filter_node = dynamic_scene_filter.dynamic_obstacle_filter_node:main",
            "dynamic_voxel_decay_map_node = dynamic_scene_filter.dynamic_voxel_decay_map_node:main",
            "dynamic_obstacle_injector_node = dynamic_scene_filter.dynamic_obstacle_injector_node:main",
        ],
    },
)
