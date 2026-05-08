from setuptools import find_packages, setup

package_name = "slam_backend_adapters"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ylhc",
    maintainer_email="ylhc@example.com",
    description="Adapters for Swarm-LIO2, Dynamic-LIO filtering, and ERASOR cleanup.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "swarm_lio2_ros2_adapter_node = slam_backend_adapters.swarm_lio2_ros2_adapter_node:main",
            "dynamic_lio_filtering_node = slam_backend_adapters.dynamic_lio_filtering_node:main",
            "erasor_adapter_node = slam_backend_adapters.erasor_adapter_node:main",
        ],
    },
)
