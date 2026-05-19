from setuptools import setup

package_name = "cfpa2_peer_coordination"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emily Lau",
    maintainer_email="zcabyl3@ucl.ac.uk",
    description="Decentralised peer-to-peer frontier negotiation for Go2 exploration",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "peer_coordinator_node = cfpa2_peer_coordination.peer_coordinator_node:main",
        ],
    },
)