from glob import glob

from setuptools import setup

package_name = "pybullet_fleet_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yu Okamoto",
    maintainer_email="yuokamoto1988@gmail.com",
    description="ROS 2 bridge for PyBulletFleet multi-robot simulation",
    license="Apache-2.0",
    tests_require=["pytest", "launch_testing"],
    entry_points={
        "console_scripts": [
            "bridge_node = pybullet_fleet_ros.bridge_node:main",
            "fleet_adapter = pybullet_fleet_ros.fleet_adapter:main",
            "door_adapter = pybullet_fleet_ros.door_adapter:main",
            "workcell_adapter = pybullet_fleet_ros.workcell_adapter:main",
        ],
    },
)
