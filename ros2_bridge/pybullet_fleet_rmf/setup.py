from glob import glob

from setuptools import setup

package_name = "pybullet_fleet_rmf"

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
    description="Open-RMF integration for PyBulletFleet — fleet adapter, door/lift/workcell handlers",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_adapter = pybullet_fleet_rmf.fleet_adapter:main",
        ],
    },
)
