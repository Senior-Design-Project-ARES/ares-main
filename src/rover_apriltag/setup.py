from glob import glob
from setuptools import setup

package_name = "rover_apriltag"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/calibration", glob("calibration/*")),
    ],
    entry_points={
        "console_scripts": [
            "target_locator = rover_apriltag.target_locator_node:main",
        ],
    },
)
