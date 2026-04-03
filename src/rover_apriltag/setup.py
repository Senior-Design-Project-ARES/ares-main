from setuptools import setup
import os
from glob import glob

package_name = "rover_apriltag"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Using glob handles all files in these folders automatically
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ares",
    maintainer_email="ares@todo.todo",
    description="AprilTag detection and fusion for rover navigation",
    license="TODO: License declaration",
    # --- TESTING HOOKS ---
    tests_require=["pytest"],
    test_suite="test",
    # ---------------------
    entry_points={
        "console_scripts": [
            "tag_fuser = rover_apriltag.tag_fuser_node:main",
        ],
    },
)