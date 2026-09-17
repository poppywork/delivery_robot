from glob import glob
from setuptools import find_packages, setup


package_name = "move_grasp_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="MoveGrasp Maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 move+grasp+place demo node using the GraspGenX pipeline for grasp poses.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_grasp_demo_node = move_grasp_demo.move_grasp_demo_node:main",
        ],
    },
)