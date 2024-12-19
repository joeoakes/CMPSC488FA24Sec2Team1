from setuptools import find_packages, setup
import os
from glob import glob

package_name = "run_nav2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nav2_proxy.launch.py"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goalpose = run_nav2.goalpose:main",
            "odom_base_tf = run_nav2.odom_base_tf:main",
            "odom_test_data = run_nav2.odom_test_data:main",
            "rtabodom = run_nav2.rtabodom:main",
            "maprelay = run_nav2.maprelay:main",
        ],
    },
)
