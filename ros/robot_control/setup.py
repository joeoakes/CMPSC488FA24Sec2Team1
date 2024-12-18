from setuptools import find_packages, setup

package_name = "robot_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team1",
    maintainer_email="HoangNKNguyen@proton.me",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist_control = robot_control.twist_control:main",
            "robot_control = robot_control.control:main",
        ],
    },
)
