from glob import glob
import os

from setuptools import find_packages, setup


package_name = "encoder_driver"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="saads",
    maintainer_email="saads@todo.todo",
    description="SAADS wheel odometry, ESP32 motor bridge, and web dashboard.",
    license="TODO",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "wheel_odometry = encoder_driver.wheel_odometry_node:main",
            "motor_dashboard = encoder_driver.motor_dashboard_node:main",
        ],
    },
)
