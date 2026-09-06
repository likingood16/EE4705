from glob import glob
from setuptools import find_packages, setup


package_name = "ee4705_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EE4705 Project Team",
    maintainer_email="replace-with-team-email@example.com",
    description="Launch and system-check tools for the EE4705 TurtleBot3 project.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "system_check = ee4705_bringup.system_check:main",
        ],
    },
)
