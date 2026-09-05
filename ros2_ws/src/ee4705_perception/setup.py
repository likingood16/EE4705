from setuptools import find_packages, setup


package_name = "ee4705_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"api": ["openai>=1.0"]},
    zip_safe=True,
    maintainer="EE4705 Project Team",
    maintainer_email="replace-with-team-email@example.com",
    description="Vision-language scene description and visual question answering.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "vision_demo = ee4705_perception.cli:main",
        ],
    },
)
