from setuptools import setup, find_packages
import os
from glob import glob

package_name = "carla_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ASU RT",
    maintainer_email="dev@asurt.eng.asu.edu",
    description="CARLA ROS 2 telemetry",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"carla_telemetry_node = {package_name}.node:main",
            f"carla_manual_control = {package_name}.manual_control:main",
        ],
    },
)
