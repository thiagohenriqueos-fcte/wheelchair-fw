from glob import glob

from setuptools import find_packages, setup

package_name = "wheelchair_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wheelchair Team",
    maintainer_email="vitorg.a.s@hotmail.com",
    description="ROS 2 semi-assisted control layer for the wheelchair firmware.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "esp_bridge = wheelchair_ros.esp_bridge_node:main",
            "shared_control = wheelchair_ros.shared_control_node:main",
        ],
    },
)
