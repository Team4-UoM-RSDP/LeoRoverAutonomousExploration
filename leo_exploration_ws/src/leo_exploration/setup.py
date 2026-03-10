from setuptools import find_packages, setup
import os
from glob import glob

package_name = "leo_exploration"

setup(
    name=package_name,
<<<<<<< HEAD
    version="2.6.0",
=======
    version="2.4.0",
>>>>>>> parent of 7088f60 (fix: prevent LiDAR wall penetration via range reduction and thicker walls)
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*")),
        (os.path.join("share", package_name, "worlds"),
         glob("worlds/*.world") + glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "urdf"),
         glob("urdf/*.urdf") + glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "scripts"),
         glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="You",
    maintainer_email="you@example.com",
<<<<<<< HEAD
    description="Frontier-based autonomous exploration for Leo Rover (ROS2 Jazzy) v2.6",
=======
    description="Frontier-based autonomous exploration for Leo Rover (ROS2 Jazzy) v2.4",
>>>>>>> parent of 7088f60 (fix: prevent LiDAR wall penetration via range reduction and thicker walls)
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_explorer = leo_exploration.frontier_explorer:main",
        ],
    },
)
