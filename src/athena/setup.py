from setuptools import setup
from glob import glob
import os

package_name = "athena"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/**")),
        (os.path.join("share", package_name, "world"), glob("world/**")),
    ],
    install_requires=["setuptools", "promptulate~=1.10.0"],
    zip_safe=True,
    maintainer="Zeeland",
    maintainer_email="zeeland4work@gmail.com",
    description="A Mulitmodal Robot Agent based on ROS2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rotate_wheel= athena.rotate_wheel:main",
            "user_client= athena.user_client:main",
            "odom_client= athena.odom_client:main",
        ],
    },
)
