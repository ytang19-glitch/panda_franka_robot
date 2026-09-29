import os
from glob import glob

from setuptools import find_packages, setup

package_name = "panda_nmpc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yujietang",
    maintainer_email="ytang19@ualberta.ca",
    description="Read-only Panda trajectory tracking scaffold for future NMPC",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "nmpc_node = panda_nmpc.nmpc_node:main",
            "reference_bridge = panda_nmpc.bridge:main",
        ],
    },
)
