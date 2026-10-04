from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rm_rcm_web_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FDU Embodied MIS",
    maintainer_email="rm@todo.todo",
    description="Standalone browser joystick bridge for the RM65 RCM robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rcm_web_teleop = rm_rcm_web_teleop.server:main",
            "dual_rcm_web_teleop = rm_rcm_web_teleop.dual_server:main",
        ],
    },
)
