from glob import glob
import os

from setuptools import setup


package_name = "mis_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FDU Embodied MIS",
    maintainer_email="rm@todo.todo",
    description="RGB-D scene perception for the embodied minimally invasive surgery task.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "d405_perception = mis_perception.d405_perception:main",
        ],
    },
)
