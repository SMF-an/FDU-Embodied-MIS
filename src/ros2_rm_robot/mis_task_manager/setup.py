from glob import glob
import os

from setuptools import setup


package_name = "mis_task_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
    description="Competition task manager, RCM executor, and monitoring dashboard.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_state_machine = mis_task_manager.task_state_machine:main",
            "rcm_task_executor = mis_task_manager.rcm_task_executor:main",
            "task_dashboard = mis_task_manager.task_dashboard:main",
        ],
    },
)
