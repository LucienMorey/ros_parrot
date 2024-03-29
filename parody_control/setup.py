import os
import glob

from setuptools import setup

package_name = "parody_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lucien",
    maintainer_email="lucien@spacemachines.co",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "parody_controller = parody_control.parody_controller:main",
            "zero_left_leg = parody_control.zero_joints:zero_left_leg",
            "zero_right_leg = parody_control.zero_joints:zero_right_leg",
            "zero_neck = parody_control.zero_joints:zero_neck",
            "zero_tail = parody_control.zero_joints:zero_tail",
        ],
    },
)
