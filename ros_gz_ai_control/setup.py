import os
from glob import glob
from setuptools import setup

package_name = "ros_gz_ai_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        # (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # (os.path.join("share", package_name, "models"), glob("models/*.xacro")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name), glob("gui.config")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Choi Woen-Sug",
    maintainer_email="woensug.choi@gmail.com",
    description="VLLM AI Control of ASV",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_broadcaster = ros_gz_ai_control.tf_broadcaster:main",
        ],
    },
)
