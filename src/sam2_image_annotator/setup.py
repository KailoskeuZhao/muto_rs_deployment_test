import os
from glob import glob

from setuptools import setup


package_name = "sam2_image_annotator"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="a98165809@gmail.com",
    description="ROS 2 image annotation node backed by SAM 2 image prediction.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sam2_image_annotator_node = sam2_image_annotator.sam2_image_annotator_node:main",
            "object_centroid_recorder_node = sam2_image_annotator.object_centroid_recorder_node:main",
        ],
    },
)
