import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_object_choice_behaviour"


def share_files(pattern):
    """Return only the regular files matching pattern (skips __pycache__ etc.)."""
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", share_files("launch/*")),
        ("share/" + package_name + "/config", share_files("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Csenge Hubay",
    maintainer_email="csengehubay@gmail.com",
    description=(
        "Object-choice behaviour tree: a person points at one of several coloured "
        "bricks, and the robot goes to (or fetches) the one indicated."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_choice_bt_node = "
            "mecanumbot_object_choice_behaviour.tree_nodes.object_choice_tree:main",
        ],
    },
)
