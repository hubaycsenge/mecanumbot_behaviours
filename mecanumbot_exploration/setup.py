import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_exploration"


def share_files(pattern):
    """Return only regular files matching pattern (skips __pycache__ etc.)."""
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
        "The m-explore-ros2 based exploration pass: uncertainty monitoring "
        "and finish detection on top of explore_lite frontier navigation."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "exploration_node = "
            "mecanumbot_exploration.tree_nodes.exploration_node:main",
            "exploration_preflight = "
            "mecanumbot_exploration.tree_nodes.preflight_node:main",
        ],
    },
)
