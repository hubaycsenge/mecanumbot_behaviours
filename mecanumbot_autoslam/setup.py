import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_autoslam"


def share_files(pattern):
    """
    Return only the regular files matching pattern (skips __pycache__ etc.).

    A bare glob hands setuptools the `__pycache__` directory that appears the
    moment anything imports a launch file, and the install then fails with
    "can't copy ... doesn't exist or not a regular file".
    """
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
        "The autoslam pass (T1): the exploration behaviours that drive the "
        "robot around a place until it has been scanned, and the preflight "
        "that clears the graph before they start."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoslam_node = mecanumbot_autoslam.tree_nodes.autoslam_node:main",
            "autoslam_preflight = "
            "mecanumbot_autoslam.tree_nodes.preflight_node:main",
        ],
    },
)
