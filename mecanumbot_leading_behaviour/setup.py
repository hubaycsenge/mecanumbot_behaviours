import os
from setuptools import find_packages, setup
from glob import glob

package_name = "mecanumbot_leading_behaviour"


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
    description="TODO: Package description",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_leading_bt_node = mecanumbot_leading_behaviour.tree_nodes.ctrl_tree:main",
            "doglike_leading_bt_node = mecanumbot_leading_behaviour.tree_nodes.dog_tree:main",
            "LED_leading_bt_node = mecanumbot_leading_behaviour.tree_nodes.LED_tree:main",
            "bottom_up_tree_node = mecanumbot_leading_behaviour.tree_nodes.bottom_up_tree:main",
            "check_route = mecanumbot_leading_behaviour.tools.route_check:main",
        ],
    },
)
