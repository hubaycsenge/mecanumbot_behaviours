from setuptools import find_packages, setup
from glob import glob

package_name = "mecanumbot_ostensive_behaviour"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Csenge Hubay",
    maintainer_email="csengehubay@gmail.com",
    description=(
        "Ostensive behaviour tree: be addressed by gesture, "
        "read a direction cue, drive there."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ostensive_bt_node = mecanumbot_ostensive_behaviour.tree_nodes.ostensive_tree:main",
        ],
    },
)
