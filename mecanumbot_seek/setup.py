from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_seek"

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
        "Seeking behaviour tree modelled on Panksepp's SEEKING circuit: "
        "watch for the object while searching where it was."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "seek_bt_node = mecanumbot_seek.tree_nodes.seek_tree:main",
        ],
    },
)
