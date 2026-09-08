from glob import glob

from setuptools import find_packages, setup

package_name = "mecanumbot_fetch_behaviour"

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
        "Fetch behaviour tree: circle and sweep the head to find a tennis "
        "ball, grip it, and take it to the first person in sight."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fetch_bt_node = mecanumbot_fetch_behaviour.tree_nodes.fetch_tree:main",
        ],
    },
)
