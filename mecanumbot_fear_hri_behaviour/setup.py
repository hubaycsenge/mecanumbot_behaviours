from setuptools import find_packages, setup
from glob import glob

package_name = "mecanumbot_fear_hri_behaviour"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fahrad Namazov",
    maintainer_email="fahradnamazov@gmail.com",
    description="Fear HRI behaviour tree",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fear_hri_bt_node = mecanumbot_fear_hri_behaviour.tree_nodes.fear_hri_tree:main",
        ],
    },
)
