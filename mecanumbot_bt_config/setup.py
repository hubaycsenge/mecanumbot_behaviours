from setuptools import find_packages, setup

package_name = "mecanumbot_bt_config"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Csenge Hubay",
    maintainer_email="csengehubay@gmail.com",
    description=(
        "Constants YAML to py_trees blackboard, without a schema: "
        "the loader takes its key names from the file."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
