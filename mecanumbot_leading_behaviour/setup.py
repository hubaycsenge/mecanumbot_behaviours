from setuptools import find_packages, setup
from glob import glob

package_name = 'mecanumbot_leading_behaviour'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  glob('launch/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Csenge Hubay',
    maintainer_email='csengehubay@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_leading_bt_node = mecanumbot_leading_behaviour.py_trees.ctrl_tree:main',
            'doglike_leading_bt_node = mecanumbot_leading_behaviour.py_trees.dog_tree:main',
            'LED_leading_bt_node = mecanumbot_leading_behaviour.py_trees.LED_tree:main'
        ],
    },
)
