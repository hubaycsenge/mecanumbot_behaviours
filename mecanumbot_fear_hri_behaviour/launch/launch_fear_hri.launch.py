"""Launch Fear HRI tree for sim or robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mecanumbot_fear_hri_behaviour")
    default_yaml = os.path.join(pkg_dir, "config", "fear_hri_constants.yaml")
    yaml_path = LaunchConfiguration("yaml_path")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument("yaml_path", default_value=default_yaml),
        DeclareLaunchArgument("namespace", default_value="mecanumbot"),
        SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
        SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
        Node(
            package="mecanumbot_fear_hri_behaviour",
            executable="fear_hri_bt_node",
            name="fear_hri_bt_node",
            namespace=namespace,
            output="screen",
            remappings=[
                ("/mecanumbot/cmd_vel", "/cmd_vel"),
            ],
        ),
    ])
