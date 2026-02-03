import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    param_file = LaunchConfiguration("params")
    namespace = LaunchConfiguration("namespace")

    leading_pkg_share_dir = get_package_share_directory('mecanumbot_leading_behaviour')
    param_path = os.path.join(leading_pkg_share_dir, 'config', "behaviour_setting_constants.yaml")

    return LaunchDescription([
        # Allow user to override the param file
        DeclareLaunchArgument(
            "params",
            default_value=param_path,
            description="YAML file with all constant parameters"
        ),

        # Allow user to set namespace
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the behaviour node"
        ),

        Node(
            package="mecanumbot_leading_behaviour",
            executable="doglike_leading_bt_node",
            name="doglike_leading_bt_node",
            namespace=namespace,
            output="screen",
            remappings= [('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos') ],
            parameters=[param_file]
        ),
    ])