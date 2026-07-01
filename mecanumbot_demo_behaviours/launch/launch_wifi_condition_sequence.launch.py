import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_wifi_ssid():
    # Prefer nmcli if available
    try:
        out = subprocess.check_output([
            "nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"
        ], stderr=subprocess.DEVNULL, text=True, timeout=2)
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass

    # Fallback to iwgetid
    try:
        out = subprocess.check_output(["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True, timeout=2)
        return out.strip() if out.strip() else None
    except Exception:
        pass

    return None


def choose_default_param_file(ssid, pkg_dir):
    if ssid == "MecanumNet":
        return os.path.join(pkg_dir, "config", "behaviour_setting_constants.yaml")
    if ssid == "MecanumetoNet":
        return os.path.join(pkg_dir, "config", "Eto_behaviour_setting_constants.yaml")
    # Fallback
    return os.path.join(pkg_dir, "config", "behaviour_setting_constants.yaml")


def generate_launch_description():
    pkg_dir = get_package_share_directory('mecanumbot_demo_behaviours')
    ssid = get_wifi_ssid()
    default_params_path = choose_default_param_file(ssid, pkg_dir)

    params = LaunchConfiguration("params")
    yaml_path = LaunchConfiguration("yaml_path")
    namespace = LaunchConfiguration("namespace")
    condition = LaunchConfiguration("condition")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=default_params_path,
            description="YAML file with all constant parameters (override will bypass SSID detection)"
        ),
        DeclareLaunchArgument(
            "yaml_path",
            default_value=default_params_path,
            description="YAML path for behaviour tree nodes (env var or CLI arg override)"
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the behaviour node"
        ),
        DeclareLaunchArgument(
            "condition",
            default_value="Doglike",
            description="Behaviour condition: Doglike / Control / LED"
        ),

        LogInfo(msg=['Detected Wi-Fi SSID: ', ssid]),
        LogInfo(msg=['Using params: ', params]),
        LogInfo(msg=['Using behaviour YAML: ', yaml_path]),
    
        DeclareLaunchArgument(
            "yaml_path",
            default_value=default_params_path,
            description="YAML path for behaviour tree nodes (env var or CLI arg override)"
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the behaviour node"
        ),
        DeclareLaunchArgument(
            "condition",
            default_value="Doglike",
            description="Behaviour condition: Doglike / Control / LED"
        ),

        SetEnvironmentVariable(
            name="YAML_PATH",
            value=yaml_path
        ),
        SetEnvironmentVariable(
            name="BEHAVIOUR_YAML_PATH",
            value=yaml_path
        ),

        Node(
            package="mecanumbot_demo_behaviours",
            executable="doglike_demo_bt_node",
            name="doglike_demo_bt_node",
            namespace=namespace,
            output="screen",
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos')],
            parameters=[params],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'Doglike'"]))
        ),

        Node(
            package="mecanumbot_demo_behaviours",
            executable="control_demo_bt_node",
            name="control_demo_bt_node",
            namespace=namespace,
            output="screen",
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos')],
            parameters=[params],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'Control'"]))
        ),

        Node(
            package="mecanumbot_demo_behaviours",
            executable="LED_demo_bt_node",
            name="LED_demo_bt_node",
            namespace=namespace,
            output="screen",
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos')],
            parameters=[params],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'LED'"]))
        ),
    ])
