"""
Launch the ostensive behaviour tree, picking its constants from the Wi-Fi SSID.

The SSID selects the room, and the room selects the YAML -- the same convention
`mecanumbot_bringup` and the leading behaviour launcher use, so all three agree
about which space the robot thinks it is in without anybody having to remember a
launch argument.

The tree only runs the behaviour. It needs, already running:

* the people-detection pipeline, for `cam_people_detections` and `people_fusion`
  (`ros2 launch mecanumbot_sensorprocess_smart mecanumbot_peopledetect.launch.py`);
* nav2 with AMCL localized, for `/amcl_pose` and the `/goal_pose` goals.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_wifi_ssid():
    """Return the SSID of the active Wi-Fi connection, or None."""
    # Prefer nmcli if available
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
        )
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass

    # Fallback to iwgetid
    try:
        out = subprocess.check_output(
            ["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True, timeout=2
        )
        return out.strip() if out.strip() else None
    except Exception:
        pass

    return None


def choose_default_param_file(ssid, pkg_dir):
    """Return the constants YAML that goes with an SSID, defaulting to AI_dept."""
    if ssid == "MecanumetoNet":
        return os.path.join(pkg_dir, "config", "Eto_ostensive_setting_constants.yaml")
    return os.path.join(pkg_dir, "config", "ostensive_setting_constants.yaml")


def generate_launch_description():
    """Build the launch description for the ostensive tree."""
    pkg_dir = get_package_share_directory("mecanumbot_ostensive_behaviour")
    ssid = get_wifi_ssid()
    default_params_path = choose_default_param_file(ssid, pkg_dir)

    params = LaunchConfiguration("params")
    yaml_path = LaunchConfiguration("yaml_path")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params_path,
                description="YAML with the ostensive constants (overrides SSID detection)",
            ),
            DeclareLaunchArgument(
                "yaml_path",
                default_value=default_params_path,
                description="YAML path handed to the tree through YAML_PATH",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="mecanumbot",
                description="Namespace for the behaviour node",
            ),
            LogInfo(msg=["Detected Wi-Fi SSID: ", str(ssid)]),
            LogInfo(msg=["Using ostensive YAML: ", yaml_path]),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            Node(
                package="mecanumbot_ostensive_behaviour",
                executable="ostensive_bt_node",
                name="ostensive_bt_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("/mecanumbot/cmd_vel", "/cmd_vel"),
                    ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
                parameters=[params],
            ),
        ]
    )
