"""
Launch the seek behaviour tree (T2), picking its constants from the Wi-Fi SSID.

The SSID selects the room and the room selects the YAML -- the same convention
`mecanumbot_bringup` and the leading and ostensive launchers use, so all of them
agree about which space the robot thinks it is in without anybody having to
remember a launch argument.

The tree only runs the behaviour. It needs, already running:

* **nav2 with AMCL localized against the T1 map** -- for `/amcl_pose` and the
  `navigate_to_pose` action. This is the phase the robot uses its *own* 2D map,
  saved at the end of T1, because it is the more stable of the two frames.
  To have the point cloud's keepouts apply here too -- the table tops and low
  steps the lidar plane cannot see -- start it against
  `mecanumbot_description/param/mecanumbot_seek_nav2.yaml`::

      NAV2_PARAMS_FILE=$(ros2 pkg prefix mecanumbot_description)/share/\
      mecanumbot_description/param/mecanumbot_seek_nav2.yaml \
      ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py;
* **the Deep3R client**, if the cloud is to be updated during T2 as well. The
  second scan updates the first, so the camera stream keeps flowing;
* **whatever publishes `/mecanumbot/seek/target` and
  `/mecanumbot/seek/detections`** -- the server's answer and the live onboard
  detections. Without the first the tree waits and then gives up; without the
  second it will drive to where the object was and never see it.

Ask it for something with:

    ros2 topic pub --once /mecanumbot/seek/request std_msgs/String "{data: 'mug'}"

and watch the circuit with:

    ros2 topic echo /mecanumbot/seek/state
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
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
        return os.path.join(pkg_dir, "config", "Eto_seek_setting_constants.yaml")
    return os.path.join(pkg_dir, "config", "seek_setting_constants.yaml")


def generate_launch_description():
    """Build the launch description for the seek tree."""
    pkg_dir = get_package_share_directory("mecanumbot_seek")
    ssid = get_wifi_ssid()
    default_params_path = choose_default_param_file(ssid, pkg_dir)

    params = LaunchConfiguration("params")
    yaml_path = LaunchConfiguration("yaml_path")
    namespace = LaunchConfiguration("namespace")
    use_agreement = LaunchConfiguration("use_agreement")
    agreement_params = LaunchConfiguration("agreement_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params_path,
                description="YAML with the seek constants (overrides SSID detection)",
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
            DeclareLaunchArgument(
                "use_agreement",
                default_value="true",
                description=(
                    "Start the 2D/3D comparison handler. Set false when a T1 "
                    "session is still running one -- two publish the same mask "
                    "to the same topic, which works but is pointless."
                ),
            ),
            DeclareLaunchArgument(
                "agreement_params",
                default_value=os.path.join(
                    get_package_share_directory("mecanumbot_custom_nav2"),
                    "config",
                    "map_agreement.yaml",
                ),
                description="Constants for the comparison handler",
            ),
            LogInfo(msg=["Detected Wi-Fi SSID: ", str(ssid)]),
            LogInfo(msg=["Using seek YAML: ", yaml_path]),
            LogInfo(
                msg=(
                    "ask for something with: ros2 topic pub --once "
                    "/mecanumbot/seek/request std_msgs/String \"{data: 'mug'}\""
                )
            ),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            Node(
                package="mecanumbot_custom_nav2",
                executable="mecanumbot_map_agreement_node",
                name="mecanumbot_map_agreement",
                output="screen",
                condition=IfCondition(use_agreement),
                parameters=[agreement_params],
            ),
            Node(
                package="mecanumbot_seek",
                executable="seek_bt_node",
                name="seek_bt_node",
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
