r"""
Launch the fetch behaviour tree, picking its constants from the Wi-Fi SSID.

The SSID selects the room and the room selects the YAML -- the same convention
`mecanumbot_bringup` and the leading, ostensive and seek launchers use, so all
of them agree about which space the robot thinks it is in without anybody having
to remember a launch argument.

The tree only runs the behaviour. It needs, already running:

* **the fetch camera detector**, which is the one that finds balls at all. It is
  off by default in the base launch because it replaces the pose detector rather
  than joining it::

      ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py \\
          use_fetch_detector:=true use_pose_detector:=false

  A pose network has exactly one class, so there is no threshold at which it
  starts finding tennis balls; and two networks on one camera stream is most of
  an Orin Nano's GPU. The trade is that the ostensive gestures are unavailable
  while this is the detector in use.
* **`mecanumbot_locate_detections`**, part of the same base launch, which turns
  the ball boxes into `/mecanumbot/ball_detections` in the map frame. This is
  where the ball's range and height come from; the LiDAR cannot see a tennis
  ball at all.
* **nav2 with AMCL localized against the room's map**, for `/amcl_pose` and the
  `navigate_to_pose` action. Every drive in this tree is a nav2 goal.

Watch a round with::

    ros2 topic echo /mecanumbot/fetch/state
    ros2 topic echo /mecanumbot/ball_fusion
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
        return os.path.join(pkg_dir, "config", "Eto_fetch_setting_constants.yaml")
    return os.path.join(pkg_dir, "config", "fetch_setting_constants.yaml")


def generate_launch_description():
    """Build the launch description for the fetch tree."""
    pkg_dir = get_package_share_directory("mecanumbot_fetch_behaviour")
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
                description="YAML with the fetch constants (overrides SSID detection)",
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
            LogInfo(msg=["Using fetch YAML: ", yaml_path]),
            LogInfo(
                msg=(
                    "the ball comes from /mecanumbot/ball_detections -- start the "
                    "base launch with use_fetch_detector:=true or nothing will "
                    "ever be seen"
                )
            ),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            Node(
                package="mecanumbot_fetch_behaviour",
                executable="fetch_bt_node",
                name="fetch_bt_node",
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
