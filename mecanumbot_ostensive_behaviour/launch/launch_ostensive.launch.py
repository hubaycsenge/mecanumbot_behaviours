"""
Launch the ostensive behaviour tree, picking its constants from the Wi-Fi SSID.

The SSID selects the room, and the room selects the YAML -- the same convention
`mecanumbot_bringup` and the leading behaviour launcher use, so all three agree
about which space the robot thinks it is in without anybody having to remember a
launch argument.

The tree runs the behaviour **and starts the perception it reads**, which the
base launch no longer does. It needs the **pose** detector specifically and not
merely "a camera detector": gestures are decoded from COCO-17 keypoints, and the
fetch detector -- the other option -- emits boxes and no skeleton at all.
`use_perception:=false` when it is already running.

`use_camera` defaults to false, so the DeepStream detector opens the camera
directly and there is no image topic. Set it true to publish
`/camera/image_raw/compressed` for a recording, at the cost of a JPEG encode and
decode per frame; the leading launcher does that by default.

Still needed, already running: nav2 with AMCL localized, for `/amcl_pose` and
the `/goal_pose` goals.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PERCEPTION_LAUNCH = os.path.join(
    get_package_share_directory("mecanumbot_sensorprocess_smart"),
    "launch",
    "perception.launch.py",
)


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
            DeclareLaunchArgument(
                "use_perception",
                default_value="true",
                description=(
                    "Start the perception pipeline this tree reads. The base "
                    "launch no longer does; set false only when it is already up"
                ),
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description=(
                    "Feed the detector from /camera/image_raw/compressed instead "
                    "of letting it open the camera directly. Does NOT start the "
                    "publisher: run camera_compressed.launch.py first. The "
                    "camera can only be opened once"
                ),
            ),
            DeclareLaunchArgument(
                "camera_width", default_value="1280", description="Frame width"
            ),
            DeclareLaunchArgument(
                "camera_height", default_value="720", description="Frame height"
            ),
            DeclareLaunchArgument(
                "yolo_imgsz",
                default_value="1280",
                description="Input size the pose model was exported at",
            ),
            DeclareLaunchArgument(
                "yolo_model",
                default_value="yolo26m-pose",
                description="Pose model stem inside models/imgsz_<yolo_imgsz>/",
            ),
            LogInfo(msg=["Detected Wi-Fi SSID: ", str(ssid)]),
            LogInfo(msg=["Using ostensive YAML: ", yaml_path]),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            # The POSE detector, and not just any camera detector: this tree
            # reads gestures off COCO-17 keypoints, which the fetch detector
            # does not produce at all.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PERCEPTION_LAUNCH),
                condition=IfCondition(LaunchConfiguration("use_perception")),
                launch_arguments={
                    "namespace": namespace,
                    "detector": "pose",
                    "use_camera": LaunchConfiguration("use_camera"),
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "yolo_imgsz": LaunchConfiguration("yolo_imgsz"),
                    "yolo_model": LaunchConfiguration("yolo_model"),
                }.items(),
            ),
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
