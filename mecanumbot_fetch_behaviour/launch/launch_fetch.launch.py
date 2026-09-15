r"""
Launch the fetch behaviour tree, picking its constants from the Wi-Fi SSID.

The SSID selects the room and the room selects the YAML -- the same convention
`mecanumbot_bringup` and the leading, ostensive and seek launchers use, so all
of them agree about which space the robot thinks it is in without anybody having
to remember a launch argument.

The tree runs the behaviour **and starts the perception it reads**, which the
base launch no longer does. It needs the **fetch** detector specifically: a pose
network has exactly one class, so there is no threshold at which it starts
finding tennis balls, and the fetch detector is the only one that produces
`cam_ball_boxes` for `mecanumbot_locate_detections` to place. The trade is that
the ostensive gestures are unavailable while it is the detector in use, and it
is why two behaviours cannot share one perception pipeline.
`use_perception:=false` when it is already running.

The LiDAR cannot see a tennis ball at all -- it scans one horizontal plane about
0.12 m up -- so both the ball's range and its height come from the camera, in
`mecanumbot_locate_detections`.

Still needed, already running: **nav2 with AMCL localized against the room's
map**, for `/amcl_pose` and the `navigate_to_pose` action. Every drive in this
tree is a nav2 goal.

Watch a round with::

    ros2 topic echo /mecanumbot/fetch/state
    ros2 topic echo /mecanumbot/ball_fusion

and see what the detector sees -- on by default here, ``debug_image:=false``
to save the per-frame JPEG encode::

    ros2 run rqt_image_view rqt_image_view \
        /mecanumbot/cam_object_detections/debug_image/compressed

People are drawn blue, balls yellow, and a refused box red with the check it
failed (score | size | shape | unconfirmed).
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
                "debug_image",
                default_value="true",
                description=(
                    "Publish the fetch detector's annotated frame on "
                    "/mecanumbot/cam_object_detections/debug_image/compressed"
                ),
            ),
            DeclareLaunchArgument(
                "camera_width", default_value="1280", description="Frame width"
            ),
            DeclareLaunchArgument(
                "camera_height", default_value="720", description="Frame height"
            ),
            DeclareLaunchArgument(
                "fetch_imgsz",
                default_value="640",
                description="Input size the fetch detector was exported at",
            ),
            DeclareLaunchArgument(
                "fetch_model",
                default_value="yolo26m",
                description="Detection model stem inside models/imgsz_<fetch_imgsz>/",
            ),
            LogInfo(msg=["Detected Wi-Fi SSID: ", str(ssid)]),
            LogInfo(msg=["Using fetch YAML: ", yaml_path]),
            LogInfo(
                msg=(
                    "the ball comes from /mecanumbot/ball_detections, published "
                    "by the perception pipeline this launch starts; with "
                    "use_perception:=false, start it yourself with "
                    "detector:=fetch or nothing will ever be seen"
                )
            ),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            # The FETCH detector, which is the only one that can see a ball --
            # and therefore the reason this game cannot run alongside the
            # ostensive one, which needs the skeletons this detector does not
            # produce.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PERCEPTION_LAUNCH),
                condition=IfCondition(LaunchConfiguration("use_perception")),
                launch_arguments={
                    "namespace": namespace,
                    "detector": "fetch",
                    "use_camera": LaunchConfiguration("use_camera"),
                    "debug_image": LaunchConfiguration("debug_image"),
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "fetch_imgsz": LaunchConfiguration("fetch_imgsz"),
                    "fetch_model": LaunchConfiguration("fetch_model"),
                }.items(),
            ),
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
