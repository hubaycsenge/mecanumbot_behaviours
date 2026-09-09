"""
Launch a leading behaviour tree, picking its constants from the Wi-Fi SSID.

`condition` picks the tree: `Doglike`, `Control` or `LED`.

It also starts **the perception the tree needs**, which the base launch no
longer does -- `mecanumbot_bringup/launch/perception.launch.py` with the pose
detector, so `cam_people_detections` and `people_fusion` are there for the
robot to find the human it is leading. `use_perception:=false` if it is already
running.

`use_camera` defaults to **true** here and to false everywhere else, and that is
the one thing about this launcher worth knowing. The camera can only be opened
once: either the DeepStream detector opens it directly (cheapest, but there is
then no image topic at all) or the compressed publisher owns it and the detector
reads the topic. A leading trial is scored afterwards from what the robot could
see, so the recording is not optional here -- `/camera/image_raw/compressed` is
published for the whole run, at the cost of a JPEG encode and decode per frame.

Still needed, already running: the base launch (drivers, nav2 with AMCL
localized against the room's map, the LED service for the LED condition).
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PERCEPTION_LAUNCH = os.path.join(
    get_package_share_directory("mecanumbot_sensorprocess_smart"),
    "launch",
    "perception.launch.py",
)


def get_wifi_ssid():
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
    if ssid == "MecanumNet":
        return os.path.join(pkg_dir, "config", "behaviour_setting_constants.yaml")
    if ssid == "MecanumetoNet":
        return os.path.join(pkg_dir, "config", "Eto_behaviour_setting_constants.yaml")
    # Fallback
    if ssid == "APOLLO2028":
        return os.path.join(pkg_dir, "config", "behaviour_setting_constants.yaml")
    return os.path.join(pkg_dir, "config", "behaviour_setting_constants.yaml")


def generate_launch_description():
    pkg_dir = get_package_share_directory("mecanumbot_leading_behaviour")
    ssid = get_wifi_ssid()
    default_params_path = choose_default_param_file(ssid, pkg_dir)

    params = LaunchConfiguration("params")
    yaml_path = LaunchConfiguration("yaml_path")
    namespace = LaunchConfiguration("namespace")
    condition = LaunchConfiguration("condition")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params_path,
                description="YAML file with all constant parameters (override will bypass SSID detection)",
            ),
            DeclareLaunchArgument(
                "yaml_path",
                default_value=default_params_path,
                description="YAML path for behaviour tree nodes (env var or CLI arg override)",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="mecanumbot",
                description="Namespace for the behaviour node",
            ),
            DeclareLaunchArgument(
                "condition",
                default_value="Doglike",
                description="Behaviour condition: Doglike / Control / LED",
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
                default_value="true",
                description=(
                    "Publish /camera/image_raw/compressed for the whole trial "
                    "and feed the detector from it. True here because a leading "
                    "trial is scored afterwards from what the robot could see; "
                    "false lets the detector open the camera directly, which is "
                    "cheaper but leaves no image topic at all"
                ),
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="1280",
                description="Frame width, for the camera and everything that "
                "turns a pixel into an angle",
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
            # str(), because with no Wi-Fi detected `ssid` is None and
            # LogInfo refuses anything that is not a string or a substitution.
            LogInfo(msg=["Detected Wi-Fi SSID: ", str(ssid)]),
            LogInfo(msg=["Using params: ", params]),
            LogInfo(msg=["Using behaviour YAML: ", yaml_path]),
            SetEnvironmentVariable(name="YAML_PATH", value=yaml_path),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=yaml_path),
            # The perception this tree reads: DR-SPAAM on the scan, the pose
            # detector for the skeletons, and the fusion that turns the two into
            # `people_fusion`. Started here rather than by the base launch, so a
            # run that is not a leading trial does not pay for a DeepStream
            # network it has no subscriber for.
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
                package="mecanumbot_leading_behaviour",
                executable="doglike_leading_bt_node",
                name="doglike_leading_bt_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("/mecanumbot/cmd_vel", "/cmd_vel"),
                    ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
                parameters=[params],
                condition=IfCondition(
                    PythonExpression(["'", condition, "' == 'Doglike'"])
                ),
            ),
            Node(
                package="mecanumbot_leading_behaviour",
                executable="control_leading_bt_node",
                name="control_leading_bt_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("/mecanumbot/cmd_vel", "/cmd_vel"),
                    ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
                parameters=[params],
                condition=IfCondition(
                    PythonExpression(["'", condition, "' == 'Control'"])
                ),
            ),
            Node(
                package="mecanumbot_leading_behaviour",
                executable="LED_leading_bt_node",
                name="LED_leading_bt_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("/mecanumbot/cmd_vel", "/cmd_vel"),
                    ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
                parameters=[params],
                condition=IfCondition(PythonExpression(["'", condition, "' == 'LED'"])),
            ),
        ]
    )
