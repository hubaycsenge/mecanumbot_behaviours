"""
Launch a leading behaviour tree, picking its constants from the Wi-Fi SSID.

`condition` picks the tree: `Doglike`, `Control` or `LED`.

It also starts **the perception the tree needs**, which the base launch no
longer does -- `mecanumbot_bringup/launch/perception.launch.py` with the pose
detector, so `cam_people_detections` and `people_fusion` are there for the
robot to find the human it is leading. `use_perception:=false` if it is already
running.

`camera_source` defaults to **direct**, as in every behaviour launcher: the
DeepStream detector opens the USB webcam itself (`v4l2src`), so no camera node
runs and no frame passes through ROS 2 on its way to the network. Nothing needs
starting by hand. There is then no `/camera/image_raw/compressed`; what the
robot saw during a trial is on the detector's annotated frame,
`/mecanumbot/cam_people_detections/debug_image/compressed` (`debug_image`, on
by default), and that is the topic to record for scoring it afterwards.

`camera_source:=topic` reads `/camera/image_raw/compressed` instead, for a
clean, unannotated recording, at the cost of a camera node, a JPEG encode and a
decode per frame. **It does not start the camera** -- run
`ros2 launch mecanumbot_camera_stream camera_compressed.launch.py width:=1280
height:=720` first, or the detector gets no frames and publishes nothing.

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
                "camera_source",
                default_value="direct",
                choices=["direct", "topic"],
                description=(
                    "direct (default): the detector opens the webcam itself, no "
                    "ROS 2 middleware in the frame path; record debug_image to "
                    "score the trial. topic: read /camera/image_raw/compressed "
                    "-- does NOT start the camera, run "
                    "camera_compressed.launch.py first"
                ),
            ),
            DeclareLaunchArgument(
                "debug_image",
                default_value="true",
                description=(
                    "Publish the pose detector's annotated frame (boxes and "
                    "skeletons) on "
                    "/mecanumbot/cam_people_detections/debug_image/compressed. "
                    "false saves a frame copy and a JPEG encode per frame"
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
                default_value="640",
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
                    "camera_source": LaunchConfiguration("camera_source"),
                    "debug_image": LaunchConfiguration("debug_image"),
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
