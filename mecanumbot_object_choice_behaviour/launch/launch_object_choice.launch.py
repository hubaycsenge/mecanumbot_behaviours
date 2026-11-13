"""
Launch the object-choice tree and the perception it reads.

Perception is the pose detector -- the cue is read off COCO-17 keypoints -- plus
the brick detector, `perception.launch.py bricks:=true`. The brick detector
opens the webcam and republishes it, so the pose detector reads that stream
(`camera_source:=topic`); the camera can be opened only once. Nothing else may
hold `/dev/video0`.

The tree node is started with five remappings, and two of them are load-bearing:
the fetch behaviours' `ball_detections` and `cam_ball_boxes` are pointed at the
object-choice relay, which forwards only the chosen brick. Started any other
way, the tree logs an error at setup saying the relay is bypassed.

Still needed, already running: the base launch (drivers, nav2 with AMCL
localized), with the robot placed at the trial's start pose facing the layout.
"""

import os

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

# What the fetch behaviours subscribe and publish to, and where it goes here.
# See mecanumbot_object_choice_behaviour/behaviours/ros_interfaces.py.
REMAPPINGS = [
    ("/mecanumbot/cmd_vel", "/cmd_vel"),
    ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
    ("/mecanumbot/ball_detections", "/mecanumbot/object_choice/target_detections"),
    ("/mecanumbot/cam_ball_boxes", "/mecanumbot/object_choice/target_boxes"),
    ("/mecanumbot/fetch/state", "/mecanumbot/object_choice/state"),
]


def generate_launch_description():
    """Build the launch description for the object-choice tree."""
    pkg_dir = get_package_share_directory("mecanumbot_object_choice_behaviour")
    default_params = os.path.join(
        pkg_dir, "config", "object_choice_setting_constants.yaml"
    )
    params = LaunchConfiguration("params")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params,
                description="YAML with the object-choice constants",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="mecanumbot", description="Node namespace"
            ),
            DeclareLaunchArgument(
                "use_perception",
                default_value="true",
                description=(
                    "Start the pose detector, the brick detector and the fusion. "
                    "false only when they are already running, the same way"
                ),
            ),
            DeclareLaunchArgument(
                "debug_image",
                default_value="true",
                description=(
                    "Annotated frames from both detectors: "
                    "cam_people_detections/debug_image/compressed and "
                    "cam_brick_detections/debug_image/compressed. Record both"
                ),
            ),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="720"),
            DeclareLaunchArgument("yolo_imgsz", default_value="1280"),
            DeclareLaunchArgument("yolo_model", default_value="yolo26m-pose"),
            LogInfo(msg=["Using object-choice YAML: ", params]),
            SetEnvironmentVariable(name="YAML_PATH", value=params),
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PERCEPTION_LAUNCH),
                condition=IfCondition(LaunchConfiguration("use_perception")),
                launch_arguments={
                    "namespace": namespace,
                    "detector": "pose",
                    "camera_source": "topic",
                    "bricks": "true",
                    "debug_image": LaunchConfiguration("debug_image"),
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "yolo_imgsz": LaunchConfiguration("yolo_imgsz"),
                    "yolo_model": LaunchConfiguration("yolo_model"),
                }.items(),
            ),
            Node(
                package="mecanumbot_object_choice_behaviour",
                executable="object_choice_bt_node",
                name="object_choice_bt_node",
                namespace=namespace,
                output="screen",
                remappings=REMAPPINGS,
                parameters=[params],
            ),
        ]
    )
