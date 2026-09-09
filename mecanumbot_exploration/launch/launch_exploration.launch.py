"""
Exploration pass: stop AMCL, start slam_toolbox + explore_lite + orchestrator.

This is the m-explore-ros2 based exploration pass.  It differs from
launch_autoslam.launch.py in one important way: it does NOT start or restart
the nav2 navigation stack.  The study nav2 (started by
launch_mecanumbot_base.launch.py) stays up throughout; the preflight only
stops the localization stack (AMCL + map_server) so slam_toolbox can own
map -> odom without competing with it.

What this starts, in order:

  1. exploration_preflight -- stops lifecycle_manager_localization, then exits.
     Everything below waits for it to finish.
  2. slam_toolbox (async_slam_toolbox_node) -- mapping mode, publishes /map and
     map -> odom.
  3. explore_lite -- frontier detection and nav2 goal dispatch.
  4. exploration_node -- covariance monitor and finish detector.  Pauses and
     resumes explore_lite; latches exploration/finished when done.

Nav2 is NOT started here.  It is already running from the base launch and will
continue to accept NavigateToPose goals from explore_lite.

**What to run first.**  Nothing manual is required if use_preflight is true
(the default).  If you want to skip the preflight:

    ros2 service call /lifecycle_manager_localization/manage_nodes \\
      nav2_msgs/srv/ManageLifecycleNodes '{command: 1}'

**Saving the map.**  Saving is deliberately not automatic -- the map is worth
looking at before it is written:

    ros2 run nav2_map_server map_saver_cli -f <maps>/AI_dept/AI_dept
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, Shutdown,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXPLORATION_SHARE = get_package_share_directory("mecanumbot_exploration")
DESCRIPTION_SHARE = get_package_share_directory("mecanumbot_description")


def generate_launch_description():
    """Build the launch description for the exploration pass."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")
    params = LaunchConfiguration("params")
    namespace = LaunchConfiguration("namespace")
    use_preflight = LaunchConfiguration("use_preflight")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(
                DESCRIPTION_SHARE, "param", "mecanumbot_slam_mapping.yaml"
            ),
            description="slam_toolbox parameters",
        ),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(
                EXPLORATION_SHARE, "config", "exploration_constants.yaml"
            ),
            description="Exploration orchestrator constants",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the exploration_node",
        ),
        DeclareLaunchArgument(
            "use_preflight",
            default_value="true",
            description=(
                "Stop AMCL and map_server before starting the pass. "
                "Set false if you have already stopped the localization stack "
                "by hand."
            ),
        ),
        DeclareLaunchArgument(
            "preflight_strict",
            default_value="true",
            description=(
                "Stop if the preflight could not confirm AMCL is gone. "
                "Set false to start anyway; slam_toolbox and AMCL will race "
                "for map -> odom."
            ),
        ),
    ]

    preflight = Node(
        package="mecanumbot_exploration",
        executable="exploration_preflight",
        name="exploration_preflight",
        output="screen",
        condition=IfCondition(use_preflight),
        parameters=[params],
    )

    def _after_preflight(event, context):
        strict = context.perform_substitution(
            LaunchConfiguration("preflight_strict")).lower()
        if event.returncode and strict not in ("false", "0"):
            return [
                LogInfo(msg=(
                    "[exploration] the preflight could not stop AMCL; "
                    "slam_toolbox would conflict with it. Stopping. "
                    "Use preflight_strict:=false to start anyway.")),
                Shutdown(reason="exploration preflight could not stop AMCL"),
            ]
        if event.returncode:
            return [
                LogInfo(msg=(
                    "[exploration] preflight could not stop AMCL, but "
                    "preflight_strict is false -- starting anyway")),
            ] + stack()
        return stack()

    def stack():
        return [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="explore_lite",
                executable="explore",
                name="explore",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "robot_base_frame": "mecanumbot/base_link",
                    "costmap_topic": "/global_costmap/costmap",
                    "costmap_updates_topic": "/global_costmap/costmap_updates",
                    "visualize": True,
                    "planner_frequency": 0.5,
                    "progress_timeout": 30.0,
                    "potential_scale": 3.0,
                    "gain_scale": 1.0,
                    "transform_tolerance": 0.5,
                    "min_frontier_size": 0.3,
                }],
            ),
            Node(
                package="mecanumbot_exploration",
                executable="exploration_node",
                name="exploration_node",
                namespace=namespace,
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
            ),
        ]

    return LaunchDescription(
        arguments
        + [
            LogInfo(msg=(
                "exploration pass: AMCL preflight, then "
                "slam_toolbox + explore_lite + exploration_node"
            )),
            LogInfo(msg=["constants from ", params,
                         " -- watch /mecanumbot/exploration/state"]),
            preflight,
            RegisterEventHandler(
                OnProcessExit(target_action=preflight, on_exit=_after_preflight),
                condition=IfCondition(use_preflight),
            ),
            GroupAction(stack(), condition=UnlessCondition(use_preflight)),
        ]
    )
