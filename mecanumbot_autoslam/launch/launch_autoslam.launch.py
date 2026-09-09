"""
T1: clear the graph, then SLAM, nav2 without localization, and explore.

This is the whole first pass in one launch file. It shuts down whatever
contradicts an exploration run, brings up slam_toolbox in mapping mode, nav2
against the exploration parameter file in `mecanumbot_description`, the 2D/3D
comparison handler, and the behaviours that drive the two of them around until
the exit criteria are met.

**The preflight runs first and the rest waits for it to exit.** Not a timer: the
stack below has to come up into a graph that has already been cleared, and a
race between shutting the study nav2 down and starting our own is the kind of
failure that looks like a bad map three hours later. What it stops and why is
`preflight.py`; the short version is the study nav2 stack, AMCL, the map server,
any behaviour tree sending its own nav2 goals, and a second slam_toolbox or
explorer. It deliberately leaves the joystick alone -- that is the human
override -- and it leaves the drivers, the camera, the perception stack and the
Deep3R client alone, because T1 needs all of them.

**And if it could not clear a name collision, nothing else starts.**
`navigation_launch.py` below registers `controller_server`, `bt_navigator` and
six more under exactly the names the study stack holds. With an old one still
running, our manager's `configure` is answered by the wrong node, nav2 aborts
its own bringup and tears down the servers it just started, and the pass logs
"nav2 action server not ready" for as long as you leave it. That is not a
degraded run, it is no run, so the preflight exits non-zero and this file stops
instead of spending a trial on it. `preflight_strict:=false` starts anyway.

Nav2 comes from `nav2_bringup/navigation_launch.py` rather than the usual
`bringup_launch.py`, and that choice is the whole point: `navigation_launch`
starts the controller, planner, behaviours and BT navigator and *nothing else*
-- no `map_server`, no AMCL. During T1 there is no map to localize against and
slam_toolbox owns `map -> odom`; bringing AMCL up as well gives two things
estimating the same transform, which is the single most confusing way for an
exploration run to fail.

It does **not** bring up the robot's drivers, its camera, or the Deep3R client.
Those come from their own launch files, and starting them here would give two
launch files that both own the OpenCR link:

    ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py use_nav2:=false
    ros2 launch mecanumbot_deep3r deep3r.launch.py
    ros2 launch mecanumbot_autoslam launch_autoslam.launch.py

`use_nav2:=false` is still worth passing even though the preflight would clear
it: not starting the study stack is cheaper and quieter than starting it and
shutting it down. `launch_t1.launch.py` does all three in the right order.

When the behaviours are satisfied they latch `/mecanumbot/exploration/finished`.
Saving the map is deliberately not automatic -- the map T2 localizes against is
worth looking at before it is written, and `map_saver_cli` is one command:

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

AUTOSLAM_SHARE = get_package_share_directory("mecanumbot_autoslam")
DESCRIPTION_SHARE = get_package_share_directory("mecanumbot_description")
CUSTOM_NAV2_SHARE = get_package_share_directory("mecanumbot_custom_nav2")
NAV2_SHARE = get_package_share_directory("nav2_bringup")


def generate_launch_description():
    """Build the launch description for the autoslam exploration pass."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")
    nav2_params = LaunchConfiguration("nav2_params")
    params = LaunchConfiguration("params")
    agreement_params = LaunchConfiguration("agreement_params")
    namespace = LaunchConfiguration("namespace")
    require_cloud = LaunchConfiguration("require_cloud")
    use_preflight = LaunchConfiguration("use_preflight")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock (sim.launch.py sets this)",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(
                DESCRIPTION_SHARE, "param", "mecanumbot_slam_mapping.yaml"
            ),
            description="slam_toolbox parameters",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(
                DESCRIPTION_SHARE, "param", "mecanumbot_exploration_nav2.yaml"
            ),
            description=(
                "Nav2 parameters. The exploration file has no AMCL block and no "
                "static layer, because slam_toolbox owns map -> odom during T1."
            ),
        ),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(
                AUTOSLAM_SHARE, "config", "autoslam_setting_constants.yaml"
            ),
            description="The autoslam constants, as a ROS parameter file",
        ),
        DeclareLaunchArgument(
            "agreement_params",
            default_value=os.path.join(
                CUSTOM_NAV2_SHARE, "config", "map_agreement.yaml"
            ),
            description="Constants for the 2D/3D comparison handler",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the autoslam node",
        ),
        DeclareLaunchArgument(
            "require_cloud",
            default_value="true",
            description=(
                "Whether T1 may only end once the server says the reconstruction "
                "is good enough. Set false for a mapping-only dry run with no "
                "Deep3R link -- otherwise the exit criteria are never satisfied."
            ),
        ),
        DeclareLaunchArgument(
            "use_preflight",
            default_value="true",
            description=(
                "Shut down what contradicts an exploration pass before starting "
                "it. False starts straight away, and then the study nav2 stack, "
                "AMCL or a running tree will fight this one."
            ),
        ),
        DeclareLaunchArgument(
            "preflight_strict",
            default_value="true",
            description=(
                "Stop if the preflight could not clear a node whose name "
                "autoslam re-registers. Those collisions make nav2 bringup "
                "abort, so starting anyway wastes the run rather than "
                "degrading it. false starts regardless."
            ),
        ),
        DeclareLaunchArgument(
            "use_agreement",
            default_value="true",
            description=(
                "Start the 2D/3D comparison handler. False for a session that "
                "already has one up from the T2 launch."
            ),
        ),
    ]

    preflight = Node(
        package="mecanumbot_autoslam",
        executable="autoslam_preflight",
        name="autoslam_preflight",
        output="screen",
        condition=IfCondition(use_preflight),
        parameters=[params],
    )

    def _after_preflight(event, context):
        """Start the pass, or stop, depending on what the preflight found."""
        strict = context.perform_substitution(
            LaunchConfiguration("preflight_strict")).lower()
        if event.returncode and strict not in ("false", "0"):
            return [
                LogInfo(msg=(
                    "[autoslam] the preflight could not clear the graph and "
                    "nav2 would not come up; stopping. Its log above says "
                    "what is still running. Start with "
                    "preflight_strict:=false to try anyway.")),
                Shutdown(reason="autoslam preflight found a name collision"),
            ]
        if event.returncode:
            return [
                LogInfo(msg=(
                    "[autoslam] the preflight could not clear the graph, but "
                    "preflight_strict is false -- starting anyway. Expect nav2 "
                    "bringup to abort if a name is still held.")),
            ] + stack()
        return stack()

    def stack():
        """
        Build the pass itself, fresh each call.

        A launch action may only appear once in a launch description, and this
        list appears in two places -- behind the preflight's exit, and straight
        away when there is no preflight -- so it is built rather than shared.
        """
        return [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
            # Navigation only: controller, planner, behaviours, BT navigator,
            # smoother. No map_server and no AMCL -- see the module docstring.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(NAV2_SHARE, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": "true",
                    # Nav2 stays outside the robot namespace, as it does
                    # everywhere else in this workspace: the trees and these
                    # behaviours address it with absolute names.
                    "namespace": "",
                }.items(),
            ),
            # Runs in both phases -- see its README. It is started here because
            # T1 is where the verdicts begin arriving, and started again by the
            # seek launch for T2; running two is harmless (they publish the same
            # mask) but pointless, so a session that spans both phases should
            # keep this one alive.
            Node(
                package="mecanumbot_custom_nav2",
                executable="mecanumbot_map_agreement_node",
                name="mecanumbot_map_agreement",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_agreement")),
                parameters=[agreement_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="mecanumbot_autoslam",
                executable="autoslam_node",
                name="autoslam_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    params,
                    {"use_sim_time": use_sim_time, "require_cloud": require_cloud},
                ],
            ),
        ]

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=(
                    "autoslam (T1): preflight, then slam_toolbox + nav2 (no "
                    "AMCL) + the 2D/3D comparison handler + the exploration "
                    "behaviours"
                )
            ),
            LogInfo(
                msg=["constants from ", params,
                     " -- watch /mecanumbot/exploration/state"],
            ),
            preflight,
            # The pass starts when the preflight has exited, not after a delay
            # -- and only if it says the graph is fit to start in. A non-zero
            # exit means something is still holding a node name we need.
            RegisterEventHandler(
                OnProcessExit(target_action=preflight, on_exit=_after_preflight),
                condition=IfCondition(use_preflight),
            ),
            GroupAction(stack(), condition=UnlessCondition(use_preflight)),
        ]
    )
