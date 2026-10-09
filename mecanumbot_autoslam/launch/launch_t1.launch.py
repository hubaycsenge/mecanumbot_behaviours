"""
T1 in one terminal: drivers, then the Deep3R client, SLAM, nav2 and the explorer.

`launch_autoslam.launch.py` is the whole pass -- the Deep3R client (which
opens the camera itself), the preflight, SLAM, nav2 and the explorer -- but
deliberately not the robot's drivers: a launch file that also owned the OpenCR
link would be two files fighting over one serial port whenever the drivers were
already up. This file is the convenience wrapper that adds them: one Ctrl-C
stops everything.

    ros2 launch mecanumbot_autoslam launch_t1.launch.py

What it starts, in order:

1. `launch_mecanumbot_base.launch.py` with **use_nav2:=false**. The base launch
   otherwise brings up nav2 against the *study* parameters with AMCL and a saved
   map, and that stack and this one both publish `map -> odom` and both serve
   `navigate_to_pose`. Hard-coded false here rather than exposed, because there
   is no T1 in which you want the study stack. `launch_autoslam` runs a
   preflight that would shut it down anyway; not starting it is cheaper and
   quieter than starting it and retiring it.
2. `launch_autoslam.launch.py`, after `explorer_delay` seconds -- slam_toolbox
   wants odometry and a scan before its first update. It starts the Deep3R
   client -- and with it the camera -- at once, and SLAM, nav2 and the
   explorer when its preflight has cleared the graph.

The delay is generous rather than tuned. Nothing here is a race that a
correctly ordered start would lose; it exists so the logs read in the order the
subsystems actually came up, which is most of what makes a failed start
diagnosable in a single scrolling terminal.

**The output is every stack in one terminal.** That is the trade. When
something is wrong and the noise is in the way, fall back to the two-terminal
form -- it is the same two launch files, and this package's README documents
them under "Running".

The server on the cluster is *not* started by this and cannot be: it is a Slurm
job on the other side of an SSH tunnel. Bring it up first, and check the tunnel,
before running this. See `RoboCamStreamProcessing/README.md`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BRINGUP_SHARE = get_package_share_directory("mecanumbot_bringup")
AUTOSLAM_SHARE = get_package_share_directory("mecanumbot_autoslam")

#: Declared here with launch_autoslam's defaults and handed straight to it, so a
#: `use_deep3r:=false` on this file reaches the include rather than stopping at
#: the wrapper.
PASSED_THROUGH = (
    ("require_cloud", "true",
     "Let the server's verdict decide when T1 is over. false drops the `placed` "
     "test -- right for a mapping dry run, wrong during a trial, because a "
     "failed T1 then looks exactly like a successful one."),
    ("camera_device", "/dev/video0",
     "The camera the Deep3R client opens. Nothing else may hold it: a device "
     "can be opened once, and the client is the only thing that needs it."),
    ("camera_width", "1280",
     "Width to ASK the camera for. A driver may refuse and give another; the "
     "client logs what it actually got, and the server's calib_width has to "
     "describe that, not this."),
    ("camera_height", "720", "Height to ask for; see camera_width."),
    ("camera_fps", "15.0", "Capture rate. The server reconstructs at ~6 Hz."),
    ("publish_debug_image", "false",
     "Have the Deep3R client republish the exact frames it sent, for eyes on "
     "the robot. Off by default: nothing in the pass reads it."),
    # Declared and passed on only so that launch_autoslam can say it was
    # ignored. Dropping it here instead would make it vanish in silence.
    ("use_camera", "",
     "RETIRED, ignored. There is no camera publisher: the Deep3R client opens "
     "the device itself, so use_deep3r is the switch. Passing it logs a "
     "warning and changes nothing."),
    ("use_deep3r", "true",
     "Start the Deep3R client. false is mapping only, and then require_cloud "
     "must be false too or the exit criteria can never be satisfied."),
    # Before `server`, whose default is built from it.
    ("server_port", "5555",
     "Port of the local end of the forward tunnel. Change it when 5555 is "
     "taken; the tunnel has to forward the same port."),
    ("server", ["tcp://127.0.0.1:", LaunchConfiguration("server_port")],
     "Local end of the forward tunnel, not a cluster address. Set it whole to "
     "override the host as well, and server_port is then ignored."),
    ("client_path", "~/robocam_client.py",
     "Deployed robocam_client.py, or the directory holding it."),
    ("run_id", "",
     "Deep3R run id. Empty starts a fresh reconstruction; a previous run's id "
     "(logged by the client at startup) resumes it."),
)


def generate_launch_description():
    """Build the launch description for a whole T1 pass in one terminal."""
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(BRINGUP_SHARE, "launch", "launch_mecanumbot_base.launch.py")
        ),
        # Not a parameter of this file: there is no T1 in which the study nav2
        # stack is what you want, and making it settable would only make it
        # possible to get wrong.
        launch_arguments={"use_nav2": "false"}.items(),
    )

    explorer = TimerAction(
        period=LaunchConfiguration("explorer_delay"),
        actions=[
            LogInfo(msg="[t1] Deep3R client (it opens the camera), then preflight, "
                        "SLAM, nav2 and the exploration behaviours"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(AUTOSLAM_SHARE, "launch", "launch_autoslam.launch.py")
                ),
                launch_arguments={
                    name: LaunchConfiguration(name) for name, _, _ in PASSED_THROUGH
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=doc)
         for name, default, doc in PASSED_THROUGH]
        + [
            DeclareLaunchArgument(
                "explorer_delay", default_value="15.0",
                description="Seconds to wait for odometry and a scan before the pass.",
            ),
            LogInfo(msg=[
                "[t1] one-terminal start. Ctrl-C stops everything. "
                "The cluster server is NOT started by this -- bring it up first.",
            ]),
            LogInfo(msg="[t1] starting drivers (nav2 off: autoslam brings its own)"),
            base,
            explorer,
        ]
    )
