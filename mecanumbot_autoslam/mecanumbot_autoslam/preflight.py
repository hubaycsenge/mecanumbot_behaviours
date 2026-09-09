"""
What has to stop before an autoslam pass can start, and how to stop it.

Pure Python -- no `rclpy`, no `subprocess`. This module only *decides*;
`tree_nodes/preflight_node.py` performs it.

T1 is the one pass in this workspace that replaces the navigation stack rather
than using it. slam_toolbox owns `map -> odom`, and nav2 comes up from
`navigation_launch.py` with no AMCL and no map server. So anything left running
from an ordinary session does not merely waste CPU -- it contradicts the pass:

* **AMCL** publishes `map -> odom` as well, and two publishers on one transform
  is the single most confusing way for an exploration run to fail.
* **`map_server`** serves the saved study map on `/map`, which is the topic
  slam_toolbox is building the new map on.
* **The study nav2 servers** register the same node names and serve the same
  `navigate_to_pose` action as the ones autoslam starts, so a goal goes to
  whichever of the two answers first.
* **A behaviour tree** sends its own nav2 goals. The explorer's goals and a
  leading tree's goals cancel each other in turn and the robot drives to
  neither.
* **A second explorer or slam_toolbox** is the same collision, from a launch
  that was not fully stopped.

Until now the answer was an instruction -- start the base launch with
`use_nav2:=false`, do not have a tree running -- and an instruction is something
to get wrong at the start of a trial. The preflight enforces it instead.

## Three ways to stop something, in order of how gentle they are

1. **A nav2 lifecycle manager.** `bringup_launch.py` puts every server it starts
   under one, and its `manage_nodes` service takes a SHUTDOWN command that
   transitions all of them to `finalized` in dependency order. One call retires
   AMCL, the map server and the whole planner/controller stack, and it is the
   only method here that nav2 itself considers a supported operation.
2. **A lifecycle node's own `change_state`**, for a managed node whose manager
   is gone (a partially stopped stack).
3. **SIGTERM to the process**, for everything that is not a lifecycle node at
   all -- the behaviour trees, slam_toolbox, an old explorer.

Method 3 is the one that can go wrong, so it is matched against the executable
name in a process's command line and never against a free-text pattern, it is
skipped for this process and everything in its own process group, and it is
behind `preflight_kill_processes` for a session that wants to be told rather
than tidied.

## What is deliberately NOT stopped

**The joystick.** `mecanumbot_joy_node` publishes `/cmd_vel`, so by the letter
of the rule it contradicts an autonomous pass. It is also the only way a person
in the room can take the robot off a wall. An exploration run that has disarmed
the human override is a worse failure than one that gets a joystick nudge in the
log.

**The drivers, the camera, the perception stack and the Deep3R client.** None of
them decide where the robot goes; T1 needs all of them.
"""

#: How a contradicting node is stopped.
BY_MANAGER = "manager"      #: nav2 lifecycle manager, `manage_nodes` SHUTDOWN
BY_LIFECYCLE = "lifecycle"  #: the node's own `change_state`, Shutdown
BY_PROCESS = "process"      #: SIGTERM to the process running it


class Contradiction:
    """One node that cannot run at the same time as an autoslam pass."""

    def __init__(self, node, method, why, executable="", manager=""):
        """Describe one node, why it contradicts, and how it is stopped."""
        self.node = node
        self.method = method
        self.why = why
        #: For a managed node, the lifecycle manager that owns it. While that
        #: manager is up the node is its business, not ours.
        self.manager = manager
        #: The executable name to look for in a command line, for BY_PROCESS.
        #: Never a substring to match loosely: it is compared against the
        #: basename of an argument, so `seek_bt_node` cannot match a launch
        #: file that merely mentions it.
        self.executable = executable or node

    def __repr__(self):
        """Return a short debugging representation."""
        return "Contradiction({!r}, {!r})".format(self.node, self.method)


#: Nav2 lifecycle managers. Shutting one of these down retires every server it
#: manages, which is why the servers below it are not listed individually.
MANAGERS = (
    Contradiction(
        "lifecycle_manager_navigation", BY_MANAGER,
        "the study nav2 stack serves the same navigate_to_pose action as the "
        "one autoslam starts"),
    Contradiction(
        "lifecycle_manager_localization", BY_MANAGER,
        "AMCL publishes map -> odom, which slam_toolbox owns during T1, and "
        "map_server serves the saved map on /map"),
    Contradiction(
        "lifecycle_manager_keepout_zone", BY_MANAGER,
        "the study keepout mask is drawn against the saved map, not the one "
        "being built"),
    Contradiction(
        "lifecycle_manager_slam", BY_MANAGER,
        "a second slam_toolbox is a second publisher of map -> odom"),
)

#: Managed nodes, for a stack whose manager is already gone. Listed after the
#: managers so that the tidy path is tried first, and each records which
#: manager owns it -- a node whose own manager is up is shut down by that
#: manager, and asking it directly as well only races.
MANAGED = tuple(
    Contradiction(name, BY_LIFECYCLE, why, manager=manager)
    for name, manager, why in (
        ("amcl", "lifecycle_manager_localization",
         "publishes map -> odom, which slam_toolbox owns during T1"),
        ("map_server", "lifecycle_manager_localization",
         "serves the saved map on /map"),
        ("controller_server", "lifecycle_manager_navigation",
         "part of the study nav2 stack"),
        ("planner_server", "lifecycle_manager_navigation",
         "part of the study nav2 stack"),
        ("smoother_server", "lifecycle_manager_navigation",
         "part of the study nav2 stack"),
        ("behavior_server", "lifecycle_manager_navigation",
         "part of the study nav2 stack"),
        ("bt_navigator", "lifecycle_manager_navigation",
         "serves navigate_to_pose"),
        ("waypoint_follower", "lifecycle_manager_navigation",
         "serves navigate_through_poses"),
        ("velocity_smoother", "lifecycle_manager_navigation",
         "republishes /cmd_vel"),
        ("keepout_filter_mask_server", "lifecycle_manager_keepout_zone",
         "serves the study keepout mask"),
        ("keepout_costmap_filter_info_server", "lifecycle_manager_keepout_zone",
         "serves the study keepout mask"),
    )
)

#: Plain nodes, stopped by signalling their process.
PROCESSES = (
    Contradiction(
        "slam_toolbox", BY_PROCESS,
        "a second slam_toolbox is a second publisher of map -> odom",
        executable="async_slam_toolbox_node"),
    Contradiction(
        "mecanumbot_autoslam", BY_PROCESS,
        "a second explorer sends its own goals to the same nav2",
        executable="autoslam_node"),
    Contradiction(
        "mecanumbot_frontier_explorer", BY_PROCESS,
        "the explorer that used to live in mecanumbot_custom_nav2, from an "
        "install that predates the move",
        executable="mecanumbot_frontier_explorer_node"),
    Contradiction(
        "bottom_up_tree_node", BY_PROCESS,
        "a leading tree sends its own nav2 goals",
        executable="bottom_up_tree_node"),
    Contradiction(
        "ostensive_bt_node", BY_PROCESS,
        "the ostensive tree publishes its own /goal_pose"),
    Contradiction(
        "seek_bt_node", BY_PROCESS,
        "the seek tree sends its own nav2 goals"),
    Contradiction(
        "fetch_bt_node", BY_PROCESS,
        "the fetch tree sends its own nav2 goals"),
    Contradiction(
        "wander_between_people_node", BY_PROCESS,
        "the demo tree sends its own nav2 goals"),
)

#: Every rule, in the order they are tried.
RULES = MANAGERS + MANAGED + PROCESSES

#: Node names that are never touched, whatever else matches. The joystick is
#: the human override; see the module docstring.
KEEP = (
    "mecanumbot_joy_node",
    "joy_node",
)

#: The leading experiment registers all four of its trees under one ROS node
#: name, so `bottom_up_tree_node` covers them -- but the *executable* differs
#: per condition, and that is what a process is matched by.
LEADING_EXECUTABLES = (
    "bottom_up_tree_node",
    "doglike_leading_bt_node",
    "control_leading_bt_node",
    "LED_leading_bt_node",
)


def bare(node_name):
    """Return a node name without its namespace."""
    return str(node_name).rsplit("/", 1)[-1]


def plan(live_nodes):
    """
    Return the contradictions among `live_nodes`, grouped by how they stop.

    `live_nodes` are node names as ROS reports them, with or without a
    namespace; matching is on the bare name, because a node in the robot's
    namespace contradicts exactly as much as one outside it.
    """
    live = {bare(name) for name in live_nodes} - set(KEEP)
    found = [rule for rule in RULES if rule.node in live]

    # A managed node whose own manager is also up is that manager's business:
    # it shuts its nodes down in dependency order, and asking one of them
    # directly as well only races it.
    live_managers = {rule.node for rule in found if rule.method == BY_MANAGER}
    found = [
        rule for rule in found
        if not (rule.method == BY_LIFECYCLE and rule.manager in live_managers)
    ]

    return {
        BY_MANAGER: [rule for rule in found if rule.method == BY_MANAGER],
        BY_LIFECYCLE: [rule for rule in found if rule.method == BY_LIFECYCLE],
        BY_PROCESS: [rule for rule in found if rule.method == BY_PROCESS],
    }


def candidate_executables(rule):
    """
    Return every executable name one rule can be running as.

    Usually one. The leading experiment is the exception: all four of its
    conditions register under the ROS node name `bottom_up_tree_node`, so the
    graph cannot say which of them is up and each has to be looked for.
    """
    if rule.executable == "bottom_up_tree_node":
        return tuple(LEADING_EXECUTABLES)
    return (rule.executable,)


def executables_to_signal(steps):
    """Return the executable names the BY_PROCESS half of a plan looks for."""
    names = []
    for rule in steps.get(BY_PROCESS, ()):
        for name in candidate_executables(rule):
            if name not in names:
                names.append(name)
    return names


def matches_executable(cmdline, executables):
    """
    Return the executable a command line runs, if it is one of `executables`.

    The command line is compared argument by argument against the *basename* of
    each argument, so `/opt/ros/install/lib/mecanumbot_seek/seek_bt_node` is a
    match and `ros2 launch mecanumbot_seek launch_seek.launch.py` is not. A
    loose substring search over the whole line would match a launch file that
    only names the node, and killing the launch that starts the drivers because
    it mentions a tree is exactly the accident this is written to avoid.
    """
    for argument in cmdline:
        base = str(argument).rsplit("/", 1)[-1]
        if base in executables:
            return base
    return ""


def processes_to_signal(processes, executables, keep_pids=(), keep_groups=()):
    """
    Return `(pid, executable)` for every process that should be signalled.

    `processes` are `(pid, process_group, cmdline)` triples. This process and
    its own process group are never included: the preflight runs from the same
    launch as the explorer it is clearing the way for, and a launch file that
    kills itself would be a memorable way to start a trial.
    """
    keep_pids = set(keep_pids)
    keep_groups = set(keep_groups)
    hits = []
    for pid, group, cmdline in processes:
        if pid in keep_pids or group in keep_groups:
            continue
        executable = matches_executable(cmdline, executables)
        if executable:
            hits.append((pid, executable))
    return hits


def summary(steps):
    """Return one line per contradiction found, for the log."""
    lines = []
    for method in (BY_MANAGER, BY_LIFECYCLE, BY_PROCESS):
        for rule in steps.get(method, ()):
            lines.append("{} ({}): {}".format(rule.node, method, rule.why))
    return lines
