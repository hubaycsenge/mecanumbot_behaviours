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

    def __init__(self, node, method, why, executable=None, manager="",
                 collides=False):
        """Describe one node, why it contradicts, and how it is stopped."""
        self.node = node
        self.method = method
        self.why = why
        #: For a managed node, the lifecycle manager that owns it. While that
        #: manager is up the node is its business, not ours.
        self.manager = manager
        #: Whether autoslam re-registers a node under this exact name. If it
        #: does, the old one has to be GONE and not merely stopped -- see the
        #: module docstring. A collision that survives makes the pass
        #: impossible, so it is also what the preflight refuses to start over.
        self.collides = collides
        #: The executable name to look for in a command line, for BY_PROCESS.
        #: Never a substring to match loosely: it is compared against the
        #: basename of an argument, so `seek_bt_node` cannot match a launch
        #: file that merely mentions it. Defaults to the node's own name,
        #: which is what a nav2 server's executable is called; an explicit ""
        #: means match on the `__node:=` remap alone, for a node whose
        #: executable is shared with something that must NOT be signalled.
        self.executable = node if executable is None else executable

    def __repr__(self):
        """Return a short debugging representation."""
        return "Contradiction({!r}, {!r})".format(self.node, self.method)


# ── stopping something, versus removing it ──────────────────────────────────
#
# The distinction the first version of this file missed, and it is the whole
# difference between a pass that starts and one that cannot.
#
# `navigation_launch.py` registers `controller_server`, `planner_server`,
# `bt_navigator` and five more under **exactly the names the study stack already
# has**. A lifecycle shutdown does not free a name: a finalized node is still on
# the graph and still answers `<name>/change_state`. So when the study servers
# are merely shut down -- or, worse, not shut down at all -- our own
# `lifecycle_manager_navigation` sends `configure` to `controller_server` and
# gets an answer from the wrong one. An *active* old server rejects `configure`
# outright, the manager logs "Failed to bring up all requested nodes. Aborting
# bringup", and then tears down the servers we just started. Nav2 never comes
# up, and the pass sits there logging "nav2 action server not ready".
#
# So: things that merely contradict are **stopped**, and things whose names we
# re-register are **removed**.
#
# **And "removed" often means one process, not eight.** `bringup_launch.py`
# defaults `use_composition` to True, so the study stack is not eight processes
# called `controller_server`, `planner_server` and so on -- it is eight *nodes*
# composed into a single `component_container_isolated` called `nav2_container`.
# Looking for a process per node finds nothing at all there, which is worse than
# useless: it reads as "not running here, must be on another machine" and lets
# the pass start into the collision. The container is the thing to signal, and
# it is matched by its `__node:=nav2_container` remap alone -- never by
# `component_container_isolated`, which is a name other containers in this
# workspace could share.

#: Nav2 lifecycle managers whose nodes autoslam does NOT re-register. Shutting
#: one of these down retires every server under it, in dependency order, and a
#: finalized AMCL publishes no transform -- which is all that is needed.
MANAGERS = (
    Contradiction(
        "lifecycle_manager_localization", BY_MANAGER,
        "AMCL publishes map -> odom, which slam_toolbox owns during T1, and "
        "map_server serves the saved map on /map"),
    Contradiction(
        "lifecycle_manager_keepout_zone", BY_MANAGER,
        "the study keepout mask is drawn against the saved map, not the one "
        "being built"),
)

#: Managed nodes, for a stack whose manager is already gone. Each records which
#: manager owns it -- a node whose own manager is up is shut down by that
#: manager, and asking it directly as well only races.
MANAGED = tuple(
    Contradiction(name, BY_LIFECYCLE, why, manager=manager)
    for name, manager, why in (
        ("amcl", "lifecycle_manager_localization",
         "publishes map -> odom, which slam_toolbox owns during T1"),
        ("map_server", "lifecycle_manager_localization",
         "serves the saved map on /map"),
        ("keepout_filter_mask_server", "lifecycle_manager_keepout_zone",
         "serves the study keepout mask"),
        ("keepout_costmap_filter_info_server", "lifecycle_manager_keepout_zone",
         "serves the study keepout mask"),
    )
)

#: The study nav2 navigation stack: every name in it is one autoslam re-uses,
#: so these are removed rather than shut down. Signalling them is the same
#: thing `ros2 launch` does to them on Ctrl-C -- rclcpp handles SIGTERM -- and
#: it leaves the rest of the base launch (drivers, joystick, web GUI) running.
#:
#: The manager is matched on its `__node:=` remap and not on its executable,
#: which is a bare `lifecycle_manager` shared with the localization and keepout
#: managers. Those are shut down cleanly above, and must not be caught here.
NAV2_STACK = (
    Contradiction(
        "lifecycle_manager_navigation", BY_PROCESS,
        "would answer for, and fight over, the manager autoslam starts under "
        "this same name", executable="", collides=True),
    Contradiction(
        "controller_server", BY_PROCESS,
        "autoslam starts a controller_server too, and an active one rejects "
        "the new manager's configure, which aborts the whole bringup",
        collides=True),
    Contradiction(
        "planner_server", BY_PROCESS,
        "autoslam starts a planner_server under this name", collides=True),
    Contradiction(
        "smoother_server", BY_PROCESS,
        "autoslam starts a smoother_server under this name", collides=True),
    Contradiction(
        "behavior_server", BY_PROCESS,
        "autoslam starts a behavior_server under this name", collides=True),
    Contradiction(
        "bt_navigator", BY_PROCESS,
        "autoslam starts a bt_navigator under this name, and both would serve "
        "navigate_to_pose", collides=True),
    Contradiction(
        "waypoint_follower", BY_PROCESS,
        "autoslam starts a waypoint_follower under this name", collides=True),
    Contradiction(
        "velocity_smoother", BY_PROCESS,
        "autoslam starts a velocity_smoother under this name, and both would "
        "republish /cmd_vel", collides=True),
)

#: The composed form of everything in NAV2_STACK. Present instead of the
#: individual processes whenever nav2 was brought up with `use_composition`,
#: which is the `bringup_launch.py` default and therefore what the base launch
#: gets. Signalling it takes every node inside it, which is the whole study
#: stack -- localization included, and that is fine: it is a contradiction too,
#: already asked to shut down cleanly by then.
CONTAINER = (
    Contradiction(
        "nav2_container", BY_PROCESS,
        "the composed study nav2 stack -- with use_composition (the nav2 "
        "bringup default) every server runs inside this one process, so there "
        "is no controller_server process to signal",
        executable="", collides=True),
)

#: Plain nodes, stopped by signalling their process.
PROCESSES = NAV2_STACK + CONTAINER + (
    Contradiction(
        "slam_toolbox", BY_PROCESS,
        "autoslam starts slam_toolbox under this name, and two of them are two "
        "publishers of map -> odom",
        executable="async_slam_toolbox_node", collides=True),
    Contradiction(
        "autoslam_node", BY_PROCESS,
        "a second pass sends its own goals to the same nav2",
        executable="autoslam_node", collides=True),
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


def candidate_tokens(rule):
    """
    Return every command-line token that identifies one rule's process.

    Two shapes, because ROS 2 offers two. A node started without a `name=`
    carries only its **executable** (`controller_server`), and one started with
    a `name=` also carries a `__node:=` **remap** -- which is the only handle on
    a node like `lifecycle_manager_navigation`, whose executable is a bare
    `lifecycle_manager` shared with the localization and keepout managers that
    must be shut down cleanly rather than signalled.

    The leading experiment is the other special case: all four of its conditions
    register under the ROS node name `bottom_up_tree_node`, so the graph cannot
    say which executable is up and each has to be looked for.
    """
    tokens = ["__node:={}".format(rule.node)]
    if rule.executable == "bottom_up_tree_node":
        tokens.extend(LEADING_EXECUTABLES)
    elif rule.executable:
        tokens.append(rule.executable)
    else:
        return tuple(tokens)
    return tuple(tokens)


def candidate_executables(rule):
    """Return the executable names one rule can be running as."""
    return tuple(token for token in candidate_tokens(rule)
                 if not token.startswith("__node:="))


def tokens_to_signal(steps):
    """Return `{token: rule}` for the BY_PROCESS half of a plan."""
    table = {}
    for rule in steps.get(BY_PROCESS, ()):
        for token in candidate_tokens(rule):
            table.setdefault(token, rule)
    return table


def matches_process(cmdline, tokens):
    """
    Return the token identifying what a command line runs, or "".

    Each argument is compared whole, and by the **basename** of its path, so
    `/opt/ros/humble/lib/nav2_controller/controller_server` matches and
    `ros2 launch mecanumbot_seek launch_seek.launch.py` does not. A loose
    substring search over the whole line would match a launch file that only
    names the node, and killing the launch that starts the drivers because its
    command line mentions a tree is the accident this is written to avoid.
    """
    for argument in cmdline:
        text = str(argument)
        if text in tokens:
            return text
        base = text.rsplit("/", 1)[-1]
        if base in tokens:
            return base
    return ""


def processes_to_signal(processes, tokens, keep_pids=(), keep_groups=()):
    """
    Return `(pid, token)` for every process that should be signalled.

    `processes` are `(pid, process_group, cmdline)` triples. This process and
    its own process group are never included: the preflight runs from the same
    launch as the pass it is clearing the way for, and a launch file that kills
    itself would be a memorable way to start a trial.
    """
    keep_pids = set(keep_pids)
    keep_groups = set(keep_groups)
    hits = []
    for pid, group, cmdline in processes:
        if pid in keep_pids or group in keep_groups:
            continue
        token = matches_process(cmdline, tokens)
        if token:
            hits.append((pid, token))
    return hits


def survivors(steps, live_nodes):
    """
    Return the rules whose node is still on the graph after acting.

    **The graph, not the process table.** What matters for a name collision is
    whether the name is still taken, and it is the graph that says so: a
    lifecycle node that shut down cleanly is still registered under its name and
    still answers `<name>/change_state`, and a node composed into a container
    never had a process of its own to look for. Asking about processes answered
    a different question and answered it wrongly -- eight composed nav2 nodes
    came back as "no local process running it, must be on another machine", and
    the pass started into exactly the collision the preflight exists to prevent.
    """
    names = {bare(name) for name in live_nodes}
    return [rule for group in steps.values() for rule in group
            if rule.node in names]


def blocking(steps, live_nodes):
    """
    Return the contradictions that make an autoslam pass impossible.

    Not the same question as "did everything stop". A tree on the operator PC
    that could not be signalled from here degrades a pass; a study
    `controller_server` still holding the name ours needs *prevents* it, because
    nav2's bringup will abort on the collision and take our own servers down
    with it. Only the second is worth refusing to start over.
    """
    return [rule for rule in survivors(steps, live_nodes) if rule.collides]


def summary(steps):
    """Return one line per contradiction found, for the log."""
    lines = []
    for method in (BY_MANAGER, BY_LIFECYCLE, BY_PROCESS):
        for rule in steps.get(method, ()):
            lines.append("{} ({}): {}".format(rule.node, method, rule.why))
    return lines
