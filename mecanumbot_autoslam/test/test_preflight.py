"""
What the preflight decides to stop, and what it must never stop.

These are claims about the *policy*, not about ROS: `plan()` takes a list of
node names and returns three lists, so every case here is a sentence about an
exploration pass that can be checked without a graph.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from mecanumbot_autoslam import preflight  # noqa: E402


def names(steps, method):
    """Return the node names planned for one method."""
    return [rule.node for rule in steps[method]]


# --- what contradicts an exploration pass ------------------------------------

def test_an_empty_graph_needs_nothing_stopped():
    steps = preflight.plan([])
    assert not any(steps.values())


def test_localization_is_stopped_through_its_manager():
    steps = preflight.plan(["/lifecycle_manager_localization", "/amcl"])
    assert names(steps, preflight.BY_MANAGER) == ["lifecycle_manager_localization"]
    # Its own manager will retire it; asking it directly as well only races.
    assert names(steps, preflight.BY_LIFECYCLE) == []


def test_a_managed_node_whose_manager_is_gone_is_asked_directly():
    steps = preflight.plan(["/amcl", "/map_server"])
    assert set(names(steps, preflight.BY_LIFECYCLE)) == {"amcl", "map_server"}


def test_one_managers_shutdown_does_not_cover_another_managers_nodes():
    # The localization manager retires amcl. The keepout mask server belongs to
    # a manager that is not up, so it still has to be asked directly.
    steps = preflight.plan([
        "/lifecycle_manager_localization", "/amcl", "/keepout_filter_mask_server"])
    assert names(steps, preflight.BY_MANAGER) == ["lifecycle_manager_localization"]
    assert names(steps, preflight.BY_LIFECYCLE) == ["keepout_filter_mask_server"]


# --- the study nav2 stack has to be gone, not merely stopped -----------------

def test_the_nav2_navigation_stack_is_removed_rather_than_shut_down():
    """
    A lifecycle shutdown does not free a node name.

    `navigation_launch.py` registers these under exactly the names the study
    stack holds, and a finalized node still answers `<name>/change_state` -- so
    a survivor answers our own manager's configure, nav2 aborts its bringup and
    tears down the servers it just started, and the pass gets no navigation.
    """
    stack = ["lifecycle_manager_navigation", "controller_server",
             "planner_server", "smoother_server", "behavior_server",
             "bt_navigator", "waypoint_follower", "velocity_smoother"]
    steps = preflight.plan(["/" + name for name in stack])
    assert set(names(steps, preflight.BY_PROCESS)) == set(stack)
    assert names(steps, preflight.BY_MANAGER) == []
    assert names(steps, preflight.BY_LIFECYCLE) == []


def test_every_name_autoslam_reuses_is_marked_as_a_collision():
    for name in ("controller_server", "bt_navigator", "slam_toolbox",
                 "lifecycle_manager_navigation", "autoslam_node"):
        steps = preflight.plan(["/" + name])
        assert steps[preflight.BY_PROCESS][0].collides, name


def test_a_tree_is_not_a_collision():
    # It contradicts a pass -- it sends its own goals -- but autoslam registers
    # nothing under its name, so a survivor degrades the run instead of
    # preventing it.
    steps = preflight.plan(["/mecanumbot/seek_bt_node"])
    assert not steps[preflight.BY_PROCESS][0].collides


def test_a_surviving_collision_blocks_the_pass():
    steps = preflight.plan(["/controller_server", "/mecanumbot/seek_bt_node"])
    assert [rule.node for rule in preflight.blocking(steps, ["/controller_server"])] \
        == ["controller_server"]


def test_a_surviving_tree_does_not_block_the_pass():
    steps = preflight.plan(["/controller_server", "/mecanumbot/seek_bt_node"])
    assert preflight.blocking(steps, ["/mecanumbot/seek_bt_node"]) == []


def test_the_navigation_manager_is_matched_by_its_node_remap_only():
    """
    Match the navigation manager by its remap, never by its executable.

    That executable is a bare `lifecycle_manager`, shared with two managers
    that must be shut down cleanly rather than signalled.
    """
    steps = preflight.plan([
        "/lifecycle_manager_navigation", "/lifecycle_manager_localization"])
    tokens = preflight.tokens_to_signal(steps)
    assert "lifecycle_manager" not in tokens
    manager = ["/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager",
               "--ros-args", "-r", "__node:=lifecycle_manager_localization"]
    assert preflight.matches_process(manager, tokens) == ""


def test_a_nav2_server_is_matched_by_its_executable():
    # navigation_launch.py starts them with no `name=`, so there is no
    # __node:= remap on the command line to match instead.
    steps = preflight.plan(["/controller_server"])
    tokens = preflight.tokens_to_signal(steps)
    argv = ["/opt/ros/humble/lib/nav2_controller/controller_server",
            "--ros-args", "--log-level", "info"]
    assert preflight.matches_process(argv, tokens) == "controller_server"


def test_a_behaviour_tree_sending_its_own_goals_is_stopped():
    for tree in ("seek_bt_node", "fetch_bt_node", "ostensive_bt_node",
                 "bottom_up_tree_node", "wander_between_people_node"):
        steps = preflight.plan(["/mecanumbot/" + tree])
        assert names(steps, preflight.BY_PROCESS) == [tree], tree


def test_a_second_slam_or_pass_is_stopped():
    steps = preflight.plan(["/slam_toolbox", "/mecanumbot/autoslam_node"])
    assert set(names(steps, preflight.BY_PROCESS)) == {
        "slam_toolbox", "autoslam_node"}


def test_the_explorer_from_before_the_move_is_still_recognised():
    steps = preflight.plan(["/mecanumbot/mecanumbot_frontier_explorer"])
    assert names(steps, preflight.BY_PROCESS) == ["mecanumbot_frontier_explorer"]


def test_a_namespaced_node_contradicts_exactly_as_much_as_a_bare_one():
    assert preflight.plan(["/mecanumbot/seek_bt_node"]) == \
        preflight.plan(["/seek_bt_node"])


# --- what is deliberately left alone -----------------------------------------

def test_the_joystick_is_never_stopped():
    # It publishes /cmd_vel, so by the letter of the rule it contradicts an
    # autonomous pass. It is also the only way a person in the room can take
    # the robot off a wall.
    steps = preflight.plan(["/mecanumbot/mecanumbot_joy_node", "/joy_node"])
    assert not any(steps.values())


def test_the_drivers_and_perception_are_left_alone():
    steps = preflight.plan([
        "/mecanumbot/mecanumbot_io_node",
        "/mecanumbot/mecanumbot_sensorproc_node",
        "/mecanumbot/mecanumbot_lidar_detect_people",
        "/mecanumbot/mecanumbot_locate_detections",
        "/mecanumbot/mecanumbot_deep3r_node",
        "/mecanumbot/mecanumbot_web_node",
    ])
    assert not any(steps.values())


# --- matching a process ------------------------------------------------------

def test_a_process_is_matched_on_the_executable_it_runs():
    assert preflight.matches_process(
        ["/opt/ws/install/lib/mecanumbot_seek/seek_bt_node", "--ros-args"],
        {"seek_bt_node": None}) == "seek_bt_node"


def test_a_launch_file_that_merely_names_a_tree_is_not_matched():
    # Killing the launch that starts the drivers because its command line
    # mentions a tree is the accident this is written to avoid.
    assert preflight.matches_process(
        ["ros2", "launch", "mecanumbot_seek", "launch_seek.launch.py"],
        {"seek_bt_node": None}) == ""


def test_our_own_process_group_is_never_signalled():
    processes = [
        (10, 10, ["/lib/mecanumbot_seek/seek_bt_node"]),
        (11, 99, ["/lib/mecanumbot_seek/seek_bt_node"]),
    ]
    hits = preflight.processes_to_signal(
        processes, {"seek_bt_node": None}, keep_groups=(99,))
    assert hits == [(10, "seek_bt_node")]


def test_every_leading_condition_is_looked_for_under_one_node_name():
    # All four leading trees register as bottom_up_tree_node, so the graph
    # cannot say which executable is running.
    steps = preflight.plan(["/bottom_up_tree_node"])
    tokens = preflight.tokens_to_signal(steps)
    assert set(preflight.LEADING_EXECUTABLES) <= set(tokens)
