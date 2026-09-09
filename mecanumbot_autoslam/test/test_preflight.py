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


def test_the_study_nav2_stack_is_stopped_through_its_manager():
    steps = preflight.plan(["/lifecycle_manager_navigation", "/bt_navigator"])
    assert names(steps, preflight.BY_MANAGER) == ["lifecycle_manager_navigation"]
    # Its own manager will retire it; asking it directly as well only races.
    assert names(steps, preflight.BY_LIFECYCLE) == []


def test_a_managed_node_whose_manager_is_gone_is_asked_directly():
    steps = preflight.plan(["/bt_navigator", "/amcl"])
    assert set(names(steps, preflight.BY_LIFECYCLE)) == {"bt_navigator", "amcl"}


def test_one_managers_shutdown_does_not_cover_another_managers_nodes():
    # The localization manager retires amcl. bt_navigator belongs to the
    # navigation manager, which is not up, so it still has to be asked.
    steps = preflight.plan(
        ["/lifecycle_manager_localization", "/amcl", "/bt_navigator"])
    assert names(steps, preflight.BY_MANAGER) == ["lifecycle_manager_localization"]
    assert names(steps, preflight.BY_LIFECYCLE) == ["bt_navigator"]


def test_a_behaviour_tree_sending_its_own_goals_is_stopped():
    for tree in ("seek_bt_node", "fetch_bt_node", "ostensive_bt_node",
                 "bottom_up_tree_node", "wander_between_people_node"):
        steps = preflight.plan(["/mecanumbot/" + tree])
        assert names(steps, preflight.BY_PROCESS) == [tree], tree


def test_a_second_slam_or_explorer_is_stopped():
    steps = preflight.plan(["/slam_toolbox", "/mecanumbot/mecanumbot_autoslam"])
    assert set(names(steps, preflight.BY_PROCESS)) == {
        "slam_toolbox", "mecanumbot_autoslam"}


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
    assert preflight.matches_executable(
        ["/opt/ws/install/lib/mecanumbot_seek/seek_bt_node", "--ros-args"],
        ["seek_bt_node"]) == "seek_bt_node"


def test_a_launch_file_that_merely_names_a_tree_is_not_matched():
    # Killing the launch that starts the drivers because its command line
    # mentions a tree is the accident this is written to avoid.
    assert preflight.matches_executable(
        ["ros2", "launch", "mecanumbot_seek", "launch_seek.launch.py"],
        ["seek_bt_node"]) == ""


def test_our_own_process_group_is_never_signalled():
    processes = [
        (10, 10, ["/lib/mecanumbot_seek/seek_bt_node"]),
        (11, 99, ["/lib/mecanumbot_seek/seek_bt_node"]),
    ]
    hits = preflight.processes_to_signal(
        processes, ["seek_bt_node"], keep_groups=(99,))
    assert hits == [(10, "seek_bt_node")]


def test_every_leading_condition_is_looked_for_under_one_node_name():
    # All four leading trees register as bottom_up_tree_node, so the graph
    # cannot say which executable is running.
    steps = preflight.plan(["/bottom_up_tree_node"])
    assert set(preflight.executables_to_signal(steps)) == \
        set(preflight.LEADING_EXECUTABLES)
