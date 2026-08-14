import argparse

import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from mecanumbot_demo_behaviours.behaviours.movement_managers import (
    DEMO_DEFAULTS,
    ProcessDetections,
    GetNextWaypoint,
)
from mecanumbot_demo_behaviours.behaviours.blackboard_managers import (
    MapWaypointsToBlackboard,
)
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS


def get_map_name():
    """
    `--map_name`, else the `demo_map_name` default.

    This tree loads no constants YAML -- it has no route, no gestures and no
    thresholds to read -- so the map it patrols is named on the command line
    rather than on the blackboard.
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--map_name", type=str, default=None)
    parsed, _ = parser.parse_known_args()
    return parsed.map_name or DEMO_DEFAULTS["demo_map_name"]


def create_tree(map_name=None):
    # 1. The Root Priority Selector
    root = py_trees.composites.Selector(name="Priority_Root", memory=False)
    map_waypoints_manager = MapWaypointsToBlackboard(
        base_name=map_name if map_name is not None else get_map_name()
    )
    # 2. INTERCEPT BRANCH

    intercept_sequence = py_trees.composites.Sequence(name="Intercept_Seq", memory=True)

    check_detections = ProcessDetections()

    # py_trees_ros ActionClient automatically handles goal cancellation!
    nav_to_intercept = py_trees_ros.actions.ActionClient(
        name="Nav_to_Intercept",
        action_type=NavigateToPose,
        action_name="/navigate_to_pose",
        # It pulls the goal message directly from this blackboard key
        action_goal_key="intercept_goal",
        generate_feedback_message=None,
    )
    intercept_sequence.add_children([check_detections, nav_to_intercept])

    # 3. PATROL BRANCH
    patrol_sequence = py_trees.composites.Sequence(name="Patrol_Seq", memory=True)

    get_waypoint = GetNextWaypoint()

    nav_to_patrol = py_trees_ros.actions.ActionClient(
        name="Nav_to_Patrol",
        action_type=NavigateToPose,
        action_name="/navigate_to_pose",
        action_goal_key="patrol_goal",
        generate_feedback_message=None,
    )

    patrol_sequence.add_children([get_waypoint, nav_to_patrol])

    # 4. Put it all together
    root.add_children([map_waypoints_manager, intercept_sequence, patrol_sequence])
    return root


# ---------------------------------------------------------
# 4. Main Execution
# ---------------------------------------------------------
def main():
    rclpy.init()

    tree = create_tree()

    # Wrap the py_trees tree in a ROS 2 tree manager
    ros_tree = py_trees_ros.trees.BehaviourTree(root=tree, unicode_tree_debug=True)

    # Add a subscriber to continuously push the PoseArray to the Blackboard
    ros_tree.add_pre_tick_handler(
        py_trees_ros.subscribers.ToBlackboard(
            name="PoseArray_Sub",
            topic_name="/detections",
            topic_type=PoseArray,
            blackboard_variables={"pose_array": None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        ).setup
    )

    # Setup and tick the tree at the library's standard rate
    ros_tree.setup(timeout=float(RUNTIME_DEFAULTS["setup_timeout"]))
    ros_tree.tick_tock(period_ms=float(RUNTIME_DEFAULTS["tick_period_ms"]))

    rclpy.spin(ros_tree.node)
    rclpy.shutdown()
