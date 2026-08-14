"""
Ask nav2 whether the route in a constants file can actually be driven.

`FollowRoute` sends a leg of checkpoints as one `NavigateThroughPoses` goal, and
nav2 turns that into one plan per hop: the robot to the first checkpoint, then
*from* each checkpoint to the next. That second kind is what a route can fail:
a checkpoint has to be somewhere the planner will set off from as well as aim
at, and a checkpoint pressed against a wall is neither once the costmap has
inflated it. Nav2 reports this as `ComputePathThroughPoses` returning nothing,
which reaches the behaviour tree as a goal that was accepted and then abandoned
-- true, and no use at all for working out which checkpoint is the problem.

So this asks the same questions nav2 asks itself, one at a time:

* what the **global costmap** says each checkpoint costs -- anything from 253 up
  is an obstacle as far as the planner is concerned, whatever the map looks
  like to a human eye;
* whether each **hop** plans, calling `ComputePathToPose` with `use_start` so
  that the hops starting on a checkpoint are tested exactly as nav2 runs them;
* whether each **leg** the dog tree would send plans as a whole, and then the
  whole route, through `ComputePathThroughPoses` itself.

Run it against the running stack, with the same constants file as the trial:

    ros2 run mecanumbot_leading_behaviour check_route
    ros2 run mecanumbot_leading_behaviour check_route --yaml_path /path/to.yaml

It only asks the planner for paths; it never sends a navigation goal, so it does
not move the robot.
"""

import argparse
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from rclpy.node import Node

from mecanumbot_bt_config.decoders import decode
from mecanumbot_bt_config.params import load_params
from mecanumbot_leading_behaviour.behaviours.defaults import file_constant
from mecanumbot_movement_behaviours.geometry import (
    bearing_to,
    distance_xy,
    quaternion_from_yaw,
)
from mecanumbot_movement_behaviours.pacing import route_leg
from mecanumbot_movement_behaviours.ros_interfaces import RobotPoseTracker
from mecanumbot_leading_behaviour.tree_nodes.tree_common import resolve_yaml_path

TREE_NAME = "route_check"
DEFAULT_YAML_FILENAME = "Eto_behaviour_setting_constants.yaml"

PLAN_TO_POSE_ACTION = "/compute_path_to_pose"
PLAN_THROUGH_POSES_ACTION = "/compute_path_through_poses"
GLOBAL_COSTMAP_SERVICE = "/global_costmap/get_costmap"

# nav2_costmap_2d cost values. The planner refuses to work with anything from
# INSCRIBED up: at that cost the robot's own footprint is already touching
# something, so there is no pose there for it to be at.
FREE_SPACE = 0
INSCRIBED = 253
LETHAL = 254
NO_INFORMATION = 255


def classify_cost(cost):
    """Say what a costmap value means for the planner."""
    if cost is None:
        return "off the costmap", False
    if cost == NO_INFORMATION:
        return "unknown (cost 255)", True  # passable while allow_unknown is true
    if cost >= LETHAL:
        return "OBSTACLE (cost 254)", False
    if cost >= INSCRIBED:
        return "INSCRIBED (cost 253)", False
    if cost == FREE_SPACE:
        return "free (cost 0)", True
    return f"inflated (cost {cost})", True


class RouteChecker(Node):
    """Puts the route's questions to nav2 and prints what comes back."""

    def __init__(self, yaml_path, timeout):
        super().__init__("mecanumbot_route_check")
        self.yaml_path = yaml_path
        self.timeout = timeout
        self.pose = RobotPoseTracker(self)
        self.costmap_client = self.create_client(GetCostmap, GLOBAL_COSTMAP_SERVICE)
        self.to_pose = ActionClient(self, ComputePathToPose, PLAN_TO_POSE_ACTION)
        self.through_poses = ActionClient(
            self, ComputePathThroughPoses, PLAN_THROUGH_POSES_ACTION
        )

    # --- the route ----------------------------------------------------------

    def load_route(self):
        """
        Split the constants file's checkpoints the way the trees split them.

        First entry is the start, last is the target the human is led to, and
        what is left over -- the first included -- is the route the robot drives.
        """
        params = load_params(self.yaml_path)
        points = decode(params["Dog_checkpoints"])
        return params, points[:-1], points[-1]

    # --- what nav2 thinks ---------------------------------------------------

    def costs_at(self, points):
        """Global-costmap cost under each point, or None where it cannot be read."""
        if not self.costmap_client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().warn(
                f"no {GLOBAL_COSTMAP_SERVICE} service; skipping the costmap check"
            )
            return [None] * len(points)

        future = self.costmap_client.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if future.result() is None:
            self.get_logger().warn("the costmap service did not answer")
            return [None] * len(points)

        costmap = future.result().map
        meta = costmap.metadata
        costs = []
        for point in points:
            column = int((point.x - meta.origin.position.x) / meta.resolution)
            row = int((point.y - meta.origin.position.y) / meta.resolution)
            if 0 <= column < meta.size_x and 0 <= row < meta.size_y:
                costs.append(int(costmap.data[row * meta.size_x + column]))
            else:
                costs.append(None)
        return costs

    def plan_hop(self, start, goal):
        """Plan one hop the way `ComputePathThroughPoses` plans it internally."""
        if not self.to_pose.wait_for_server(timeout_sec=self.timeout):
            return None, f"no {PLAN_TO_POSE_ACTION} action server"

        request = ComputePathToPose.Goal()
        request.goal = goal
        if start is not None:
            request.start = start
            request.use_start = True
        return self._run(self.to_pose, request)

    def plan_through(self, goals):
        """Plan a whole leg, the goal the tree actually sends."""
        if not self.through_poses.wait_for_server(timeout_sec=self.timeout):
            return None, f"no {PLAN_THROUGH_POSES_ACTION} action server"

        request = ComputePathThroughPoses.Goal()
        request.goals = goals
        return self._run(self.through_poses, request)

    def _run(self, client, request):
        """Send a planning goal and wait for its path; returns (poses, error)."""
        send = client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, send, timeout_sec=self.timeout)
        handle = send.result()
        if handle is None:
            return None, "the planner never answered the request"
        if not handle.accepted:
            return None, "the planner rejected the request"

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=self.timeout)
        if result.result() is None:
            return None, f"the planner produced nothing within {self.timeout:.0f} s"

        path = result.result().result.path.poses
        if not path:
            return None, "the planner returned an empty path"
        return path, None

    # --- poses --------------------------------------------------------------

    def stamped(self, point, facing=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.orientation = quaternion_from_yaw(
            0.0 if facing is None else bearing_to(point, facing)
        )
        return pose

    def robot_pose(self, wait=3.0):
        """Wait for AMCL to say where the robot is, and return that pose."""
        deadline = self.get_clock().now().nanoseconds / 1e9 + wait
        while self.pose.pose is None:
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose.pose


def path_length(poses):
    return sum(
        distance_xy(a.pose.position, b.pose.position) for a, b in zip(poses, poses[1:])
    )


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Check that nav2 can drive the route in a behaviour constants file."
    )
    parser.add_argument("--yaml_path", type=str, default=None)
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for each planner answer",
    )
    parsed, _ = parser.parse_known_args()

    yaml_path = parsed.yaml_path or os.getenv("YAML_PATH") or os.getenv(
        "BEHAVIOUR_YAML_PATH"
    )
    if not yaml_path:
        yaml_path = resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)

    rclpy.init(args=args)
    node = RouteChecker(yaml_path, parsed.timeout)
    try:
        failures = run_checks(node, yaml_path)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failures else 0


def run_checks(node, yaml_path):
    """Work through the route and print what nav2 says; returns the failures."""
    params, route, target = node.load_route()
    failures = []

    print(f"\nRoute from {yaml_path}")
    print(f"{len(route)} checkpoints to drive, then the target at "
          f"({target.x:.2f}, {target.y:.2f})\n")

    # --- 1. what the costmap says each checkpoint is ------------------------
    print("Checkpoints in the global costmap")
    print("-" * 62)
    for index, (point, cost) in enumerate(zip(route, node.costs_at(route))):
        verdict, plannable = classify_cost(cost)
        mark = "  " if plannable else "!!"
        print(f"{mark} {index:>2}  ({point.x:7.2f}, {point.y:7.2f})  {verdict}")
        if not plannable:
            failures.append(
                f"checkpoint {index} is {verdict} -- nav2 will not plan from or to it"
            )
    print()

    # --- 2. the hops nav2 plans one at a time -------------------------------
    # The first hop starts at the robot; every later one starts on a
    # checkpoint, which is the case a single goal never exercises.
    print("Hops (as ComputePathThroughPoses plans them internally)")
    print("-" * 62)
    robot = node.robot_pose()
    if robot is None:
        print("   no /amcl_pose yet, so the robot -> checkpoint 0 hop is skipped")
    else:
        goal = node.stamped(route[0], facing=route[min(1, len(route) - 1)])
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = robot
        report_hop(node, "robot", "0", start, goal, failures)

    for index in range(len(route) - 1):
        here, next_point = route[index], route[index + 1]
        start = node.stamped(here, facing=next_point)
        beyond = route[index + 2] if index + 2 < len(route) else target
        report_hop(
            node,
            str(index),
            str(index + 1),
            start,
            node.stamped(next_point, facing=beyond),
            failures,
        )
    print()

    # --- 3. the legs the tree actually sends --------------------------------
    print("Legs (the NavigateThroughPoses goals the dog tree sends)")
    print("-" * 62)
    lookahead = file_constant(params, "route_lookahead")
    every = file_constant(params, "check_in_every_checkpoints")
    index, seen = 0, set()
    while index < len(route):
        leg = route_leg(index, len(route) - 1, 0, every, lookahead)
        if tuple(leg) in seen:
            break
        seen.add(tuple(leg))
        goals = [
            node.stamped(
                route[i], facing=route[i + 1] if i + 1 < len(route) else target
            )
            for i in leg
        ]
        poses, error = node.plan_through(goals)
        if error:
            print(f"!! checkpoints {leg}: {error}")
            failures.append(f"leg {leg} could not be planned ({error})")
        else:
            print(f"   checkpoints {leg}: planned, {path_length(poses):.2f} m")
        index = leg[-1] + 1
    print()

    # --- verdict ------------------------------------------------------------
    print("=" * 62)
    if not failures:
        print("nav2 can drive every checkpoint, hop and leg of this route.")
        return failures

    print(f"{len(failures)} problem(s):\n")
    for failure in failures:
        print(f"  * {failure}")
    print(
        "\nA checkpoint the planner will not set off from stops the whole leg, "
        "because\nComputePathThroughPoses plans out of each checkpoint in turn. "
        "Move it into\nopen space -- the inflation radius is how far from a wall "
        "'open' starts -- or\nlower inflation_radius in the nav2 costmap "
        "parameters."
    )
    return failures


def report_hop(node, from_label, to_label, start, goal, failures):
    poses, error = node.plan_hop(start, goal)
    straight = distance_xy(start.pose.position, goal.pose.position)
    if error:
        print(f"!! {from_label:>5} -> {to_label:<3} {straight:5.2f} m apart: {error}")
        failures.append(f"nav2 cannot plan {from_label} -> {to_label} ({error})")
        return
    length = path_length(poses)
    detour = length / straight if straight > 0.01 else 1.0
    note = "" if detour < 1.6 else f"   <- {detour:.1f}x the straight line, it goes round"
    print(f"   {from_label:>5} -> {to_label:<3} {straight:5.2f} m apart: "
          f"planned {length:5.2f} m{note}")


if __name__ == "__main__":
    sys.exit(main())
