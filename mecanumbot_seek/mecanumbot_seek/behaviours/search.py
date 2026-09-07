"""
Looking for the object when it is not where it was supposed to be.

One behaviour, and it is where the SEEKING model earns its place. `SearchAround`
walks the expanding ring pattern from `search_patterns.py` centred on the last
known location, and two things about how it walks come from the circuit rather
than from a schedule:

* **how wide it goes.** The rings are rebuilt from `drive.search_radius()`,
  which grows as expectancy falls, so a robot that still believes the object is
  nearby works a tight ring and one that has been disappointed several times
  widens out;
* **when it stops.** Not when the waypoints run out -- they never do, because a
  completed sweep is a non-reward and a non-reward rebuilds the rings wider. It
  stops when expectancy falls under the extinction threshold. The robot gives up
  because it has stopped believing the thing is findable, which is the one place
  in this system where a behaviour ends for a modelled reason rather than a
  numeric one.

It never returns SUCCESS. Succeeding would mean "I found it", and finding it is
`WatchForObject`'s job in the other branch of the parallel; the only two things
that can happen here are that the search continues, or that it is given up.
"""

import py_trees
from geometry_msgs.msg import Point, Pose

from mecanumbot_movement_behaviours.geometry import distance_xy, quaternion_from_yaw
from mecanumbot_movement_behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    STATUS_SUCCEEDED,
    Nav2PoseNavigator,
    RobotPoseTracker,
)

from mecanumbot_seek.defaults import constant, register_param_keys
from mecanumbot_seek.search_patterns import (
    expanding_search,
    nearest_unvisited,
    radii_up_to,
    sweep_complete,
)


class SearchAround(py_trees.behaviour.Behaviour):
    """
    Walk an expanding ring search around the last known place until extinction.

    RUNNING while the search continues. FAILURE when the drive extinguishes, or
    when `seek_search_timeout` runs out -- the backstop, not the mechanism.
    Never SUCCESS.
    """

    def __init__(self, name="SearchAround", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="seek_drive", access=py_trees.common.Access.READ
        )
        for key in ("seek_object_class", "seek_last_known"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        for key in ("seek_search_waypoints", "seek_search_visited"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the pose tracker and the navigator."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "seek_search_timeout")
        )
        self.first_radius = float(constant(self.blackboard, "seek_ring_first"))
        self.radius_step = float(constant(self.blackboard, "seek_ring_step"))
        self.spacing = float(constant(self.blackboard, "seek_ring_spacing"))
        self.min_stops = int(constant(self.blackboard, "seek_ring_min_stops"))
        self.max_stops = int(constant(self.blackboard, "seek_ring_max_stops"))
        self.reached = float(constant(self.blackboard, "seek_waypoint_reached"))
        self.waypoint_timeout = float(
            constant(self.blackboard, "seek_waypoint_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start a fresh search: no waypoints yet, nothing visited."""
        self.nav2.reset()
        self._start = self.node.get_clock().now()
        self._waypoint_started = None
        self._current = None
        self._sweeps = 0
        self.blackboard.seek_search_waypoints = []
        self.blackboard.seek_search_visited = set()
        self.node.get_logger().info(
            f"{self.name}: searching for '{self.blackboard.seek_object_class}'"
        )

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

    def update(self):
        """Walk the rings; rebuild them wider on each fruitless sweep."""
        drive = self.blackboard.seek_drive

        if drive.extinguished:
            self.node.get_logger().info(
                f"{self.name}: giving up -- expectancy {drive.expectancy:.2f} is "
                f"below the extinction threshold after {self._sweeps} sweep(s)"
            )
            return py_trees.common.Status.FAILURE

        if self._elapsed() > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: searched for {self.timeout:.0f} s, giving up"
            )
            return py_trees.common.Status.FAILURE

        centre = self.blackboard.seek_last_known
        if centre is None:
            self.node.get_logger().info(f"{self.name}: nowhere to search around")
            return py_trees.common.Status.FAILURE

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        waypoints = self.blackboard.seek_search_waypoints
        if not waypoints:
            waypoints = self._build_rings(centre, drive)

        position = (self.pose.pose.position.x, self.pose.pose.position.y)
        index = nearest_unvisited(
            waypoints, position, self.blackboard.seek_search_visited, self.reached
        )

        if index is None or sweep_complete(
            waypoints, self.blackboard.seek_search_visited
        ):
            return self._sweep_finished(drive)

        return self._drive_to(waypoints, index)

    # --- internals ------------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start).nanoseconds / 1e9

    def _build_rings(self, centre, drive):
        """Lay out the rings out to the radius the current expectancy asks for."""
        radius = drive.search_radius()
        radii = radii_up_to(radius, self.first_radius, self.radius_step)
        waypoints = expanding_search(
            (centre.x, centre.y),
            radii,
            spacing=self.spacing,
            minimum=self.min_stops,
            maximum=self.max_stops,
        )
        self.blackboard.seek_search_waypoints = waypoints
        self.blackboard.seek_search_visited = set()
        self._current = None
        self.node.get_logger().info(
            f"{self.name}: sweep {self._sweeps + 1} -- {len(waypoints)} stop(s) on "
            f"{len(radii)} ring(s) out to {radius:.1f} m "
            f"(expectancy {drive.expectancy:.2f})"
        )
        return waypoints

    def _sweep_finished(self, drive):
        """
        Record that a whole sweep found nothing: frustrative non-reward.

        The event that lowers expectancy, and therefore both the reason the next
        sweep is wider and, eventually, the reason there is no next sweep.
        """
        self._sweeps += 1
        drive.non_reward()
        self.node.get_logger().info(
            f"{self.name}: sweep {self._sweeps} found nothing; expectancy now "
            f"{drive.expectancy:.2f}"
        )
        self.nav2.cancel()
        self.blackboard.seek_search_waypoints = []
        self.blackboard.seek_search_visited = set()
        self._current = None
        return py_trees.common.Status.RUNNING

    def _drive_to(self, waypoints, index):
        """Send or hold the goal for one search waypoint."""
        if index != self._current:
            self._send_waypoint(waypoints, index)
            return py_trees.common.Status.RUNNING

        if not self.nav2.server_ready():
            self.feedback_message = "waiting for the nav2 action server"
            return py_trees.common.Status.RUNNING

        held = (self.node.get_clock().now() - self._waypoint_started).nanoseconds / 1e9
        status = self.nav2.status()

        if status == STATUS_SUCCEEDED or self._at(waypoints[index]):
            self.blackboard.seek_search_visited.add(index)
            self._current = None
            return py_trees.common.Status.RUNNING

        if held > self.waypoint_timeout or (
            status is not None and status not in GOAL_ACTIVE_STATUSES
        ):
            # A waypoint nav2 cannot reach is written off rather than retried:
            # it is inside a table or behind a wall, and the search has more
            # places to be. The sweep still counts as covered.
            self.node.get_logger().info(
                f"{self.name}: waypoint {index} unreachable, moving on"
            )
            self.blackboard.seek_search_visited.add(index)
            self._current = None
            return py_trees.common.Status.RUNNING

        visited = len(self.blackboard.seek_search_visited)
        self.feedback_message = f"waypoint {visited + 1}/{len(waypoints)}"
        return py_trees.common.Status.RUNNING

    def _send_waypoint(self, waypoints, index):
        x, y, yaw = waypoints[index]
        goal = Pose()
        goal.position.x = float(x)
        goal.position.y = float(y)
        goal.orientation = quaternion_from_yaw(yaw)
        self.nav2.go_to(goal)
        self._current = index
        self._waypoint_started = self.node.get_clock().now()
        self.feedback_message = f"driving to waypoint {index}"

    def _at(self, waypoint):
        x, y, _ = waypoint
        return (
            distance_xy(self.pose.pose.position, Point(x=float(x), y=float(y)))
            <= self.reached
        )
