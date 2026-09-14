"""
Looking for the ball: watching for it, circling for it, and sweeping the head.

Three behaviours, and they run together rather than in turn.

`WatchForBall` is the one that ends the search. It runs in parallel with
everything else for the whole search, so wherever the robot happens to be --
halfway round a circle, driving out to a wider one -- seeing the ball ends the
parallel and the tree drops into the approach. Checking for the ball between
search steps instead would mean the robot drives straight past a ball it can see
and notices at the next stop.

`CircleSearch` is where it drives, and `SweepHead` is where it points its head.
(`HopToNextSpot` is `CircleSearch` cut into single drives, for the search that
turns a full circle at each place it stops.)
They are separate because they search different dimensions and neither can do
the other's job: circling covers the floor, sweeping covers the *band* of that
floor the camera can see at all. A ball outside the current tilt's band is
invisible however good the detector is, so a search with a fixed head is a
search that finds balls at one distance.
"""

import math

import py_trees
from geometry_msgs.msg import Point, Pose

from mecanumbot_movement_behaviours.geometry import distance_xy, quaternion_from_yaw
from mecanumbot_movement_behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    STATUS_SUCCEEDED,
    AccessoryCommander,
    Nav2PoseNavigator,
    RobotPoseTracker,
)

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import BallDetectionTracker
from mecanumbot_fetch_behaviour.defaults import constant, register_param_keys
from mecanumbot_fetch_behaviour.search_patterns import (
    FACING_TANGENT,
    expanding_circles,
    head_sweep,
    nearest_unvisited,
    radii_up_to,
    sweep_complete,
)


class WatchForBall(py_trees.behaviour.Behaviour):
    """
    Watch for a ball; SUCCESS once one has been in sight long enough.

    The dwell is short but not zero. One frame of a marginal detection is not a
    reason to abandon a search leg the robot is halfway through, and a single
    frame is exactly what noise looks like. Holding the sighting for
    `fetch_sighting_dwell` costs a fraction of a second and removes almost all
    of it.

    Never FAILURE. As the selected child of a `SuccessOnSelected` parallel, a
    failure here would end the whole search, and "I have not seen it yet" is not
    a reason to stop looking -- running out of laps is, and that belongs to
    `CircleSearch`.
    """

    def __init__(self, name="WatchForBall", dwell=None):
        super().__init__(name)
        self.dwell = dwell
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        for key in ("fetch_ball_position", "fetch_ball_score", "fetch_ball_height"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Subscribe to the located balls."""
        self.node = kwargs["node"]
        self.dwell = float(
            self.dwell
            if self.dwell is not None
            else constant(self.blackboard, "fetch_sighting_dwell")
        )
        self.threshold = float(constant(self.blackboard, "fetch_detection_threshold"))
        self.pose = RobotPoseTracker(self.node)
        self.balls = BallDetectionTracker(
            self.node,
            timeout=constant(self.blackboard, "fetch_detection_timeout"),
            class_id=str(constant(self.blackboard, "fetch_ball_class")) or None,
        )
        # A label that matches nothing drops every ball without a sound, so the
        # one in force is said at startup.
        self.node.get_logger().info(
            f"{self.name}: watching {self.balls._subscription.topic_name} for "
            + (
                f"balls labelled {self.balls.class_id!r}"
                if self.balls.class_id
                else "balls with any label"
            )
            + f" at score >= {self.threshold:.2f}"
        )
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Forget any sighting from a previous episode."""
        self._sighted_since = None

    def update(self):
        """Succeed once a ball has been continuously visible for the dwell."""
        # "Nearest" only means something once the robot knows where it is.
        if self.pose.pose is not None:
            self.balls.look_from(self.pose.pose.position)

        if not self.balls.visible(self.threshold):
            if self._sighted_since is not None:
                self.node.get_logger().info(f"{self.name}: lost sight of it again")
            self._sighted_since = None
            self.feedback_message = f"nothing at or above {self.threshold:.2f}"
            self._explain_ignored_ball()
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()
        if self._sighted_since is None:
            self._sighted_since = now
            self.node.get_logger().info(
                f"{self.name}: possible ball, score "
                f"{self.balls.hypothesis.score:.2f} >= {self.threshold:.2f}"
            )

        held = (now - self._sighted_since).nanoseconds / 1e9
        if held < self.dwell:
            self.feedback_message = f"held for {held:.1f}/{self.dwell:.1f} s"
            return py_trees.common.Status.RUNNING

        position = self.balls.hypothesis.position
        self.blackboard.fetch_ball_position = position
        self.blackboard.fetch_ball_score = float(self.balls.hypothesis.score)
        # The map frame's z is the floor plane, so the detection's z *is* the
        # ball's height above the floor. That is the number the reachability
        # check turns into "close the grabbers" or "it is not on the floor".
        self.blackboard.fetch_ball_height = float(position.z)
        self.node.get_logger().info(
            f"{self.name}: ball in sight at x={position.x:.2f} y={position.y:.2f}, "
            f"{position.z * 100:.0f} cm up"
        )
        return py_trees.common.Status.SUCCESS

    def _explain_ignored_ball(self):
        """
        Say why a ball that is being published is not being acted on.

        Silent only when `ball_detections` is silent too. A message that arrived
        in the last couple of seconds and still does not count is one of three
        things, and each points somewhere different: the label does not match
        `fetch_ball_class`, the score is under `fetch_detection_threshold`, or
        the stamp is older than `fetch_detection_timeout` although the message
        is new -- which is two clocks disagreeing, not a slow network.
        """
        received = self.balls.received_age()
        if received is None or received > 2.0:
            return
        logger = self.node.get_logger()
        hypothesis = self.balls.hypothesis
        if self.balls.last_ignored_classes and (
            hypothesis is None or not self.balls.fresh()
        ):
            reason = (
                f"it is labelled {list(self.balls.last_ignored_classes)}, and "
                f"fetch_ball_class is {self.balls.class_id!r}"
            )
        elif hypothesis is None:
            return
        elif not self.balls.fresh():
            reason = (
                f"its stamp is {self.balls.age:.2f} s old although it arrived "
                f"{received:.2f} s ago, over fetch_detection_timeout "
                f"{self.balls.timeout:.2f} s -- the publisher's clock and this "
                "node's disagree"
            )
        else:
            reason = (
                f"its score {hypothesis.score:.2f} is under "
                f"fetch_detection_threshold {self.threshold:.2f}"
            )
        logger.warn(
            f"{self.name}: ball_detections has a ball, ignored because {reason}",
            throttle_duration_sec=2.0,
        )


class SweepHead(py_trees.behaviour.Behaviour):
    """
    Tilt the head slowly up and down for as long as the search runs.

    Always RUNNING: it is a modifier on whatever its siblings are doing, and it
    must not be able to end or fail the parallel it sits in.

    The neck is commanded at `fetch_head_command_interval` rather than every
    tick, because each command is a serial write to the accessory board and the
    tree ticks at 10 Hz. The sweep itself is continuous -- the position is a
    function of elapsed time, not of how many commands have been sent -- so the
    interval changes how smooth the motion is, not how far or how fast it goes.

    On the way out the head is left at the *low* pose rather than centred: the
    behaviour that follows a successful search is the approach, which wants the
    ball in frame at close range, and that is where the head has to be anyway.
    """

    def __init__(self, name="SweepHead"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        """Build the accessory commander and read the sweep's shape."""
        self.node = kwargs["node"]
        self.low = float(constant(self.blackboard, "fetch_head_low"))
        self.high = float(constant(self.blackboard, "fetch_head_high"))
        self.period = float(constant(self.blackboard, "fetch_head_sweep_period"))
        self.interval = float(
            constant(self.blackboard, "fetch_head_command_interval")
        )
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start the sweep at the bottom of its travel."""
        self._start = self.node.get_clock().now()
        self._last_command = None
        self.node.get_logger().info(
            f"{self.name}: sweeping the head between {self.low:.1f} and "
            f"{self.high:.1f} every {self.period:.0f} s"
        )

    def terminate(self, new_status):
        """Leave the head down, which is where the approach wants it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.accessories.send(self.low)

    def update(self):
        """Command the tilt the sweep is currently at, and keep running."""
        now = self.node.get_clock().now()
        if self._last_command is not None:
            since = (now - self._last_command).nanoseconds / 1e9
            if since < self.interval:
                return py_trees.common.Status.RUNNING

        elapsed = (now - self._start).nanoseconds / 1e9
        position = head_sweep(elapsed, self.low, self.high, self.period)
        self.accessories.send(position)
        self._last_command = now
        self.feedback_message = f"head at {position:.2f}"
        return py_trees.common.Status.RUNNING


class CircleSearch(py_trees.behaviour.Behaviour):
    """
    Walk widening circles around where the robot started, until the laps run out.

    RUNNING while the search continues. FAILURE when `fetch_search_laps` whole
    laps have found nothing, or when `fetch_search_timeout` runs out -- the
    backstop, not the mechanism. Never SUCCESS: succeeding would mean "I found
    it", and finding it is `WatchForBall`'s job in the other branch of the
    parallel.

    Each lap is rebuilt from the innermost circle rather than continuing
    outwards for ever, and that is deliberate: this is a game with a person in
    it, and the ball that was not at the robot's feet a minute ago is quite
    likely to be there now because somebody threw it. A search that only ever
    widens is a search for a stationary object, which is `mecanumbot_seek`'s
    problem and not this one.
    """

    def __init__(self, name="CircleSearch", timeout=None):
        super().__init__(name)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        for key in (
            "fetch_search_waypoints",
            "fetch_search_visited",
            "fetch_laps_done",
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Build the pose tracker and the navigator, and read the pattern."""
        self.node = kwargs["node"]
        self.timeout = float(
            self.timeout
            if self.timeout is not None
            else constant(self.blackboard, "fetch_search_timeout")
        )
        self.first_radius = float(constant(self.blackboard, "fetch_circle_first"))
        self.radius_step = float(constant(self.blackboard, "fetch_circle_step"))
        self.max_radius = float(constant(self.blackboard, "fetch_circle_max"))
        self.spacing = float(constant(self.blackboard, "fetch_circle_spacing"))
        self.min_stops = int(constant(self.blackboard, "fetch_circle_min_stops"))
        self.max_stops = int(constant(self.blackboard, "fetch_circle_max_stops"))
        self.facing = str(constant(self.blackboard, "fetch_circle_facing"))
        self.reached = float(constant(self.blackboard, "fetch_waypoint_reached"))
        self.waypoint_timeout = float(
            constant(self.blackboard, "fetch_waypoint_timeout")
        )
        self.max_laps = int(constant(self.blackboard, "fetch_search_laps"))
        self.pose = RobotPoseTracker(self.node)
        self.nav2 = Nav2PoseNavigator(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start a fresh search: no centre yet, no waypoints, nothing visited."""
        self.nav2.reset()
        self._waypoint_started = None
        self._current = None
        self._start_search()

    def _start_search(self):
        self._start = self.node.get_clock().now()
        self._centre = None
        self.blackboard.fetch_search_waypoints = []
        self.blackboard.fetch_search_visited = set()
        self.blackboard.fetch_laps_done = 0

    def terminate(self, new_status):
        """Stop driving when the behaviour stops, whatever stopped it."""
        if new_status != py_trees.common.Status.RUNNING:
            self.nav2.cancel()

    def update(self):
        """Walk the circles; start the pattern over on each fruitless lap."""
        if self._elapsed() > self.timeout:
            self.node.get_logger().info(
                f"{self.name}: searched for {self.timeout:.0f} s, giving up"
            )
            return py_trees.common.Status.FAILURE

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._centre is None:
            # Centred on where the robot was standing when the search began,
            # because that is the only place it has any reason to prefer.
            self._centre = (
                self.pose.pose.position.x,
                self.pose.pose.position.y,
            )

        waypoints = self.blackboard.fetch_search_waypoints
        if not waypoints:
            waypoints = self._build_circles()

        position = (self.pose.pose.position.x, self.pose.pose.position.y)
        index = nearest_unvisited(
            waypoints, position, self.blackboard.fetch_search_visited, self.reached
        )

        if index is None or sweep_complete(
            waypoints, self.blackboard.fetch_search_visited
        ):
            return self._lap_finished()

        return self._drive_to(waypoints, index)

    # --- internals ------------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start).nanoseconds / 1e9

    def _arrived(self):
        """Keep going on reaching a stop: the circle is not over."""
        return py_trees.common.Status.RUNNING

    def _build_circles(self):
        """Lay out the circles for one lap, tightest first."""
        radii = radii_up_to(self.max_radius, self.first_radius, self.radius_step)
        waypoints = expanding_circles(
            self._centre,
            radii,
            spacing=self.spacing,
            minimum=self.min_stops,
            maximum=self.max_stops,
            facing=self.facing,
        )
        self.blackboard.fetch_search_waypoints = waypoints
        self.blackboard.fetch_search_visited = set()
        self._current = None
        self.node.get_logger().info(
            f"{self.name}: lap {self.blackboard.fetch_laps_done + 1} of "
            f"{self.max_laps} -- {len(waypoints)} stop(s) on {len(radii)} "
            f"circle(s) out to {radii[-1]:.1f} m, facing {self.facing}"
        )
        return waypoints

    def _lap_finished(self):
        """Count a whole lap that found nothing, and start over or give up."""
        self.blackboard.fetch_laps_done += 1
        done = self.blackboard.fetch_laps_done
        self.nav2.cancel()
        self.blackboard.fetch_search_waypoints = []
        self.blackboard.fetch_search_visited = set()
        self._current = None
        if done >= self.max_laps:
            self.node.get_logger().info(
                f"{self.name}: {done} lap(s) found no ball, giving up"
            )
            return py_trees.common.Status.FAILURE
        self.node.get_logger().info(
            f"{self.name}: lap {done} found no ball; going round again"
        )
        return py_trees.common.Status.RUNNING

    def _drive_to(self, waypoints, index):
        """Send or hold the goal for one stop on a circle."""
        if index != self._current:
            self._send_waypoint(waypoints, index)
            return py_trees.common.Status.RUNNING

        if not self.nav2.server_ready():
            self.feedback_message = "waiting for the nav2 action server"
            return py_trees.common.Status.RUNNING

        held = (self.node.get_clock().now() - self._waypoint_started).nanoseconds / 1e9
        status = self.nav2.status()

        if status == STATUS_SUCCEEDED or self._at(waypoints[index]):
            self.blackboard.fetch_search_visited.add(index)
            self._current = None
            return self._arrived()

        if held > self.waypoint_timeout or (
            status is not None and status not in GOAL_ACTIVE_STATUSES
        ):
            # A stop nav2 cannot reach is written off rather than retried: it is
            # inside a table or behind a wall, and the lap has more places to
            # be. It still counts as covered.
            self.node.get_logger().info(
                f"{self.name}: stop {index} unreachable, moving on"
            )
            self.blackboard.fetch_search_visited.add(index)
            self._current = None
            return py_trees.common.Status.RUNNING

        visited = len(self.blackboard.fetch_search_visited)
        self.feedback_message = f"stop {visited + 1}/{len(waypoints)}"
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
        self.feedback_message = (
            f"driving to stop {index} facing {math.degrees(yaw):.0f} deg"
        )

    def _at(self, waypoint):
        x, y, _ = waypoint
        return (
            distance_xy(self.pose.pose.position, Point(x=float(x), y=float(y)))
            <= self.reached
        )


class HopToNextSpot(CircleSearch):
    """
    Drive to the next place to turn a full circle from; SUCCESS on arrival.

    The other half of the `spin` search, which alternates a full revolution on
    the spot with one of these. The spots are laid out like `CircleSearch`'s
    stops -- rings around where the search began, nearest first -- but sparser,
    because each one is looked round from rather than looked along: one spot
    per `fetch_spot_spacing` metres of ring, not per camera width.

    The pattern outlives the behaviour. Every hop re-enters this behaviour, so
    unlike `CircleSearch` it does not start the search over on `initialise()`;
    it starts over only when the episode's search state is empty and no lap has
    been counted, which is what `ClearFetchEpisode` leaves behind. FAILURE is
    the same as `CircleSearch`'s: the laps or `fetch_search_timeout` ran out.
    """

    def setup(self, **kwargs):
        """Read `CircleSearch`'s constants, then the spot layout over them."""
        super().setup(**kwargs)
        self.first_radius = float(constant(self.blackboard, "fetch_spot_first"))
        self.radius_step = float(constant(self.blackboard, "fetch_spot_step"))
        self.max_radius = float(constant(self.blackboard, "fetch_spot_max"))
        self.spacing = float(constant(self.blackboard, "fetch_spot_spacing"))
        self.min_stops = int(constant(self.blackboard, "fetch_spot_min_stops"))
        self.max_stops = int(constant(self.blackboard, "fetch_spot_max_stops"))
        # The robot turns a full circle at every spot, so which way it arrives
        # facing is not a search choice here; facing the way it drove in is the
        # goal nav2 finishes soonest.
        self.facing = FACING_TANGENT
        self._centre = None
        return True

    def _start_search(self):
        fresh = (
            self._centre is None
            or not self.blackboard.fetch_search_waypoints
            and self.blackboard.fetch_laps_done == 0
        )
        if fresh:
            super()._start_search()

    def _arrived(self):
        visited = len(self.blackboard.fetch_search_visited)
        total = len(self.blackboard.fetch_search_waypoints)
        self.node.get_logger().info(
            f"{self.name}: at spot {visited}/{total}, turning a full circle"
        )
        return py_trees.common.Status.SUCCESS
