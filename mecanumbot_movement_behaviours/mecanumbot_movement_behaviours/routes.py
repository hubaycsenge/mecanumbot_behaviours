"""
Driving a route of checkpoints, and searching that same route for a lost human.

Three things live here, and they are all about a list of checkpoints rather than
a single place:

* `FollowRoute` -- one leg of the route as a single nav2 waypoint goal;
* `WaitForPerson` / `ManageSearchCheckpoint` -- the "where did my human go"
  recovery patrol, which walks the same list backwards and forwards;
* `CheckRobotAtLastCheckpoint` -- whether the end of it has been reached.

None of them writes down what the route is *called*. The blackboard keys come
from a `KeyMap`, so the leading conditions' `Dog_checkpoints` and any other
experiment's spelling are both just a spelling -- see `keys.py`.
"""

import py_trees

from mecanumbot_movement_behaviours.defaults import (
    LOOK_DOWN_KEYS,
    constant,
    register_param_keys,
    resolve,
)
from mecanumbot_movement_behaviours.geometry import (
    at_route_end,
    closest_checkpoint_index,
    distance_xy,
    path_progress_sign,
    pose_to_goal,
    route_poses,
)
from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS
from mecanumbot_movement_behaviours.pacing import route_leg
from mecanumbot_movement_behaviours.ros_interfaces import (
    AccessoryCommander,
    FollowedSubjectTracker,
    GOAL_ACTIVE_STATUSES,
    HEAD_SEEK,
    HeadLookDown,
    Nav2PoseNavigator,
    Nav2RouteNavigator,
    PeopleTracker,
    RobotPoseTracker,
    STATUS_CANCELING,
    STATUS_SUCCEEDED,
    VelocityCommander,
)

SEARCH_BACKWARDS = -1  # back down the route, where the human was last following
SEARCH_FORWARDS = 1
# Seconds a stalled goal's cancel is waited on before the replacement goes out.
STALL_CANCEL_WAIT = 3.0


class FollowRoute(py_trees.behaviour.Behaviour):
    """
    Lead one leg of the route: several checkpoints in a single nav2 goal.

    Leading used to be one nav2 goal per checkpoint, which meant the robot came
    to a full stop, waited for the goal to be reported done, turned, and set off
    again at every one of them. A leg goes through `NavigateThroughPoses`
    instead: nav2 plans through all the checkpoints of the leg at once and
    drives them without stopping, and because `route_poses()` orients every
    waypoint along the route the robot is already pointing the right way when it
    passes one.

    How long a leg is comes from `pacing.route_leg()` -- as far as the look-back
    pacing allows, so the robot never plans past a check-in it is about to owe,
    and never further than `route_lookahead` checkpoints.

    The leg is cut short, with SUCCESS, when the human stops following: they
    have been out of sight for longer than `sight_timeout`, or further away than
    the following threshold, for `check_in_grace` seconds together. SUCCESS
    rather than FAILURE because stopping the drive is not a failure of it -- the
    check-in that comes next in the cycle is due by exactly the same
    measurements, and it is the look back that decides whether this is a human
    who fell behind or a human who is gone.

    Checkpoints count as reached by distance (`checkpoint_reached_distance`),
    or when nav2 reports it has dropped them from the route, whichever comes
    first. nav2's `RemovePassedGoals` drops a waypoint once the robot is within
    its own radius (0.7 m in the stock Humble tree) and heads for the next one;
    until 2026-09-30 only the distance counted here, so a checkpoint passed at
    0.5-0.7 m stayed "current" while nav2 drove away from it, the stall watch
    below read that as no progress towards it, cancelled the leg and sent the
    robot back to it on its own (the 08:46 run of that day, twice).

    A waypoint run asks more of the route than a single goal does.
    `ComputePathThroughPoses` plans robot -> first checkpoint, then *from* that
    checkpoint to the next, so every checkpoint has to be somewhere the planner
    will both aim at and set off from -- and a checkpoint pressed against a wall
    is neither, once the costmap has inflated it. `Approach` never asked that:
    it aims the closeness threshold short of its target and stops there.

    So a leg nav2 gives up on is not retried as-is. It falls back to exactly
    that older move -- one `NavigateToPose` goal stopping short of the next
    checkpoint -- and the leg carries on checkpoint by checkpoint from there.
    Leading is then no smoother than it used to be, but it never stops dead over
    one awkwardly placed checkpoint.

    nav2 does not always give up, though: a checkpoint pressed against an
    obstacle the saved map does not have sets its recoveries backing up and
    retrying indefinitely. So a goal that has not brought the robot
    `route_stall_progress` closer to its checkpoint within `route_stall_timeout`
    is handled exactly as if nav2 had dropped it.
    """

    KEYS = DEFAULT_KEYS

    def __init__(
        self,
        name="FollowRoute",
        goal_timeout=None,
        retries=None,
        grace=None,
        sight_timeout=None,
        keys=None,
    ):
        super().__init__(name)
        self.goal_timeout = goal_timeout
        self.retries = retries
        self.grace = grace
        self.sight_timeout = sight_timeout
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.READ,
            "checkpoints",
            "max_checkpoint",
            "target_position",
            "following_threshold",
        )
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.WRITE,
            "current_checkpoint",
            "checkpoints_since",
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.goal_timeout = float(
            resolve(self.goal_timeout, self.blackboard, "approach_goal_timeout")
        )
        self.retries = int(resolve(self.retries, self.blackboard, "nav_goal_retries"))
        self.grace = float(resolve(self.grace, self.blackboard, "check_in_grace"))
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.subject = FollowedSubjectTracker(self.node, self.sight_timeout)
        self.route = Nav2RouteNavigator(self.node)
        self.single = Nav2PoseNavigator(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.route.reset()
        self.single.reset()
        self.nav2 = self.route
        self._leg = None
        self._step = 0
        self._sent_step = 0
        self._sent_count = 0
        self._resends = 0
        self._lagging_since = None
        self._start_time = self.node.get_clock().now()
        self._reset_stall_watch()

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.route.cancel()
            self.single.cancel()

    def update(self):
        if self.pose.position is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._leg is None:
            if not self.route.server_ready() or not self.single.server_ready():
                if self._elapsed() > self.goal_timeout:
                    self.node.get_logger().info(
                        f"{self.name}: no nav2 waypoint action server within "
                        f"{self.goal_timeout:.0f} s, giving up"
                    )
                    return py_trees.common.Status.FAILURE
                self.feedback_message = "waiting for the nav2 waypoint action server"
                return py_trees.common.Status.RUNNING
            return self._begin_leg()

        self._mark_checkpoints_passed()
        if self._step >= len(self._leg):
            return self._leg_done("leg driven")

        lagging = self._lagging_reason()
        if lagging is not None:
            return self._leg_done(f"stopping the leg early: {lagging}")

        status = self.nav2.status()
        if status is None:
            if self.nav2.seconds_since_send() > self.goal_timeout:
                self.node.get_logger().info(
                    f"{self.name}: nav2 never reported our route within "
                    f"{self.goal_timeout:.0f} s, giving up"
                )
                return py_trees.common.Status.FAILURE
            self.feedback_message = "waiting for nav2 to accept the route"
            return py_trees.common.Status.RUNNING

        if status == STATUS_SUCCEEDED:
            # Take nav2's word for what it drove, even where the robot ended up
            # further from the checkpoint than our own measure counts as
            # reached: a goal it says it finished must not be sent again, or the
            # same instant success comes straight back.
            if self.nav2 is self.route:
                while self._step < len(self._leg):
                    self._advance()
                return self._leg_done("nav2 finished the route")

            # One checkpoint of a leg being walked a checkpoint at a time.
            self._advance()
            if self._step >= len(self._leg):
                return self._leg_done("leg driven one checkpoint at a time")
            return self._send_single()
        if status in GOAL_ACTIVE_STATUSES:
            if self._cancel_pending():
                self.feedback_message = "waiting for nav2 to drop the stalled goal"
                return py_trees.common.Status.RUNNING
            if not self._stalled():
                self.feedback_message = self._progress()
                return py_trees.common.Status.RUNNING
            self.node.get_logger().warn(
                f"{self.name}: no progress towards checkpoint "
                f"{self._heading_for()} in {self._stall_timeout():.0f} s, "
                "treating the goal as dropped"
            )
            self.nav2.cancel()
            self._cancelled_at = self.node.get_clock().now()
            return py_trees.common.Status.RUNNING
        if status == STATUS_CANCELING and self._cancel_pending():
            # bt_navigator refuses a goal on one action while the other is
            # still running, so the replacement waits for the cancel to land.
            return py_trees.common.Status.RUNNING

        # Nav2 gave up on the goal, or it stalled. Sending the same one again
        # only buys the same answer, so the first drop steps down to the
        # single-goal move that stops short of the checkpoint; only that one is
        # worth retrying.
        if self.nav2 is self.route:
            self.node.get_logger().warn(
                f"{self.name}: nav2 could not drive the leg as waypoints -- check "
                "whether the checkpoints themselves are clear of the costmap "
                "inflation; leading to them one at a time instead"
            )
            return self._send_single()

        if self._resends >= self.retries:
            self.node.get_logger().info(
                f"{self.name}: nav2 dropped the goal {self._resends} time(s), giving up"
            )
            return py_trees.common.Status.FAILURE
        self._resends += 1
        self.node.get_logger().info(
            f"{self.name}: nav2 dropped the goal, sending it again "
            f"({self._resends}/{self.retries})"
        )
        return self._send_single()

    # --- internals ----------------------------------------------------------

    def _elapsed(self):
        return (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _checkpoints(self):
        return self.blackboard.get(self.keys.checkpoints)

    def _last_index(self):
        return self.blackboard.get(self.keys.max_checkpoint)

    def _begin_leg(self):
        """Work out how far this leg goes, and send it."""
        if not self._checkpoints():
            self.node.get_logger().warn(f"{self.name}: no route checkpoints to lead to")
            return py_trees.common.Status.FAILURE

        self._leg = route_leg(
            self.blackboard.get(self.keys.current_checkpoint),
            self._last_index(),
            self.blackboard.get(self.keys.checkpoints_since),
            constant(self.blackboard, "check_in_every_checkpoints"),
            constant(self.blackboard, "route_lookahead"),
        )
        self._step = 0

        # Checkpoints the robot is already standing on are not driven past, so
        # they are dropped from the goal and left out of the look-back pacing:
        # waiting at the end of the route is not covering ground, and counting
        # it as ground covered would have the robot glancing back on the spot.
        self._mark_checkpoints_passed(count=False)
        if self._step >= len(self._leg):
            return self._leg_done("already at the end of this leg")
        return self._send(self._leg[self._step :])

    def _send(self, indices):
        self._reset_stall_watch()
        self.nav2 = self.route
        self._sent_step = self._step
        self._sent_count = len(indices)
        self.route.follow(
            route_poses(
                self._checkpoints(),
                indices,
                look_beyond=self.blackboard.get(self.keys.target_position),
            )
        )
        self.node.get_logger().info(
            f"{self.name}: leading through checkpoint(s) {indices} of "
            f"0..{self._last_index()}"
        )
        return py_trees.common.Status.RUNNING

    def _send_single(self):
        """Fall back to `Approach`'s move: one goal, stopping short of the checkpoint."""
        index = self._heading_for()
        checkpoint = self._checkpoints()[index]
        self._reset_stall_watch()
        self.nav2 = self.single
        self.single.go_to(
            pose_to_goal(
                checkpoint,
                self.pose.pose,
                stop_threshold=constant(self.blackboard, "route_stop_distance"),
                go_threshold=constant(self.blackboard, "route_step_distance"),
            )
        )
        self.node.get_logger().info(
            f"{self.name}: leading to checkpoint {index} on its own"
        )
        return py_trees.common.Status.RUNNING

    def _heading_for(self):
        """Index of the checkpoint the leg is currently driving to."""
        return self._leg[min(self._step, len(self._leg) - 1)]

    def _stall_timeout(self):
        return float(constant(self.blackboard, "route_stall_timeout"))

    def _cancel_pending(self):
        """Say whether a stall cancel is recent enough to keep waiting on."""
        if self._cancelled_at is None:
            return False
        waited = (self.node.get_clock().now() - self._cancelled_at).nanoseconds / 1e9
        return waited < STALL_CANCEL_WAIT

    def _reset_stall_watch(self):
        self._cancelled_at = None
        self._closest = None
        self._closest_time = None
        self._closest_index = None

    def _stalled(self):
        """Say whether the robot has stopped getting closer to its checkpoint."""
        index = self._heading_for()
        distance = distance_xy(self.pose.position, self._checkpoints()[index])
        now = self.node.get_clock().now()
        progress = float(constant(self.blackboard, "route_stall_progress"))
        if (
            self._closest is None
            or index != self._closest_index
            or distance < self._closest - progress
        ):
            self._closest = distance
            self._closest_time = now
            self._closest_index = index
            return False
        return (now - self._closest_time).nanoseconds / 1e9 > self._stall_timeout()

    def _progress(self):
        index = self._heading_for()
        if self.nav2 is not self.route:
            return f"leading to checkpoint {index}"
        remaining = self.route.poses_remaining
        return f"leading to checkpoint {index}" + (
            "" if remaining is None else f", {remaining} waypoint(s) left"
        )

    def _mark_checkpoints_passed(self, count=True):
        """Walk the route index on for every checkpoint of the leg driven past."""
        checkpoints = self._checkpoints()
        reached = constant(self.blackboard, "checkpoint_reached_distance")
        while self._step < len(self._leg):
            index = self._leg[self._step]
            if distance_xy(self.pose.position, checkpoints[index]) > reached:
                break
            self._advance(count=count)

        # Waypoints nav2 has already dropped from the goal are behind us too.
        passed_by_nav2 = self._passed_by_nav2()
        while self._step < min(self._sent_step + passed_by_nav2, len(self._leg)):
            self._advance(count=count)

    def _passed_by_nav2(self):
        """Return how many waypoints of the running route goal nav2 has dropped."""
        if self.nav2 is not self.route or self._leg is None:
            return 0
        remaining = self.route.poses_remaining
        if remaining is None:
            return 0
        return max(0, self._sent_count - remaining)

    def _advance(self, count=True):
        """Put one checkpoint of the leg behind us."""
        index = self._leg[self._step]
        self._step += 1
        if count:
            self.blackboard.set(
                self.keys.checkpoints_since,
                self.blackboard.get(self.keys.checkpoints_since) + 1,
            )
        next_index = min(index + 1, self._last_index())
        self.blackboard.set(self.keys.current_checkpoint, next_index)
        self.node.get_logger().info(
            f"{self.name}: checkpoint {index} "
            f"{'reached' if count else 'already behind us'}, next is {next_index}"
        )

    def _lagging_reason(self):
        """Why the human is no longer following, once they have been for `grace`."""
        reason = None
        age = self.subject.age
        if self.subject.position is None or age is None:
            reason = "the human has not been seen yet"
        elif age > self.sight_timeout:
            # Out of sight is a reason to stop and look back, not to drive on
            # until the distance to where they were last seen runs out: until
            # 2026-09-30 only that distance counted, and in the 08:46 run the
            # robot led for 15 s on a human it had last seen 1.3 m away.
            reason = f"the human has been out of sight for {age:.1f} s"
        else:
            distance = distance_xy(self.pose.position, self.subject.position)
            allowed = self.blackboard.get(self.keys.following_threshold)
            if distance > allowed:
                reason = (
                    f"the human is {distance:.2f} m behind, "
                    f"more than the {allowed:.2f} m allowed"
                )

        if reason is None:
            self._lagging_since = None
            return None

        # A human who steps behind a pillar for a moment is still following, so
        # the reason has to hold for a while before it stops the leg.
        now = self.node.get_clock().now()
        if self._lagging_since is None:
            self._lagging_since = now
            return None
        if (now - self._lagging_since).nanoseconds / 1e9 < self.grace:
            self.feedback_message = f"{reason} (waiting {self.grace:.0f} s)"
            return None
        return reason

    def _leg_done(self, reason):
        self.nav2.cancel()
        self.node.get_logger().info(f"{self.name}: {reason}")
        self.feedback_message = reason
        return py_trees.common.Status.SUCCESS


class CheckRobotAtLastCheckpoint(py_trees.behaviour.Behaviour):
    """
    SUCCESS when the robot stands at the last checkpoint of the route.

    Standing there is a matter of where the robot is (`at_route_end()`), not
    only of the route index. The index is the checkpoint the robot is heading
    *for*, so it already reads "last" for the whole of the final stretch -- and
    from wherever `DogResumeLeading` picks the last checkpoint to lead on to.
    Until 2026-10-04 the index alone was the test, and the dog tree showed the
    target, gripper gesture and all, from the checkpoint before the last one.

    `by_index=False` leaves the index out, for a tree that drives to the end of
    the route in one `Approach` and never walks it.
    """

    KEYS = DEFAULT_KEYS

    def __init__(self, name="CheckRobotAtLastCheckpoint", by_index=True, keys=None):
        super().__init__(name)
        self.by_index = by_index
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.READ,
            "checkpoints",
            "current_checkpoint",
            "max_checkpoint",
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        if self.by_index:
            current = self.blackboard.get(self.keys.current_checkpoint)
            last = self.blackboard.get(self.keys.max_checkpoint)
            if current < last:
                self.feedback_message = f"checkpoint {current}/{last}"
                return py_trees.common.Status.FAILURE

        checkpoints = self.blackboard.get(self.keys.checkpoints)
        if self.pose.position is None or not checkpoints:
            self.feedback_message = "no pose or no route to place the robot on"
            return py_trees.common.Status.FAILURE

        distance = distance_xy(self.pose.position, checkpoints[-1])
        self.feedback_message = f"{distance:.2f} m from the last checkpoint"
        if at_route_end(
            distance,
            constant(self.blackboard, "checkpoint_reached_distance"),
            constant(self.blackboard, "route_stop_distance"),
            constant(self.blackboard, "route_end_margin"),
        ):
            self.node.get_logger().info(
                f"{self.name}: robot is at the last checkpoint ({distance:.2f} m)"
            )
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class WaitForPerson(py_trees.behaviour.Behaviour):
    """
    SUCCESS as soon as somebody is visible again.

    Runs as the interrupt branch of the recovery parallel: the patrol keeps
    searching in the other branch until this one succeeds. The head is lifted
    for the whole wait -- it reads as the robot seeking contact, and it gives the
    pose detector a full-body view instead of a pair of knees. A face at the
    bottom of the frame drops it for a few seconds and then raises it again
    (`HeadLookDown`), for somebody sitting too low for the lifted camera.
    `look_down=False` leaves that out, for a tree whose head must not nod: the
    head then only moves if a body gaze is steering it.

    On success it also records which way along the route the person turned up,
    so a later patrol starts searching in that direction.
    """

    KEYS = DEFAULT_KEYS

    def __init__(
        self, name="WaitForPerson", sight_timeout=None, keys=None, look_down=True
    ):
        super().__init__(name)
        self.sight_timeout = sight_timeout
        self.use_look_down = bool(look_down)
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "sight_timeout", *LOOK_DOWN_KEYS)
        self.keys.register(self.blackboard, py_trees.common.Access.READ, "checkpoints")
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.WRITE,
            "patrol_direction",
            "patrol_initialized",
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.pose = RobotPoseTracker(self.node)
        self.people = PeopleTracker(self.node, self.sight_timeout)
        self.velocity = VelocityCommander(self.node)
        self.accessories = AccessoryCommander(self.node)
        self.look_down = None
        if self.use_look_down:
            self.look_down = HeadLookDown(
                self.node,
                self.accessories,
                *(constant(self.blackboard, key) for key in LOOK_DOWN_KEYS),
            )
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.accessories.look(HEAD_SEEK)
        self.node.get_logger().info(f"{self.name}: watching for a person, head lifted")

    def terminate(self, new_status):
        if self.look_down is not None:
            self.look_down.release()

    def update(self):
        if not self.people.has_fresh_detection():
            if self.look_down is not None:
                self.look_down.update()
            self.feedback_message = "nobody visible yet"
            return py_trees.common.Status.RUNNING

        self.velocity.stop()  # the patrol branch is about to be cancelled
        self._remember_search_direction()
        self.node.get_logger().info(
            f"{self.name}: person found, interrupting the search"
        )
        return py_trees.common.Status.SUCCESS

    def _remember_search_direction(self):
        """Note whether the person showed up ahead of or behind the robot."""
        # Next patrol re-snaps to the checkpoint nearest wherever we end up.
        self.blackboard.set(self.keys.patrol_initialized, False)

        checkpoints = self.blackboard.get(self.keys.checkpoints)
        person_pose = self.people.last_seen_pose
        if self.pose.position is None or person_pose is None or not checkpoints:
            self.blackboard.set(self.keys.patrol_direction, SEARCH_BACKWARDS)
            self.node.get_logger().warn(
                f"{self.name}: no pose or checkpoints, searching backwards by default"
            )
            return

        direction = path_progress_sign(
            checkpoints, self.pose.position, person_pose.position
        )
        self.blackboard.set(self.keys.patrol_direction, direction)
        self.node.get_logger().info(
            f"{self.name}: person is "
            f"{'ahead' if direction > 0 else 'behind'} on the route"
        )


class ManageSearchCheckpoint(py_trees.behaviour.Behaviour):
    """
    Walk the patrol index along the checkpoint list.

    The first run (and every run after a person was found) snaps to the
    checkpoint nearest the robot, then each run steps one checkpoint in the
    patrol direction, reversing at either end of the route.

    The direction is set on that first run from where the human was last seen:
    back down the route if they were behind the robot, on up it if they were
    ahead. Until 2026-09-24 it was only ever written by `WaitForPerson`, when
    somebody was *found* -- so the patrol after losing the human searched the
    way the previous find had pointed. In the run of that day the tree opened
    by finding the human ahead on the route, lost them behind it five minutes
    later, and patrolled forwards, away from them, checkpoint 1 to 2 to 3.
    The direction `WaitForPerson` recorded is now only the fallback for a human
    who was never placed at all.
    """

    KEYS = DEFAULT_KEYS

    def __init__(self, name="ManageSearchCheckpoint", keys=None):
        super().__init__(name)
        self.keys = keys or self.KEYS

        self.blackboard = self.attach_blackboard_client(name=name)
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.READ,
            "checkpoints",
            "max_checkpoint",
        )
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.WRITE,
            "patrol_current_checkpoint",
            "patrol_direction",
            "patrol_initialized",
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        # No sight timeout matters here: only the last place is read, never
        # whether it is fresh -- a lost human is by definition not fresh.
        self.subject = FollowedSubjectTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        if self.pose.position is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if not self.blackboard.get(self.keys.patrol_initialized):
            checkpoints = self.blackboard.get(self.keys.checkpoints)
            index = closest_checkpoint_index(
                checkpoints, self.pose.position.x, self.pose.position.y
            )
            self.blackboard.set(self.keys.patrol_current_checkpoint, index)
            self.blackboard.set(self.keys.patrol_initialized, True)
            self.node.get_logger().info(
                f"{self.name}: search starts at checkpoint {index}, "
                f"{self._direction_from_last_sighting(checkpoints)}"
            )
            return py_trees.common.Status.SUCCESS

        last_index = self.blackboard.get(self.keys.max_checkpoint)
        index = self.blackboard.get(
            self.keys.patrol_current_checkpoint
        ) + self.blackboard.get(self.keys.patrol_direction)

        if index <= 0:
            index = 0
            self.blackboard.set(self.keys.patrol_direction, SEARCH_FORWARDS)
            self.node.get_logger().info(
                f"{self.name}: reached the start, searching forwards"
            )
        elif index >= last_index:
            index = last_index
            self.blackboard.set(self.keys.patrol_direction, SEARCH_BACKWARDS)
            self.node.get_logger().info(
                f"{self.name}: reached the end, searching backwards"
            )

        self.blackboard.set(self.keys.patrol_current_checkpoint, index)
        self.node.get_logger().info(f"{self.name}: next search checkpoint is {index}")
        return py_trees.common.Status.SUCCESS

    def _direction_from_last_sighting(self, checkpoints):
        """Point the patrol at where the human was last seen; say which way."""
        last_position = self.subject.position
        if last_position is None or not checkpoints:
            direction = self.blackboard.get(self.keys.patrol_direction)
            return (
                f"searching {'forwards' if direction > 0 else 'backwards'} "
                "(the human was never placed)"
            )
        direction = path_progress_sign(checkpoints, self.pose.position, last_position)
        self.blackboard.set(self.keys.patrol_direction, direction)
        return (
            f"searching {'forwards' if direction > 0 else 'backwards'}: the human "
            f"was last seen {'ahead of' if direction > 0 else 'behind'} the robot"
        )
