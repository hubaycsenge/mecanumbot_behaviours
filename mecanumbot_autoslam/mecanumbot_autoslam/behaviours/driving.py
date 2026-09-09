"""
The behaviour that moves the robot -- the only one in this package that does.

It drives nothing itself. Every translation is a nav2 `NavigateToPose` goal,
exactly as in the leading and seek trees, so the local costmap, the obstacle
avoidance and the recovery behaviours all apply to an exploration leg the same
way they apply to an experimental one. `/cmd_vel` is never published here; the
explorer has no reason to turn in place, and turning in place is the one thing
in this workspace that bypasses nav2 on purpose.

This is why the behaviour lives in `mecanumbot_behaviours` and not in
`mecanumbot_custom_nav2`. The frontier detector, the scoring, the occupancy
model and the exit criteria are judgements about a map; this is a decision to
send the robot somewhere, and that is a behaviour whatever it is implemented
with.

**One goal at a time.** A goal is held until nav2 finishes it, the robot gets
within `goal_reached_distance` of it, or `goal_timeout` passes -- and then the
choice is made again from the current map rather than from the map that was
current when the goal was sent.

**A goal nav2 gives up on is dropped, not retried.** A frontier behind a wall
the planner cannot get around is not a frontier worth keeping; the detector will
happily propose it again every cycle, and taking nav2's refusal at face value is
the cheapest way to stop that.

**A goal nav2 *rejects* is a different thing, and is not the goal's fault.**
The action server exists as soon as `bt_navigator` is constructed, but it turns
every goal down until the lifecycle manager has activated it -- which never
happens if the bringup aborted. Treated like an abort, that produced a goal per
tick, each one rejected, each one counted: thirteen "goals" in thirteen seconds,
none of which happened, and the interleaving counter running on regardless. So a
rejection backs off for `nav2_retry_delay` and is not counted as a goal at all,
and the log says what it means -- nav2 is up but not activated.
"""

import math

from mecanumbot_autoslam.behaviours import choosing
from mecanumbot_autoslam.behaviours.base import FAILURE, RUNNING, SUCCESS, Behaviour
from mecanumbot_autoslam.behaviours.ros_interfaces import ExplorationNavigator


class DriveToGoal(Behaviour):
    """Keep one nav2 goal in flight: hold the current one, or pick the next."""

    def __init__(self, name="drive_to_goal"):
        """Create the behaviour; the navigator is built in `setup()`."""
        super().__init__(name)
        self.navigator = None
        self.goal = None
        self.goal_source = ""
        self.goals_sent = 0
        self._timeout = 60.0
        self._reached = 0.6
        self._uncertain_every = 3
        self._revisit_uncertain = True
        self._retry_delay = 5.0
        #: When the next goal may be sent, after nav2 turned one down.
        self._blocked_until = 0.0

    def setup(self, node, params):
        """Build the nav2 action client and cache the goal constants."""
        super().setup(node, params)
        self.navigator = ExplorationNavigator(node, params["nav2_action"])
        self._timeout = float(params["goal_timeout"])
        self._reached = float(params["goal_reached_distance"])
        self._uncertain_every = int(params["uncertain_every"])
        self._revisit_uncertain = bool(params["revisit_uncertain"])
        self._retry_delay = float(params["nav2_retry_delay"])

    def update(self, context, revisit_points=()):
        """Hold the goal in flight, or choose and send the next one."""
        if not self.navigator.server_ready():
            self.node.get_logger().warn("nav2 action server not ready", once=True)
            return FAILURE

        if self._in_flight(context):
            context.goal = self.goal
            context.goal_source = self.goal_source
            return RUNNING

        if context.now < self._blocked_until:
            return FAILURE

        point, source = choosing.select(
            context.best,
            revisit_points,
            self.goals_sent,
            uncertain_every=self._uncertain_every,
            revisit_uncertain=self._revisit_uncertain,
        )
        if point is None:
            return FAILURE

        self._send(point, source)
        context.goal = self.goal
        context.goal_source = self.goal_source
        return SUCCESS

    def terminate(self):
        """Cancel whatever is in flight. Called when the pass ends."""
        if self.navigator is not None and self.navigator.goal_sent:
            self.navigator.cancel()
        self._clear()

    # --- the goal in flight ---------------------------------------------------

    def _in_flight(self, context):
        """Report whether the current goal is still worth waiting for."""
        if not self.navigator.goal_sent:
            return False

        if self.navigator.settled():
            if self.navigator.succeeded():
                self.log("arrived at the {} goal".format(self.goal_source))
            elif self.navigator.rejected():
                # Not this goal's fault: nav2 is not accepting any. The most
                # likely reason by far is a stack that came up but was never
                # activated, so say so rather than blaming the frontier.
                self.goals_sent -= 1
                self._blocked_until = context.now + self._retry_delay
                self.log(
                    "nav2 turned the goal down -- the action server is there "
                    "but not activated. Waiting {:.0f} s. If this repeats, "
                    "nav2's bringup failed; check the lifecycle manager's log "
                    "for 'Failed to bring up all requested nodes'"
                    .format(self._retry_delay)
                )
            else:
                # See the module docstring: not retried, dropped.
                self.log("nav2 gave up on the {} goal".format(self.goal_source))
            self._clear()
            return False

        elapsed = self.navigator.seconds_since_send()
        if elapsed > self._timeout:
            self.log(
                "goal timed out after {:.0f} s, cancelling and re-deciding"
                .format(elapsed)
            )
            self.navigator.cancel()
            self._clear()
            return False

        if self.goal is not None and context.robot_xy is not None:
            reached = math.hypot(
                self.goal[0] - context.robot_xy[0],
                self.goal[1] - context.robot_xy[1],
            )
            if reached <= self._reached:
                # Close enough. The goal is let go of but not cancelled: a
                # frontier has no meaningful orientation, so there is nothing
                # to wait for nav2 to park on, and the next goal preempts this
                # one the moment there is one.
                self.log("close enough to the {} goal".format(self.goal_source))
                self._clear()
                return False
        return True

    def _send(self, point, source):
        """Send one goal and start following it."""
        self.goal = (float(point[0]), float(point[1]))
        self.goal_source = source
        self.goals_sent += 1
        self.navigator.go_to_point(self.goal)
        self.log(
            "goal #{} ({}) at x={:.2f} y={:.2f}".format(
                self.goals_sent, source, self.goal[0], self.goal[1])
        )

    def _clear(self):
        """Stop following the goal, without cancelling it."""
        self.goal = None
        self.goal_source = ""
        self.navigator.reset()
