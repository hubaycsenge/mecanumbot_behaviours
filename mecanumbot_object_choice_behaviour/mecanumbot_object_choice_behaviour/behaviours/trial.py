"""
The parts of an object-choice trial that neither borrowed tree has.

Everything that moves the robot towards a person or a brick is the ostensive or
the fetch package's. What is here is the trial around it:

* `RememberStartPose` -- where the robot stands when the run begins, which is
  where every trial starts from;
* `WaitForGo` and `StartTrial` -- the experimenter sets the bricks out, says go,
  and a numbered trial begins;
* `SurveyBricks` -- with the head down, which bricks are on the floor and where,
  remembered in the map before the head comes up to read the person;
* `ChooseBrick` -- the decision, from the cue and the survey (`choice_geometry`);
* `LookAtBrick`, `NoteTrial`, `ReturnToStart`;
* `FinishTrial` -- one JSON record per trial on `/mecanumbot/object_choice/trial`,
  whatever happened, with the outcome read off how far the trial got.

**The record is the experiment's data.** A trial in which the robot abstained,
or never saw a cue, is a trial: it goes out with its reason, and an analysis
that drops it does so on purpose rather than because the robot never said.
"""

import math

import py_trees
from geometry_msgs.msg import Point, Pose

from mecanumbot_fetch_behaviour.behaviours.ros_interfaces import GripperCommander
from mecanumbot_fetch_behaviour.defaults import constant as fetch_constant
from mecanumbot_movement_behaviours.defaults import (
    register_param_keys as register_movement_keys,
)
from mecanumbot_movement_behaviours.geometry import distance_xy
from mecanumbot_movement_behaviours.ros_interfaces import (
    GOAL_ACTIVE_STATUSES,
    STATUS_SUCCEEDED,
    Nav2PoseNavigator,
    RobotPoseTracker,
)

from mecanumbot_object_choice_behaviour import choice_geometry
from mecanumbot_object_choice_behaviour.behaviours.blackboard_managers import (
    register_trial_keys,
)
from mecanumbot_object_choice_behaviour.behaviours.ros_interfaces import (
    BrickTracker,
    GoSignal,
    StatePublisher,
    TargetRelay,
    TrialRecordPublisher,
)
from mecanumbot_object_choice_behaviour.defaults import constant, register_param_keys

# Outcomes, in the order a trial can get stuck.
OUTCOME_COMPLETED = "completed"
OUTCOME_NO_BRICKS = "no_bricks"
OUTCOME_NOT_ADDRESSED = "not_addressed"
OUTCOME_NO_CUE = "no_cue"
OUTCOME_NO_CHOICE = "no_choice"
OUTCOME_RESPONSE_FAILED = "response_failed"
OUTCOME_RETURN_FAILED = "return_failed"


def _seconds(node):
    return node.get_clock().now().nanoseconds / 1e9


def _xyz(point):
    return [round(float(point.x), 3), round(float(point.y), 3), round(float(point.z), 3)]


class TrialPhase(py_trees.behaviour.Behaviour):
    """Publish one phase label on the object-choice state topic; always SUCCESS."""

    def __init__(self, name, phase):
        super().__init__(name)
        self.phase = phase

    def setup(self, **kwargs):
        """Build the state publisher."""
        self.node = kwargs["node"]
        self.state = StatePublisher(self.node)
        return True

    def update(self):
        """Announce the phase."""
        self.state.publish(self.phase)
        self.feedback_message = self.phase
        return py_trees.common.Status.SUCCESS


class RememberStartPose(py_trees.behaviour.Behaviour):
    """
    Record where the robot stands when the run begins; RUNNING until AMCL says.

    Only the first time: every trial returns to this pose, so the layout the
    experimenter measured out stays the one the robot starts from.
    """

    def __init__(self, name="RememberStartPose"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_trial_keys(self.blackboard)

    def setup(self, **kwargs):
        """Subscribe to the AMCL pose."""
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        return True

    def update(self):
        """Store the first AMCL pose."""
        if self.blackboard.oc_start_pose is not None:
            return py_trees.common.Status.SUCCESS
        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING
        start = Pose()
        start.position = self.pose.pose.position
        start.orientation = self.pose.pose.orientation
        self.blackboard.oc_start_pose = start
        self.node.get_logger().info(
            f"{self.name}: trials start from x={start.position.x:.2f} "
            f"y={start.position.y:.2f} yaw={math.degrees(self.pose.yaw):+.0f} deg"
        )
        return py_trees.common.Status.SUCCESS


class WaitForGo(py_trees.behaviour.Behaviour):
    """
    Wait for the experimenter's go, or for `oc_rest` when `oc_wait_for_go` is off.

    A go sent before this started waiting does not count: it was for the trial
    before, or sent while the robot was still driving back.
    """

    def __init__(self, name="WaitForGo"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        """Subscribe to the go topic."""
        self.node = kwargs["node"]
        self.go = GoSignal(self.node)
        return True

    def initialise(self):
        """Remember how many gos had arrived, and start the clock."""
        self._count = self.go.count
        self._start = _seconds(self.node)
        self._told = False

    def update(self):
        """Succeed on a new go, or after the rest."""
        if not bool(constant(self.blackboard, "oc_wait_for_go")):
            rest = float(constant(self.blackboard, "oc_rest"))
            if _seconds(self.node) - self._start >= rest:
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"resting {rest:.0f} s"
            return py_trees.common.Status.RUNNING
        if self.go.count > self._count:
            self.node.get_logger().info(f"{self.name}: go")
            return py_trees.common.Status.SUCCESS
        if not self._told:
            self._told = True
            self.node.get_logger().info(
                f"{self.name}: set the bricks out, then "
                "`ros2 topic pub --once /mecanumbot/object_choice/next std_msgs/Empty`"
            )
        self.feedback_message = "waiting for go"
        return py_trees.common.Status.RUNNING


class StartTrial(py_trees.behaviour.Behaviour):
    """Number a new trial and start its record; always SUCCESS."""

    def __init__(self, name="StartTrial"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_trial_keys(self.blackboard)

    def setup(self, **kwargs):
        """Keep the node handle."""
        self.node = kwargs["node"]
        return True

    def update(self):
        """Clear the last trial's bricks and choice, and open a record."""
        self.blackboard.oc_trial_index = int(self.blackboard.oc_trial_index) + 1
        self.blackboard.oc_bricks = {}
        self.blackboard.oc_choice = None
        TargetRelay.set_target(None)
        self.blackboard.oc_trial = {
            "trial": self.blackboard.oc_trial_index,
            "started": round(_seconds(self.node), 3),
            "response_mode": str(constant(self.blackboard, "oc_response_mode")),
        }
        self.node.get_logger().info(
            f"{self.name}: trial {self.blackboard.oc_trial_index} begins"
        )
        return py_trees.common.Status.SUCCESS


class SurveyBricks(py_trees.behaviour.Behaviour):
    """
    Look down at the floor and remember where each brick is; SUCCESS when enough are.

    The camera cannot see the floor and a standing person's arms at one tilt,
    so the bricks are surveyed first, with the head at `oc_survey_head`, and
    remembered in the map frame. Each colour's position is the mean of the
    sightings in the last `oc_survey_window` seconds, once it has
    `oc_survey_min_samples` of them. Enough means every colour in
    `oc_brick_colours`, or `oc_min_bricks` colours when the list is empty.

    FAILURE after `oc_survey_timeout`, recorded with what was seen.
    """

    def __init__(self, name="SurveyBricks"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_trial_keys(self.blackboard)
        for key in ("fetch_detection_threshold", "fetch_head_hold_resend"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key="fetch_head_position", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Subscribe to the bricks and build the neck commander."""
        self.node = kwargs["node"]
        self.bricks = BrickTracker(self.node)
        self.neck = GripperCommander(self.node)
        return True

    def initialise(self):
        """Put the head down and start collecting."""
        self.head = float(constant(self.blackboard, "oc_survey_head"))
        self.settle = float(constant(self.blackboard, "oc_survey_settle"))
        self.window = float(constant(self.blackboard, "oc_survey_window"))
        self.min_samples = int(constant(self.blackboard, "oc_survey_min_samples"))
        self.timeout = float(constant(self.blackboard, "oc_survey_timeout"))
        self.required = [str(c) for c in (constant(self.blackboard, "oc_brick_colours") or [])]
        self.min_bricks = int(constant(self.blackboard, "oc_min_bricks"))
        self.threshold = float(fetch_constant(self.blackboard, "fetch_detection_threshold"))
        self.resend = float(fetch_constant(self.blackboard, "fetch_head_hold_resend"))
        self._samples = {}
        self._last_stamp = {}
        self._start = _seconds(self.node)
        self._look()

    def update(self):
        """Collect sightings and decide whether the layout is surveyed."""
        now = _seconds(self.node)
        if now - self._sent_at >= self.resend:
            self._look()
        if now - self._start < self.settle:
            self.feedback_message = "letting the head settle"
            return py_trees.common.Status.RUNNING

        for colour, sighting in self.bricks.fresh(0.5, self.threshold).items():
            # In nanoseconds: rclpy's Time refuses to be compared with None.
            stamp = sighting.stamp.nanoseconds
            if self._last_stamp.get(colour) == stamp:
                continue
            self._last_stamp[colour] = stamp
            self._samples.setdefault(colour, []).append((now, sighting.position))
        for colour in list(self._samples):
            self._samples[colour] = [
                (t, p) for t, p in self._samples[colour] if now - t <= self.window
            ]

        surveyed = {
            colour: samples
            for colour, samples in self._samples.items()
            if len(samples) >= self.min_samples
        }
        if self._enough(surveyed):
            bricks = {colour: self._mean(samples) for colour, samples in surveyed.items()}
            if self.required:
                bricks = {c: p for c, p in bricks.items() if c in self.required}
            self.blackboard.oc_bricks = bricks
            self.blackboard.oc_trial["bricks"] = {c: list(p) for c, p in bricks.items()}
            self.node.get_logger().info(
                f"{self.name}: "
                + ", ".join(
                    f"{c} at ({p[0]:.2f}, {p[1]:.2f})" for c, p in sorted(bricks.items())
                )
            )
            return py_trees.common.Status.SUCCESS

        if now - self._start > self.timeout:
            seen = sorted(surveyed)
            self.blackboard.oc_trial["bricks_seen"] = seen
            self.node.get_logger().warn(
                f"{self.name}: after {self.timeout:.0f} s saw {seen or 'no bricks'}; "
                + (
                    f"the trial needs {self.required}"
                    if self.required
                    else f"the trial needs {self.min_bricks} colours"
                )
            )
            return py_trees.common.Status.FAILURE
        self.feedback_message = f"seen so far: {sorted(surveyed)}"
        return py_trees.common.Status.RUNNING

    def _enough(self, surveyed):
        if self.required:
            return all(colour in surveyed for colour in self.required)
        return len(surveyed) >= self.min_bricks

    @staticmethod
    def _mean(samples):
        count = float(len(samples))
        return tuple(
            round(sum(getattr(p, axis) for _, p in samples) / count, 3)
            for axis in ("x", "y", "z")
        )

    def _look(self):
        self.neck.look(self.head)
        self.blackboard.fetch_head_position = self.head
        self._sent_at = _seconds(self.node)


class ChooseBrick(py_trees.behaviour.Behaviour):
    """
    Turn the settled pointing cue into a chosen brick; FAILURE if it picks none.

    On a choice the relay starts forwarding that colour, and the fetch keys the
    approach reads are filled in from the survey -- so the approach starts from
    where the brick was seen even before it is seen again. Abstaining is
    FAILURE so the trial ends there, but the record says why.
    """

    def __init__(self, name="ChooseBrick"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_trial_keys(self.blackboard)
        self.blackboard.register_key("ostensive_cue", access=py_trees.common.Access.READ)
        for key in ("fetch_ball_position", "fetch_ball_score", "fetch_ball_height"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Keep the node handle."""
        self.node = kwargs["node"]
        return True

    def update(self):
        """Choose, record, and hand the chosen brick to the approach."""
        cue = self.blackboard.ostensive_cue
        bricks = self.blackboard.oc_bricks or {}
        if cue is None:
            self.node.get_logger().info(f"{self.name}: there is no cue to choose by")
            return py_trees.common.Status.FAILURE

        origin = (float(cue.origin[0]), float(cue.origin[1]))
        self.blackboard.oc_trial["cue"] = {
            "side": cue.side,
            "azimuth_deg": round(math.degrees(cue.azimuth), 1),
            "bearing_deg": round(math.degrees(cue.bearing), 1),
            "person": [round(origin[0], 3), round(origin[1], 3)],
        }
        choice = choice_geometry.choose(
            origin,
            cue.bearing,
            bricks,
            max_deviation=float(constant(self.blackboard, "oc_max_deviation")),
            min_margin=float(constant(self.blackboard, "oc_min_margin")),
            min_distance=float(constant(self.blackboard, "oc_min_brick_distance")),
        )
        self.blackboard.oc_trial["choice"] = choice.as_record()
        ranking = ", ".join(
            f"{c.colour} {math.degrees(c.deviation):.0f} deg"
            + (f" ({c.excluded})" if c.excluded else "")
            for c in choice.candidates
        )
        if not choice.chosen:
            self.node.get_logger().info(
                f"{self.name}: no brick chosen ({choice.reason}); {ranking}"
            )
            return py_trees.common.Status.FAILURE

        x, y, z = (list(bricks[choice.colour]) + [0.0])[:3]
        position = Point(x=float(x), y=float(y), z=float(z))
        self.blackboard.oc_choice = {"colour": choice.colour, "position": position}
        self.blackboard.fetch_ball_position = position
        self.blackboard.fetch_ball_height = float(z)
        self.blackboard.fetch_ball_score = 1.0
        TargetRelay.set_target(choice.colour)
        self.node.get_logger().info(f"{self.name}: chose {choice.colour}; {ranking}")
        return py_trees.common.Status.SUCCESS


class LookAtBrick(py_trees.behaviour.Behaviour):
    """
    Drop the head from the person back to the survey tilt; always SUCCESS.

    The fetch head tracker starts from `fetch_head_position` and only steps
    from there, so it has to start at a tilt that sees the floor.
    """

    def __init__(self, name="LookAtBrick"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_head_position", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Build the neck commander."""
        self.node = kwargs["node"]
        self.neck = GripperCommander(self.node)
        return True

    def update(self):
        """Command the survey tilt."""
        head = float(constant(self.blackboard, "oc_survey_head"))
        self.neck.look(head)
        self.blackboard.fetch_head_position = head
        return py_trees.common.Status.SUCCESS


class NoteTrial(py_trees.behaviour.Behaviour):
    """Write one field into the trial record; always SUCCESS."""

    def __init__(self, name, field, value=True):
        super().__init__(name)
        self.field = field
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=name)
        register_trial_keys(self.blackboard)

    def update(self):
        """Note the field."""
        self.blackboard.oc_trial[self.field] = self.value
        return py_trees.common.Status.SUCCESS


class ReturnToStart(py_trees.behaviour.Behaviour):
    """
    Drive back to the pose the run began at, through nav2.

    FAILURE on `oc_return_timeout`, or when nav2 drops the goal more than
    `nav_goal_retries` times.
    """

    def __init__(self, name="ReturnToStart"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        register_movement_keys(self.blackboard)
        register_trial_keys(self.blackboard, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Build the navigator."""
        self.node = kwargs["node"]
        self.nav2 = Nav2PoseNavigator(self.node)
        return True

    def initialise(self):
        """Forget any earlier goal and start the clock."""
        self.nav2.reset()
        self._sent = False
        self._resends = 0
        self._start = _seconds(self.node)

    def terminate(self, new_status):
        """Cancel the drive if anything interrupts it."""
        if new_status == py_trees.common.Status.INVALID:
            self.nav2.cancel()

    def update(self):
        """Send the start pose and follow nav2 there."""
        start = self.blackboard.oc_start_pose
        if start is None:
            self.node.get_logger().warn(f"{self.name}: no start pose recorded")
            return py_trees.common.Status.FAILURE
        timeout = float(constant(self.blackboard, "oc_return_timeout"))
        if _seconds(self.node) - self._start > timeout:
            self.node.get_logger().warn(f"{self.name}: not back after {timeout:.0f} s")
            self.nav2.cancel()
            return py_trees.common.Status.FAILURE
        if not self._sent:
            if not self.nav2.server_ready():
                self.feedback_message = "waiting for the nav2 action server"
                return py_trees.common.Status.RUNNING
            self.nav2.go_to(start)
            self._sent = True
            return py_trees.common.Status.RUNNING

        status = self.nav2.status()
        if status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name}: back at the start")
            return py_trees.common.Status.SUCCESS
        if status is None or status in GOAL_ACTIVE_STATUSES:
            self.feedback_message = "driving back to the start"
            return py_trees.common.Status.RUNNING
        retries = int(self.blackboard.nav_goal_retries)
        if self._resends >= retries:
            self.node.get_logger().warn(
                f"{self.name}: nav2 dropped the goal {self._resends} time(s)"
            )
            return py_trees.common.Status.FAILURE
        self._resends += 1
        self.nav2.go_to(start)
        return py_trees.common.Status.RUNNING


class FinishTrial(py_trees.behaviour.Behaviour):
    """
    Publish the trial's record and forget the trial; always SUCCESS.

    `outcome` is given on the path that completed. On the path that absorbs a
    failure it is None, and the outcome is read off how far the trial got:
    no bricks surveyed, nobody addressed the robot, no cue, no choice, a
    response that did not finish, or a return that did not.
    """

    def __init__(self, name="FinishTrial", outcome=None):
        super().__init__(name)
        self.outcome = outcome
        self.blackboard = self.attach_blackboard_client(name=name)
        register_trial_keys(self.blackboard)
        for key in ("ostensive_target", "ostensive_cue"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
        for key in ("fetch_grasped", "fetch_delivered"):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Build the record publisher and the pose tracker."""
        self.node = kwargs["node"]
        self.records = TrialRecordPublisher(self.node)
        self.state = StatePublisher(self.node)
        self.pose = RobotPoseTracker(self.node)
        return True

    def update(self):
        """Fill in the outcome, publish, and clear the trial state."""
        record = dict(self.blackboard.oc_trial or {})
        if not record:
            # Nothing was started -- the go never came, say -- so nothing to say.
            return py_trees.common.Status.SUCCESS
        outcome = self.outcome or self._outcome(record)
        record["outcome"] = outcome
        record["ended"] = round(_seconds(self.node), 3)
        record["grasped"] = bool(self.blackboard.fetch_grasped)
        record["delivered"] = bool(self.blackboard.fetch_delivered)
        choice = self.blackboard.oc_choice
        if choice is not None and self.pose.pose is not None:
            record["final_distance_to_choice"] = round(
                distance_xy(self.pose.pose.position, choice["position"]), 3
            )
            record["final_robot_position"] = _xyz(self.pose.pose.position)
        self.records.publish(record)
        self.state.publish(f"trial {record.get('trial')}: {outcome}")
        self.node.get_logger().info(
            f"{self.name}: trial {record.get('trial')} -- {outcome}"
            + (f", chose {choice['colour']}" if choice else "")
        )

        TargetRelay.set_target(None)
        self.blackboard.oc_trial = {}
        self.blackboard.oc_choice = None
        self.blackboard.oc_bricks = {}
        self.blackboard.ostensive_target = None
        self.blackboard.ostensive_cue = None
        return py_trees.common.Status.SUCCESS

    def _outcome(self, record):
        if "bricks" not in record:
            return OUTCOME_NO_BRICKS
        if self.blackboard.ostensive_target is None:
            return OUTCOME_NOT_ADDRESSED
        if "cue" not in record:
            return OUTCOME_NO_CUE
        if self.blackboard.oc_choice is None:
            return OUTCOME_NO_CHOICE
        if not record.get("responded"):
            return OUTCOME_RESPONSE_FAILED
        return OUTCOME_RETURN_FAILED
