"""In-place turning behaviours with a velocity profile and a chosen direction.

All rotations run on `/cmd_vel` through `SmoothTurner` rather than as nav2
rotate-in-place goals. Two reasons:

* Smoothness -- the turn ramps up with a bounded acceleration and eases out
  proportionally to the angle left, instead of the stop-start of a fresh nav2
  goal per turn.
* Direction -- a nav2 goal pose only says *where* to face, never *which way
  round*. Owning the rotation lets the robot unwind: having looked back for its
  human clockwise, it returns to its route counterclockwise.

Translations still go through nav2 (see `Approach`), so obstacle avoidance is
untouched for anything that actually drives somewhere.
"""

import math

import py_trees

from mecanumbot_leading_behaviour.behaviours.geometry import (
    COUNTERCLOCKWISE,
    bearing_to,
    normalize_angle,
    signed_rotation,
)
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    Nav2GoalMonitor,
    PeopleTracker,
    RobotPoseTracker,
    VelocityCommander,
)
from mecanumbot_leading_behaviour.behaviours.targets import (
    SUBJECT,
    default_head_pose,
    is_human,
    register_target_keys,
    resolve_target_position,
)

# Blackboard key holding the handedness (+1 counterclockwise / -1 clockwise) of
# the rotation the robot last used while looking for a human.
SPIN_SIGN_KEY = "search_spin_sign"

# `direction` options of TurnToward.
SHORTEST = "shortest"
UNWIND = "unwind"
REPEAT = "repeat"


class SmoothTurner:
    """Velocity-profiled in-place rotation by a signed angle.

    Progress is measured on AMCL yaw and dead-reckoned from the commanded speed
    between pose updates, so a slow localisation update rate does not stall the
    profile.
    """

    def __init__(
        self,
        node,
        pose_tracker,
        velocity,
        max_speed=0.6,
        accel=0.8,
        decel_gain=1.6,
        min_speed=0.12,
        tolerance=math.radians(3.0),
    ):
        self.node = node
        self.pose = pose_tracker
        self.velocity = velocity
        self.max_speed = float(max_speed)
        self.accel = float(accel)
        self.decel_gain = float(decel_gain)
        self.min_speed = float(min_speed)
        self.tolerance = float(tolerance)
        self._target = 0.0
        self._direction = COUNTERCLOCKWISE
        self._speed = 0.0
        self._measured = 0.0
        self._extrapolated = 0.0
        self._last_yaw = None
        self._last_updates = None
        self._last_step_time = None

    def begin(self, rotation):
        """Start turning by `rotation` [rad]; sign gives the direction."""
        self._target = float(rotation)
        self._direction = COUNTERCLOCKWISE if rotation >= 0.0 else -COUNTERCLOCKWISE
        self._speed = 0.0
        self._measured = 0.0
        self._extrapolated = 0.0
        self._last_yaw = self.pose.yaw
        self._last_updates = self.pose.updates
        self._last_step_time = self.node.get_clock().now()

    @property
    def travelled(self):
        return self._measured + self._extrapolated

    @property
    def remaining(self):
        return self._target - self.travelled

    def step(self):
        """Publish the next command; True once the requested turn is done."""
        dt = self._tick()
        remaining = self.remaining
        overshot = self._target != 0.0 and remaining * self._target < 0.0
        if abs(remaining) <= self.tolerance or overshot:
            self.velocity.stop()
            self._speed = 0.0
            return True

        speed = min(self.max_speed, self.decel_gain * abs(remaining))  # ease out
        speed = max(speed, self.min_speed)
        speed = min(speed, self._speed + self.accel * dt)  # ease in
        self._speed = speed
        self.velocity.turn(math.copysign(speed, remaining))
        return False

    def brake(self):
        """Ramp the rotation down to a stop; True once stopped."""
        dt = self._tick()
        speed = self._speed - self.accel * dt
        if speed <= 0.0:
            self.velocity.stop()
            self._speed = 0.0
            return True
        self._speed = speed
        self.velocity.turn(math.copysign(speed, self._direction))
        return False

    def stop(self):
        """Hard stop, for termination paths where there is no time to ramp."""
        self._speed = 0.0
        self.velocity.stop()

    def _tick(self):
        now = self.node.get_clock().now()
        if self._last_step_time is None:
            self._last_step_time = now
            return 0.0
        dt = min((now - self._last_step_time).nanoseconds / 1e9, 0.5)
        self._last_step_time = now

        yaw, updates = self.pose.yaw, self.pose.updates
        if yaw is not None and updates != self._last_updates:
            if self._last_yaw is not None:
                self._measured += normalize_angle(yaw - self._last_yaw)
            self._last_yaw, self._last_updates = yaw, updates
            self._extrapolated = 0.0
        else:
            self._extrapolated += math.copysign(self._speed, self._direction) * dt
        return dt


class InPlaceTurn(py_trees.behaviour.Behaviour):
    """Shared scaffolding for the rotating behaviours.

    Subclasses decide *what* to turn by; this class owns the ROS handles, the
    velocity profile, the head pose, the nav2 hand-over and the exit ramp.
    """

    def __init__(self, name, head=None, timeout=20.0, nav2_wait=2.0, **turner_kwargs):
        super().__init__(name)
        self.head = head
        self.timeout = float(timeout)
        self.nav2_wait = float(nav2_wait)
        self._turner_kwargs = turner_kwargs

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=SPIN_SIGN_KEY, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.pose = RobotPoseTracker(self.node)
        self.velocity = VelocityCommander(self.node)
        self.accessories = AccessoryCommander(self.node)
        self.nav2 = Nav2GoalMonitor(self.node)
        self.turner = SmoothTurner(
            self.node, self.pose, self.velocity, **self._turner_kwargs
        )
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self._start_time = self.node.get_clock().now()
        self._exit_status = None
        self.accessories.look(self.head)

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.turner.stop()

    # --- helpers for subclasses ---------------------------------------------

    def elapsed(self):
        return (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9

    def nav2_handed_over(self):
        """True once nav2 stopped driving, so `/cmd_vel` is ours to command.

        A goal that never finishes must not block the tree forever, so after
        `nav2_wait` seconds the turn goes ahead anyway.
        """
        if not self.nav2.busy():
            return True
        if self.elapsed() > self.nav2_wait:
            self.node.get_logger().warn(
                f"{self.name}: nav2 still busy, taking /cmd_vel anyway"
            )
            return True
        self.feedback_message = "waiting for nav2 to finish its goal"
        return False

    def finish(self, status, reason=""):
        """Ramp down and return `status` once the robot stands still."""
        self._exit_status = status
        if reason:
            self.node.get_logger().info(f"{self.name}: {reason}")
        return self.ramp_down()

    def ramp_down(self):
        if self.turner.brake():
            return self._exit_status
        return py_trees.common.Status.RUNNING

    def record_spin_sign(self, rotation):
        """Remember which way round the robot turned to look at a human."""
        if rotation:
            self.blackboard.search_spin_sign = (
                COUNTERCLOCKWISE if rotation > 0.0 else -COUNTERCLOCKWISE
            )

    def last_spin_sign(self):
        """Handedness of the last search turn, 0 if the robot has not turned yet."""
        try:
            return self.blackboard.search_spin_sign or 0
        except (AttributeError, KeyError):
            return 0

    def preferred_sign(self, direction):
        """Turn handedness a `direction` mode asks for, or None for "shortest"."""
        sign = self.last_spin_sign()
        if direction == SHORTEST or not sign:
            return None
        return -sign if direction == UNWIND else sign


class TurnToward(InPlaceTurn):
    """Turn in place until the robot faces a target.

    `direction` picks which way round:

    * `"shortest"` -- the short way (default when facing a human);
    * `"unwind"` -- opposite to the rotation last used while looking for a
      human, i.e. back the way the robot came (default for route targets);
    * `"repeat"` -- keep that same handedness.

    After the profiled turn a human target is re-checked once, so a person who
    stepped aside meanwhile is still faced properly.
    """

    def __init__(
        self,
        name="TurnToward",
        target_type=SUBJECT,
        direction=None,
        head=None,
        corrections=1,
        target_timeout=3.0,
        sight_timeout=1.0,
        **kwargs,
    ):
        super().__init__(
            name,
            head=head if head is not None else default_head_pose(target_type),
            **kwargs,
        )
        self.target_type = target_type
        self.direction = (
            direction
            if direction is not None
            else (SHORTEST if is_human(target_type) else UNWIND)
        )
        self.corrections = int(corrections)
        self.target_timeout = float(target_timeout)
        self.sight_timeout = float(sight_timeout)
        register_target_keys(self.blackboard)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.people = (
            PeopleTracker(self.node, self.sight_timeout)
            if is_human(self.target_type)
            else None
        )

    def initialise(self):
        super().initialise()
        self._target_position = None
        self._turning = False
        self._corrections_left = self.corrections
        self.turner.stop()

    def update(self):
        if self._exit_status is not None:
            return self.ramp_down()

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self._target_position is None:
            self._target_position = self._look_up_target()
            if self._target_position is None:
                if self.elapsed() > self.target_timeout:
                    return self.finish(
                        py_trees.common.Status.FAILURE,
                        f"no {self.target_type} to turn to after {self.target_timeout:.0f} s",
                    )
                self.feedback_message = f"waiting for a {self.target_type} to turn to"
                return py_trees.common.Status.RUNNING

        if self.elapsed() > self.timeout:
            return self.finish(py_trees.common.Status.FAILURE, "turn timed out")

        if not self._turning:
            return self._start_turn()

        if not self.turner.step():
            return py_trees.common.Status.RUNNING
        return self._turn_finished()

    # --- internals ----------------------------------------------------------

    def _look_up_target(self):
        subject_pose = None
        if self.people is not None:
            subject_pose = self.people.last_seen_pose
            if subject_pose is None:
                return None
        return resolve_target_position(
            self.blackboard,
            self.target_type,
            subject_position=None if subject_pose is None else subject_pose.position,
        )

    def _start_turn(self):
        if not self.nav2_handed_over():
            return py_trees.common.Status.RUNNING

        rotation = signed_rotation(
            self.pose.yaw,
            bearing_to(self.pose.position, self._target_position),
            self.preferred_sign(self.direction),
        )
        if is_human(self.target_type):
            self.record_spin_sign(rotation)
        if rotation == 0.0:
            self.node.get_logger().info(
                f"{self.name}: already facing the {self.target_type}"
            )
            return py_trees.common.Status.SUCCESS

        self.turner.begin(rotation)
        self._turning = True
        self.node.get_logger().info(
            f"{self.name}: turning {math.degrees(rotation):+.0f} deg to face the "
            f"{self.target_type} ({self.direction})"
        )
        return py_trees.common.Status.RUNNING

    def _turn_finished(self):
        if self._corrections_left > 0 and is_human(self.target_type):
            self._corrections_left -= 1
            fresh = self._look_up_target()
            if fresh is not None:
                self._target_position = fresh
                error = signed_rotation(
                    self.pose.yaw, bearing_to(self.pose.position, fresh)
                )
                if error != 0.0:
                    self.turner.begin(error)
                    self.node.get_logger().info(
                        f"{self.name}: correcting by {math.degrees(error):+.0f} deg"
                    )
                    return py_trees.common.Status.RUNNING

        self.node.get_logger().info(f"{self.name}: facing the {self.target_type}")
        return py_trees.common.Status.SUCCESS


class RelativeTurnPattern(InPlaceTurn):
    """Attention-getting wiggle that ends on the heading it started from.

    The first step continues the handedness of the last search turn, so the
    wiggle flows out of that motion instead of reversing abruptly.
    """

    def __init__(
        self, name="RelativeTurnPattern", step_angle_deg=20.0, head=HEAD_SEEK, **kwargs
    ):
        super().__init__(name, head=head, **kwargs)
        step = math.radians(float(step_angle_deg))
        self._pattern = (step, -2.0 * step, 2.0 * step, -step)

    def initialise(self):
        super().initialise()
        sign = self.last_spin_sign() or COUNTERCLOCKWISE
        self._offsets = [offset * sign for offset in self._pattern]
        self._index = -1
        self.turner.stop()

    def update(self):
        if self._exit_status is not None:
            return self.ramp_down()

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self.elapsed() > self.timeout:
            return self.finish(py_trees.common.Status.FAILURE, "turn pattern timed out")

        if self._index < 0:
            if not self.nav2_handed_over():
                return py_trees.common.Status.RUNNING
            return self._next_step()

        if not self.turner.step():
            return py_trees.common.Status.RUNNING
        return self._next_step()

    def _next_step(self):
        self._index += 1
        if self._index >= len(self._offsets):
            self.node.get_logger().info(f"{self.name}: turn pattern completed")
            return py_trees.common.Status.SUCCESS
        self.turner.begin(self._offsets[self._index])
        self.feedback_message = f"step {self._index + 1}/{len(self._offsets)}"
        return py_trees.common.Status.RUNNING


class ScanSpin(InPlaceTurn):
    """Spin in place looking for people, head lifted for the whole scan.

    The spin direction defaults to whichever way is shorter towards the last
    place a person was seen, and is stored on the blackboard so the turn back to
    the route can unwind it. `revolutions=None` spins until somebody is seen or
    the timeout expires.
    """

    def __init__(
        self,
        name="ScanSpin",
        revolutions=1.0,
        spin_speed=0.5,
        direction=None,
        stop_on_person=True,
        sight_timeout=1.0,
        timeout=10.0,
        head=HEAD_SEEK,
        **kwargs,
    ):
        kwargs.setdefault("max_speed", float(spin_speed))
        super().__init__(name, head=head, timeout=timeout, **kwargs)
        self.revolutions = None if revolutions is None else float(revolutions)
        self.direction = direction
        self.stop_on_person = bool(stop_on_person)
        self.sight_timeout = float(sight_timeout)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.people = PeopleTracker(self.node, self.sight_timeout)

    def initialise(self):
        super().initialise()
        self._spinning = False
        self.turner.stop()

    def update(self):
        if self._exit_status is not None:
            return self.ramp_down()

        if self.stop_on_person and self.people.has_fresh_detection():
            return self.finish(
                py_trees.common.Status.SUCCESS, "person detected, stopping the scan"
            )

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self.elapsed() > self.timeout:
            return self.finish(
                py_trees.common.Status.FAILURE, "scan found nobody in time"
            )

        if not self._spinning:
            return self._start_spin()

        if self.turner.step():
            return self.finish(py_trees.common.Status.SUCCESS, "scan completed")
        self.feedback_message = (
            f"scanned {math.degrees(abs(self.turner.travelled)):.0f} deg"
        )
        return py_trees.common.Status.RUNNING

    def _start_spin(self):
        if not self.nav2_handed_over():
            return py_trees.common.Status.RUNNING

        sign = self.direction or self._sign_towards_last_person()
        self.record_spin_sign(sign)
        # An open-ended scan gets a target it will never reach; it exits on a
        # detection or on the timeout, both of which ramp down cleanly.
        sweep = 1.0e4 if self.revolutions is None else 2.0 * math.pi * self.revolutions
        self.turner.begin(sign * sweep)
        self._spinning = True
        self.node.get_logger().info(
            f"{self.name}: scanning {'counterclockwise' if sign > 0 else 'clockwise'}"
        )
        return py_trees.common.Status.RUNNING

    def _sign_towards_last_person(self):
        """Turn towards where somebody was last seen -- usually the shortest look back."""
        last_pose = self.people.last_seen_pose
        if last_pose is None:
            return COUNTERCLOCKWISE
        rotation = signed_rotation(
            self.pose.yaw, bearing_to(self.pose.position, last_pose.position)
        )
        if rotation == 0.0:
            return COUNTERCLOCKWISE
        return COUNTERCLOCKWISE if rotation > 0.0 else -COUNTERCLOCKWISE


class FindPeople(ScanSpin):
    """Spin until a fresh people detection arrives, FAILURE if nobody shows up."""

    def __init__(
        self,
        name="FindPeople",
        spin_speed=0.5,
        sight_timeout=1.0,
        timeout=10.0,
        **kwargs,
    ):
        super().__init__(
            name,
            revolutions=None,
            spin_speed=spin_speed,
            stop_on_person=True,
            sight_timeout=sight_timeout,
            timeout=timeout,
            **kwargs,
        )


class Spin360(ScanSpin):
    """One full scanning revolution.

    Detections are not acted on here: this runs inside the recovery parallel
    where `WaitForPerson` is the branch that reacts to seeing somebody.
    """

    def __init__(self, name="Spin360", spin_speed=0.3, timeout=45.0, **kwargs):
        super().__init__(
            name,
            revolutions=1.0,
            spin_speed=spin_speed,
            stop_on_person=False,
            timeout=timeout,
            **kwargs,
        )
