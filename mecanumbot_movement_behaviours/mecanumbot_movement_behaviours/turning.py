"""
In-place turning behaviours with a velocity profile and a chosen direction.

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

from mecanumbot_movement_behaviours.defaults import (
    constant,
    register_param_keys,
    resolve,
)
from mecanumbot_movement_behaviours.geometry import (
    COUNTERCLOCKWISE,
    bearing_to,
    normalize_angle,
    signed_rotation,
)
from mecanumbot_movement_behaviours.keys import DEFAULT_KEYS
from mecanumbot_movement_behaviours.pacing import sweep_pattern
from mecanumbot_movement_behaviours.ros_interfaces import (
    AccessoryCommander,
    HEAD_SEEK,
    Nav2GoalMonitor,
    PeopleTracker,
    RobotPoseTracker,
    VelocityCommander,
    now_seconds,
)
from mecanumbot_movement_behaviours.targets import (
    SUBJECT,
    default_head_pose,
    is_human,
    register_target_keys,
    resolve_target_position,
)

# `direction` options of TurnToward.
SHORTEST = "shortest"
UNWIND = "unwind"
REPEAT = "repeat"

# `SmoothTurner` keyword -> the blackboard constant that supplies it when the
# call site does not. `InPlaceTurn` fills these in before it builds the turner.
TURNER_PARAMS = {
    "max_speed": "turn_max_speed",
    "accel": "turn_accel",
    "decel_gain": "turn_decel_gain",
    "min_speed": "turn_min_speed",
    "tolerance": "turn_tolerance",
}


class SmoothTurner:
    """
    Velocity-profiled in-place rotation by a signed angle.

    Progress is measured on AMCL yaw and dead-reckoned from the commanded speed
    between pose updates, so a slow localisation update rate does not stall the
    profile.

    The signature defaults are only what a turner built outside a behaviour
    gets; every one the trees create is handed the configured values through
    `TURNER_PARAMS`.
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
    """
    Shared scaffolding for the rotating behaviours.

    Subclasses decide *what* to turn by; this class owns the ROS handles, the
    velocity profile, the head pose, the nav2 hand-over and the exit ramp.

    `timeout` and `nav2_wait` default to None, meaning "take the configured
    value". A caller may still pass a number, which wins -- the constants are
    the default for every turn in the tree, not a ceiling on what one turn is
    allowed to be.

    `keys` is the spelling of the blackboard keys this behaviour reads; a
    package that names them differently binds its own map on `KEYS` in a
    subclass rather than passing one at every call site.
    """

    KEYS = DEFAULT_KEYS

    def __init__(
        self, name, head=None, timeout=None, nav2_wait=None, keys=None, **turner_kwargs
    ):
        super().__init__(name)
        self.head = head
        self.timeout = timeout
        self.nav2_wait = nav2_wait
        self.keys = keys or self.KEYS
        self._turner_kwargs = turner_kwargs

        self.blackboard = self.attach_blackboard_client(name=name)
        self.keys.register(self.blackboard, py_trees.common.Access.WRITE, "spin_sign")
        register_param_keys(self.blackboard)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.timeout = float(resolve(self.timeout, self.blackboard, "turn_timeout"))
        self.nav2_wait = float(
            resolve(self.nav2_wait, self.blackboard, "turn_nav2_wait")
        )
        self.facing_epsilon = float(constant(self.blackboard, "facing_epsilon"))
        for keyword, key in TURNER_PARAMS.items():
            self._turner_kwargs.setdefault(keyword, constant(self.blackboard, key))

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
        """
        Wait for nav2 to stop driving, so `/cmd_vel` is ours to command.

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
            self.blackboard.set(
                self.keys.spin_sign,
                COUNTERCLOCKWISE if rotation > 0.0 else -COUNTERCLOCKWISE,
            )

    def last_spin_sign(self):
        """Handedness of the last search turn, 0 if the robot has not turned yet."""
        try:
            return self.blackboard.get(self.keys.spin_sign) or 0
        except (AttributeError, KeyError):
            return 0

    def preferred_sign(self, direction):
        """Turn handedness a `direction` mode asks for, or None for "shortest"."""
        sign = self.last_spin_sign()
        if direction == SHORTEST or not sign:
            return None
        return -sign if direction == UNWIND else sign


class TurnToward(InPlaceTurn):
    """
    Turn in place until the robot faces a target.

    `direction` picks which way round:

    * `"shortest"` -- the short way (default when facing a human);
    * `"unwind"` -- opposite to the rotation last used while looking for a
      human, i.e. back the way the robot came (default for route targets);
    * `"repeat"` -- keep that same handedness.

    After the profiled turn a human target is re-checked once, so a person who
    stepped aside meanwhile is still faced properly.

    `min_rotation_key` names a constant holding the smallest turn worth making
    for this call site. Below it the behaviour succeeds without turning at all,
    which is how the route turns stay out of the way: setting off down a leg
    that bends by a few degrees is nav2's job, and stopping to rotate for it
    only breaks the drive up. Left unset, every turn is made, however small --
    the comparison conditions turn exactly as they always did.
    """

    def __init__(
        self,
        name="TurnToward",
        target_type=SUBJECT,
        direction=None,
        head=None,
        corrections=None,
        target_timeout=None,
        sight_timeout=None,
        min_rotation=None,
        min_rotation_key=None,
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
        self.corrections = corrections
        self.target_timeout = target_timeout
        self.sight_timeout = sight_timeout
        self.min_rotation = min_rotation
        self.min_rotation_key = min_rotation_key
        register_target_keys(self.blackboard, self.keys)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        if self.min_rotation is None:
            self.min_rotation = (
                0.0
                if self.min_rotation_key is None
                else float(constant(self.blackboard, self.min_rotation_key))
            )
        self.corrections = int(
            resolve(self.corrections, self.blackboard, "turn_corrections")
        )
        self.target_timeout = float(
            resolve(self.target_timeout, self.blackboard, "turn_target_timeout")
        )
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
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
            keys=self.keys,
        )

    def _start_turn(self):
        if not self.nav2_handed_over():
            return py_trees.common.Status.RUNNING

        rotation = signed_rotation(
            self.pose.yaw,
            bearing_to(self.pose.position, self._target_position),
            self.preferred_sign(self.direction),
            epsilon=self.facing_epsilon,
        )
        if is_human(self.target_type):
            self.record_spin_sign(rotation)
        if rotation == 0.0 or abs(rotation) < self.min_rotation:
            self.node.get_logger().info(
                f"{self.name}: already facing the {self.target_type} closely enough "
                f"({math.degrees(rotation):+.0f} deg off)"
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
                    self.pose.yaw,
                    bearing_to(self.pose.position, fresh),
                    epsilon=self.facing_epsilon,
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
    """
    Attention-getting wiggle that ends on the heading it started from.

    The first step continues the handedness of the last search turn, so the
    wiggle flows out of that motion instead of reversing abruptly.
    """

    def __init__(
        self, name="RelativeTurnPattern", step_angle=None, head=HEAD_SEEK, **kwargs
    ):
        super().__init__(name, head=head, **kwargs)
        self.step_angle = step_angle

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # `attention_turn_step` is declared in degrees and reaches the
        # blackboard in radians, like every other angle.
        step = float(resolve(self.step_angle, self.blackboard, "attention_turn_step"))
        self._pattern = sweep_pattern(step)

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
    """
    Spin in place looking for people, head lifted for the whole scan.

    The spin direction defaults to whichever way is shorter towards the last
    place a person was seen, and is stored on the blackboard so the turn back to
    the route can unwind it. `revolutions=None` spins until somebody is seen or
    the timeout expires.

    Which constants configure the scan is a class attribute, so the subclasses
    below get their own entries in the YAML: a glance around and the recovery
    patrol's full revolution are different gestures and are tuned apart.
    """

    SPEED_KEY = "scan_spin_speed"
    TIMEOUT_KEY = "scan_timeout"
    # Set by a subclass whose sweep length is configured too; where it is None,
    # `revolutions=None` keeps its plain meaning of "spin until something stops
    # the scan".
    REVOLUTIONS_KEY = None

    def __init__(
        self,
        name="ScanSpin",
        revolutions=1.0,
        spin_speed=None,
        direction=None,
        stop_on_person=True,
        sight_timeout=None,
        timeout=None,
        head=HEAD_SEEK,
        **kwargs,
    ):
        super().__init__(name, head=head, timeout=timeout, **kwargs)
        self.revolutions = revolutions
        self.spin_speed = spin_speed
        self.direction = direction
        self.stop_on_person = bool(stop_on_person)
        self.sight_timeout = sight_timeout

    def setup(self, **kwargs):
        # The spin speed *is* the turner's top speed, and the timeout is the one
        # InPlaceTurn would otherwise resolve, so both have to be settled before
        # the profile is built.
        speed = resolve(self.spin_speed, self.blackboard, self.SPEED_KEY)
        self._turner_kwargs.setdefault("max_speed", float(speed))
        self.timeout = float(resolve(self.timeout, self.blackboard, self.TIMEOUT_KEY))

        super().setup(**kwargs)
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        if self.REVOLUTIONS_KEY is not None:
            self.revolutions = resolve(
                self.revolutions, self.blackboard, self.REVOLUTIONS_KEY
            )
        self.revolutions = None if self.revolutions is None else float(self.revolutions)
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
            self.pose.yaw,
            bearing_to(self.pose.position, last_pose.position),
            epsilon=self.facing_epsilon,
        )
        if rotation == 0.0:
            return COUNTERCLOCKWISE
        return COUNTERCLOCKWISE if rotation > 0.0 else -COUNTERCLOCKWISE


class FindPeople(ScanSpin):
    """
    Spin until a fresh people detection arrives, FAILURE if nobody shows up.

    Configured by `scan_spin_speed` / `scan_timeout` -- the glance-around
    numbers, which is what this is.
    """

    def __init__(self, name="FindPeople", spin_speed=None, **kwargs):
        super().__init__(
            name,
            revolutions=None,
            spin_speed=spin_speed,
            stop_on_person=True,
            **kwargs,
        )


class Spin360(ScanSpin):
    """
    A full scanning revolution, configured by the `full_scan_*` constants.

    Detections are not acted on here: this runs inside the recovery parallel
    where `WaitForPerson` is the branch that reacts to seeing somebody. It is
    slower and longer than a glance because it is a search of the room.
    """

    SPEED_KEY = "full_scan_spin_speed"
    TIMEOUT_KEY = "full_scan_timeout"
    REVOLUTIONS_KEY = "full_scan_revolutions"

    def __init__(self, name="Spin360", spin_speed=None, revolutions=None, **kwargs):
        super().__init__(
            name,
            revolutions=revolutions,
            spin_speed=spin_speed,
            stop_on_person=False,
            **kwargs,
        )


class GlanceBack(InPlaceTurn):
    """
    The look over the shoulder: turn to where the human was, and check.

    This is the dog's check-in, and it is a different movement from a search.
    It goes at `glance_spin_speed`, slower than either scan, for two reasons at
    once: a slow turn reads as looking rather than casting about, and it gives
    the camera detector time to find a person the robot sweeps past -- turning
    fast past somebody and declaring them missing is exactly how a following
    human gets treated as a lost one.

    Two phases:

    1. **look** -- turn onto the bearing of the place the human was last seen,
       the short way round. With nobody ever seen it faces back down the route
       instead, towards the checkpoint the pair came from, because that is where
       somebody following would be.
    2. **sweep** -- only if nobody is in sight once the robot is facing there:
       a slow `+1, -2, +2, -1` sweep of `glance_sweep` either side, ending back
       on the middle. Any detection at any point during it ends the glance.

    SUCCESS means the human was found and leading can carry on. FAILURE means
    they were not, which is the only thing that starts the recovery patrol: the
    robot does not go hunting through the building for somebody it has not
    properly looked for yet.

    Whatever the answer, the check-in counters are reset as the behaviour ends,
    so the pacing counts from this look back rather than from the last one.
    """

    def __init__(
        self,
        name="GlanceBack",
        spin_speed=None,
        sweep_angle=None,
        timeout=None,
        sight_timeout=None,
        head=HEAD_SEEK,
        **kwargs,
    ):
        super().__init__(name, head=head, timeout=timeout, **kwargs)
        self.spin_speed = spin_speed
        self.sweep_angle = sweep_angle
        self.sight_timeout = sight_timeout
        register_target_keys(self.blackboard, self.keys)
        self.keys.register(
            self.blackboard,
            py_trees.common.Access.WRITE,
            "checkpoints_since",
            "last_check_in",
        )

    def setup(self, **kwargs):
        # Both the top speed of the profile and the timeout have to be settled
        # before InPlaceTurn builds the turner out of them.
        self._turner_kwargs.setdefault(
            "max_speed",
            float(resolve(self.spin_speed, self.blackboard, "glance_spin_speed")),
        )
        self.timeout = float(resolve(self.timeout, self.blackboard, "glance_timeout"))

        super().setup(**kwargs)
        self.sweep_angle = float(
            resolve(self.sweep_angle, self.blackboard, "glance_sweep")
        )
        self.sight_timeout = float(
            resolve(self.sight_timeout, self.blackboard, "sight_timeout")
        )
        self.people = PeopleTracker(self.node, self.sight_timeout)

    def initialise(self):
        super().initialise()
        self._phase = "look"
        self._offsets = []
        self._index = -1
        self._turning = False
        self.turner.stop()

    def terminate(self, new_status):
        super().terminate(new_status)
        if new_status in (
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ):
            self.blackboard.set(self.keys.checkpoints_since, 0)
            self.blackboard.set(self.keys.last_check_in, now_seconds(self.node))

    def update(self):
        if self._exit_status is not None:
            return self.ramp_down()

        if self.pose.pose is None:
            self.feedback_message = "waiting for AMCL pose"
            return py_trees.common.Status.RUNNING

        if self.elapsed() > self.timeout:
            return self.finish(
                py_trees.common.Status.FAILURE, "ran out of time looking back"
            )

        if not self._turning:
            if not self.nav2_handed_over():
                return py_trees.common.Status.RUNNING
            return self._start_look()

        if not self.turner.step():
            if self._phase == "sweep" and self.people.has_fresh_detection():
                return self.finish(
                    py_trees.common.Status.SUCCESS, "the human turned up in the sweep"
                )
            self.feedback_message = f"{self._phase}ing for the human"
            return py_trees.common.Status.RUNNING

        if self._phase == "look":
            if self.people.has_fresh_detection():
                return self.finish(
                    py_trees.common.Status.SUCCESS, "the human is there, still following"
                )
            return self._start_sweep()
        return self._next_sweep_step()

    # --- internals ----------------------------------------------------------

    def _start_look(self):
        rotation = signed_rotation(
            self.pose.yaw,
            self._look_bearing(),
            epsilon=self.facing_epsilon,
        )
        self.record_spin_sign(rotation)
        self._turning = True
        if rotation == 0.0:
            # Already facing the place: there is nothing to look round at, so
            # this is a sweep from the start.
            return self._start_sweep()
        self.turner.begin(rotation)
        self.node.get_logger().info(
            f"{self.name}: looking back {math.degrees(rotation):+.0f} deg for the human"
        )
        return py_trees.common.Status.RUNNING

    def _look_bearing(self):
        """Where to look: the human's last known place, else back down the route."""
        last_seen = self.people.last_seen_pose
        if last_seen is not None:
            return bearing_to(self.pose.position, last_seen.position)

        behind = self._checkpoint_behind()
        if behind is not None:
            return bearing_to(self.pose.position, behind)
        return normalize_angle(self.pose.yaw + math.pi)

    def _checkpoint_behind(self):
        checkpoints = self.blackboard.get(self.keys.checkpoints)
        index = int(self.blackboard.get(self.keys.current_checkpoint)) - 1
        if checkpoints and 0 <= index < len(checkpoints):
            return checkpoints[index]
        return None

    def _start_sweep(self):
        # The sweep carries on the handedness of the look, so the two read as
        # one movement rather than as a turn followed by a correction.
        sign = self.last_spin_sign() or COUNTERCLOCKWISE
        self._offsets = [offset * sign for offset in sweep_pattern(self.sweep_angle)]
        self._phase = "sweep"
        self._index = -1
        self.node.get_logger().info(
            f"{self.name}: nobody there, sweeping "
            f"{math.degrees(self.sweep_angle):.0f} deg either side"
        )
        return self._next_sweep_step()

    def _next_sweep_step(self):
        if self.people.has_fresh_detection():
            return self.finish(
                py_trees.common.Status.SUCCESS, "the human turned up in the sweep"
            )
        self._index += 1
        if self._index >= len(self._offsets):
            return self.finish(
                py_trees.common.Status.FAILURE, "the human is nowhere in sight"
            )
        self.turner.begin(self._offsets[self._index])
        self.feedback_message = f"sweep step {self._index + 1}/{len(self._offsets)}"
        return py_trees.common.Status.RUNNING
