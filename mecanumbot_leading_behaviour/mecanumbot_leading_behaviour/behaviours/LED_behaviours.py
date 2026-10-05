"""LED signalling behaviours."""

import py_trees

from mecanumbot_msgs.srv import SetLedStatus

from mecanumbot_movement_behaviours.geometry import (
    bearing_to,
    distance_xy,
    normalize_angle,
)
from mecanumbot_movement_behaviours.ros_interfaces import (
    HEAD_LEVEL,
    AccessoryCommander,
    RobotPoseTracker,
    SubjectPoseTracker,
    duration,
)
from mecanumbot_movement_behaviours.targets import (
    LAST_CHECKPOINT,
    TARGET,
    register_target_keys,
    resolve_target_position,
)

from mecanumbot_leading_behaviour.behaviours.defaults import (
    constant,
    register_param_keys,
)
from mecanumbot_leading_behaviour.behaviours.keys import LEADING_KEYS
from mecanumbot_leading_behaviour.behaviours.led_signals import (
    CORNERS,
    DIRECTION_MODES,
    DIRECTION_NONE,
    LEFT,
    RIGHT,
    STRAIGHT,
    direction_corners,
    facing_corners,
    lead_side,
    progress_fill,
)

LED_SERVICE = "/mecanumbot/set_led_status"

# The three settings the lead signal switches between, by which way the light flows.
LEAD_KEYS = {
    STRAIGHT: "LED_lead_straight",
    LEFT: "LED_lead_left",
    RIGHT: "LED_lead_right",
}

# A dark panel: solid black.
OFF_MODE = 4
OFF_COLOR = 0


def copy_setting(setting):
    """Return a new `SetLedStatus` request with the same panels as `setting`."""
    request = SetLedStatus.Request()
    for corner in CORNERS:
        for field in ("mode", "color"):
            name = f"{corner}_{field}"
            setattr(request, name, getattr(setting, name))
    request.progress = setting.progress
    request.progress_color = setting.progress_color
    return request


def relative_bearing(pose_tracker, position):
    """Bearing [rad] of a map position in the robot's frame, left positive."""
    return normalize_angle(
        bearing_to(pose_tracker.position, position) - pose_tracker.yaw
    )

# Signalling mode -> the pair of blackboard keys holding its patterns and timings.
LED_KEYS = {
    "indicate_target": ("LED_indicate_target_seq", "LED_indicate_target_times"),
    "indicate_close_target": (
        "LED_indicate_close_target_seq",
        "LED_indicate_close_target_times",
    ),
    "catch_attention": ("LED_catch_attention_seq", "LED_catch_attention_times"),
    "thank": ("LED_thank_seq", "LED_thank_times"),
}


class LEDBehaviourSequence(py_trees.behaviour.Behaviour):
    """
    Play one timed LED pattern sequence from the blackboard.

    Each pattern is a `SetLedStatus` service call; the next one is only sent
    after the previous call returned and its hold time elapsed.

    An `addressed` sequence is meant for the person rather than the room: only
    the two panels on the half of the robot that faces them are lit, and the
    other two stay dark. Where the person is unknown, or `LED_address_person`
    is off in the constants file, all four play as written.
    """

    def __init__(self, name="LEDBehaviourSeq", mode="catch_attention", addressed=False):
        super().__init__(name)
        self.addressed = addressed
        if mode not in LED_KEYS:
            raise ValueError(
                f"unknown LED behaviour mode '{mode}', expected one of {sorted(LED_KEYS)}"
            )
        self.mode = mode
        self.seq_key, self.times_key = LED_KEYS[mode]

        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard, "LED_address_person")
        for seq_key, times_key in LED_KEYS.values():
            self.blackboard.register_key(
                key=seq_key, access=py_trees.common.Access.READ
            )
            self.blackboard.register_key(
                key=times_key, access=py_trees.common.Access.READ
            )

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        self.pose = RobotPoseTracker(self.node) if self.addressed else None
        self.subject = SubjectPoseTracker(self.node) if self.addressed else None
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.lit_corners = self._corners_facing_person()
        self.index = 0
        self.next_send_time = None
        self.pending_call = None
        self.patterns = getattr(self.blackboard, self.seq_key)
        self.delays = getattr(self.blackboard, self.times_key)

    def update(self):
        if not self.patterns or not self.delays:
            self.feedback_message = f"no patterns configured for '{self.mode}'"
            return py_trees.common.Status.RUNNING

        now = self.node.get_clock().now()

        if self.pending_call is not None:
            if not self.pending_call.done():
                self.feedback_message = "waiting for the LED service"
                return py_trees.common.Status.RUNNING
            self.pending_call = None
            self.next_send_time = now + duration(self.delays[self.index - 1])
            return py_trees.common.Status.RUNNING

        if self.next_send_time is not None and now < self.next_send_time:
            self.feedback_message = f"step {self.index}/{len(self.patterns)}"
            return py_trees.common.Status.RUNNING

        if self.index >= len(self.patterns):
            self.node.get_logger().info(
                f"{self.name}: LED pattern '{self.mode}' completed"
            )
            return py_trees.common.Status.SUCCESS

        self.pending_call = self.led_client.call_async(
            self._addressed(self.patterns[self.index])
        )
        self.index += 1
        self.next_send_time = None
        return py_trees.common.Status.RUNNING

    def _corners_facing_person(self):
        """Return the panels to light, or None for all of them."""
        if not self.addressed or not constant(self.blackboard, "LED_address_person"):
            return None
        if self.pose.pose is None or self.subject.position is None:
            return None
        return facing_corners(relative_bearing(self.pose, self.subject.position))

    def _addressed(self, pattern):
        if self.lit_corners is None:
            return pattern
        request = copy_setting(pattern)
        for corner in CORNERS:
            if corner not in self.lit_corners:
                setattr(request, f"{corner}_mode", OFF_MODE)
                setattr(request, f"{corner}_color", OFF_COLOR)
        return request


class LevelHead(py_trees.behaviour.Behaviour):
    """
    Park the head at the level pose, and return SUCCESS.

    The LED condition's head only moves while the robot is looking for the
    person, where the body gaze tilts it to the height the LiDAR expects them
    at. This ends that: it takes the head out of the seeking pose, so the gaze
    leaves it alone, and puts it back where the condition has always held it.
    Placed before the drive to the target, so the robot leads and signals with
    a still head.
    """

    def __init__(self, name="LevelHead"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.accessories = AccessoryCommander(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def update(self):
        self.accessories.look(HEAD_LEVEL)
        return py_trees.common.Status.SUCCESS


class LEDLeadSignal(py_trees.behaviour.Behaviour):
    """
    Keep the LEDs signalling for as long as the robot drives to a place.

    Two things are shown at once, on all four panels. The light *flows*: ahead
    while the destination is ahead, and towards its side once it is more than
    `LED_lead_straight_band` off the robot's heading (`LED_lead_direction:
    none` keeps it flowing ahead throughout). And the panels *fill* with
    `LED_lead_progress_color`, one LED per eighth of the way driven, so how far
    there is to go can be read at any moment rather than only when it changes.

    The behaviour never finishes by itself -- it is the companion of a drive in
    a parallel, and ends when the drive does. The LED controller holds a
    setting until it is sent another, so the service is only called when what
    is shown has to change. The fill never goes back: a detour that takes the
    robot further from the destination leaves the bar where it was.
    """

    def __init__(self, name="LEDLeadSignal", target_type=LAST_CHECKPOINT):
        super().__init__(name)
        self.target_type = target_type
        self.blackboard = self.attach_blackboard_client(name=name)
        register_target_keys(self.blackboard, LEADING_KEYS)
        register_param_keys(self.blackboard)
        for key in LEAD_KEYS.values():
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        self.pose = RobotPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.direction = constant(self.blackboard, "LED_lead_direction")
        if self.direction not in DIRECTION_MODES:
            raise ValueError(
                f"unknown LED_lead_direction '{self.direction}', "
                f"expected one of {DIRECTION_MODES}"
            )
        self.band = float(constant(self.blackboard, "LED_lead_straight_band"))
        self.progress_color = int(constant(self.blackboard, "LED_lead_progress_color"))
        self.stop_distance = float(constant(self.blackboard, "route_stop_distance"))
        self.start_distance = None
        self.side = STRAIGHT
        self.fill = 0
        self.shown = None
        self.pending_call = None

    def update(self):
        destination = resolve_target_position(
            self.blackboard, self.target_type, keys=LEADING_KEYS
        )
        if self.pose.pose is None or destination is None:
            self.feedback_message = "waiting for a pose and a destination"
            return py_trees.common.Status.RUNNING

        distance = distance_xy(self.pose.position, destination)
        if self.start_distance is None:
            self.start_distance = distance
        self.fill = max(
            self.fill,
            progress_fill(self.start_distance, distance, self.stop_distance),
        )
        if self.direction != DIRECTION_NONE:
            self.side = lead_side(
                relative_bearing(self.pose, destination), self.band, self.side
            )

        self.feedback_message = f"{self.side}, {self.fill} LEDs of progress"
        if self.pending_call is not None:
            if not self.pending_call.done():
                return py_trees.common.Status.RUNNING
            self.pending_call = None
        if (self.side, self.fill) != self.shown:
            self.pending_call = self.led_client.call_async(self._setting())
            self.shown = (self.side, self.fill)
        return py_trees.common.Status.RUNNING

    def _setting(self):
        setting = getattr(self.blackboard, LEAD_KEYS[self.side])
        if isinstance(setting, list):
            setting = setting[0]
        request = copy_setting(setting)
        request.progress = self.fill
        request.progress_color = self.progress_color
        return request


class LEDTargetDirection(py_trees.behaviour.Behaviour):
    """
    Blink the panels that point at a place, for `LED_direction_hold` seconds.

    For telling somebody where the target is while the robot faces *them*: the
    panels on the target's side blink `LED_direction_color`, and the others
    show `LED_direction_base_color` (black, so only the pointing ones are lit).
    Which panels point is `led_signals.direction_corners`.

    SUCCESS when the time is up, with the lights dark again. The service is
    called when the pointing panels change, which for a robot standing still is
    once.
    """

    def __init__(self, name="LEDTargetDirection", target_type=TARGET):
        super().__init__(name)
        self.target_type = target_type
        self.blackboard = self.attach_blackboard_client(name=name)
        register_target_keys(self.blackboard, LEADING_KEYS)
        register_param_keys(self.blackboard)
        self.shown = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        self.pose = RobotPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")

    def initialise(self):
        self.mode = int(constant(self.blackboard, "LED_direction_mode"))
        self.color = int(constant(self.blackboard, "LED_direction_color"))
        self.base_color = int(constant(self.blackboard, "LED_direction_base_color"))
        self.spread = float(constant(self.blackboard, "LED_direction_spread"))
        self.hold = duration(constant(self.blackboard, "LED_direction_hold"))
        self.start_time = self.node.get_clock().now()
        self.pointing = ()
        self.shown = None
        self.pending_call = None

    def update(self):
        if self.node.get_clock().now() - self.start_time >= self.hold:
            self._go_dark()
            self.node.get_logger().info(f"{self.name}: direction signal completed")
            return py_trees.common.Status.SUCCESS

        destination = resolve_target_position(
            self.blackboard, self.target_type, keys=LEADING_KEYS
        )
        if self.pose.pose is None or destination is None:
            self.feedback_message = "waiting for a pose and a target"
            return py_trees.common.Status.RUNNING

        self.pointing = direction_corners(
            relative_bearing(self.pose, destination), self.spread, self.pointing
        )
        self.feedback_message = f"target towards {'+'.join(self.pointing)}"
        if self.pending_call is not None:
            if not self.pending_call.done():
                return py_trees.common.Status.RUNNING
            self.pending_call = None
        if self.pointing != self.shown:
            self.pending_call = self.led_client.call_async(self._setting())
            self.shown = self.pointing
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self._go_dark()

    def _setting(self):
        request = SetLedStatus.Request()
        for corner in CORNERS:
            pointing = corner in self.pointing
            setattr(request, f"{corner}_mode", self.mode)
            setattr(
                request, f"{corner}_color", self.color if pointing else self.base_color
            )
        return request

    def _go_dark(self):
        if self.shown is None:
            return
        self.shown = None
        request = SetLedStatus.Request()
        for corner in CORNERS:
            setattr(request, f"{corner}_mode", OFF_MODE)
            setattr(request, f"{corner}_color", OFF_COLOR)
        self.led_client.call_async(request)
