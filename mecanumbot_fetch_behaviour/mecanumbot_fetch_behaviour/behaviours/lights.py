"""
Showing where the ball is with the robot's lights.

One behaviour, `ShowBallDirection`, which runs beside the approach from the
sighting to the grab. All four LED panels blink `fetch_led_color`, and the
panels on the ball's side blink `fetch_led_direction_color` instead; which
panels those are is decided in `ball_lights.py`.

The LED controller holds a setting until it is sent another, so the service is
only called when the panels that point at the ball change, and once more to
turn the lights off when the approach ends.
"""

import py_trees

from mecanumbot_movement_behaviours.geometry import bearing_to, normalize_angle
from mecanumbot_movement_behaviours.ros_interfaces import RobotPoseTracker
from mecanumbot_msgs.srv import SetLedStatus

from mecanumbot_fetch_behaviour.ball_lights import (
    CORNERS,
    direction_corners,
    panel_colors,
)
from mecanumbot_fetch_behaviour.defaults import constant, register_param_keys

LED_SERVICE = "/mecanumbot/set_led_status"

# A dark panel: solid black.
OFF_MODE = 4
OFF_COLOR = 0


def led_request(mode, colors):
    """Return a `SetLedStatus` request with one mode and a colour per panel."""
    request = SetLedStatus.Request()
    for corner in CORNERS:
        setattr(request, f"{corner}_mode", int(mode))
        setattr(request, f"{corner}_color", int(colors[corner]))
    return request


class ShowBallDirection(py_trees.behaviour.Behaviour):
    """
    Blink all the lights, the ones on the ball's side in another colour.

    Always RUNNING: a modifier on the approach beside it, like the head
    tracker, so it must not be able to end or fail the parallel it sits in. The
    lights go dark when it is stopped.

    The direction is the bearing of `fetch_ball_position` in the robot's frame,
    so it is where the robot has placed the ball in the map and keeps pointing
    there while the robot turns, whether or not the ball is in the picture.
    `fetch_led_enabled: false` makes this a behaviour that does nothing.
    """

    def __init__(self, name="ShowBallDirection"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        register_param_keys(self.blackboard)
        self.blackboard.register_key(
            key="fetch_ball_position", access=py_trees.common.Access.READ
        )
        self._shown = None

    def setup(self, **kwargs):
        """Read the pattern and build the LED client and the pose tracker."""
        self.node = kwargs["node"]
        self.enabled = bool(constant(self.blackboard, "fetch_led_enabled"))
        self.mode = int(constant(self.blackboard, "fetch_led_mode"))
        self.color = int(constant(self.blackboard, "fetch_led_color"))
        self.direction_color = int(
            constant(self.blackboard, "fetch_led_direction_color")
        )
        self.spread = float(constant(self.blackboard, "fetch_led_direction_spread"))
        self.led_client = self.node.create_client(SetLedStatus, LED_SERVICE)
        self.pose = RobotPoseTracker(self.node)
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def initialise(self):
        """Start with nothing shown."""
        self._shown = None
        self._direction = ()
        self._pending = None

    def update(self):
        """Show the ball's side, calling the service only when it changes."""
        if not self.enabled:
            self.feedback_message = "lights switched off in the constants file"
            return py_trees.common.Status.RUNNING

        ball = self.blackboard.fetch_ball_position
        if self.pose.pose is None or ball is None:
            self.feedback_message = "waiting for a pose and a ball"
            return py_trees.common.Status.RUNNING

        bearing = normalize_angle(bearing_to(self.pose.position, ball) - self.pose.yaw)
        self._direction = direction_corners(bearing, self.spread, self._direction)
        self.feedback_message = f"ball towards {'+'.join(self._direction) or 'nowhere'}"

        if self._pending is not None:
            if not self._pending.done():
                return py_trees.common.Status.RUNNING
            self._pending = None
        if self._direction != self._shown and self.led_client.service_is_ready():
            self._pending = self.led_client.call_async(
                led_request(
                    self.mode,
                    panel_colors(self._direction, self.color, self.direction_color),
                )
            )
            self._shown = self._direction
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Turn the lights off if this behaviour turned them on."""
        if self._shown is None:
            return
        self._shown = None
        if self.led_client.service_is_ready():
            self.led_client.call_async(
                led_request(OFF_MODE, dict.fromkeys(CORNERS, OFF_COLOR))
            )
