"""
Tilting the head down onto a face at the bottom of the frame, and back up.

While the robot searches for people its head is lifted (`HEAD_SEEK`): that gives
the pose detector whole standing bodies instead of knees. It also puts somebody
sitting low -- on a bean bag, on the floor -- at the very bottom edge of the
frame with only their face in shot, and the detection gate wants a torso. In the
08:46 run of 2026-09-30 the robot drove and turned past a person on the bean bag
for a minute and a half with their face at the bottom of the frame.

The camera detector publishes a cue when it sees a face down there
(`cam_people_detections/low_head`). On a fresh cue the head drops by `step` and
stays down for `hold` seconds, which is long enough for the gate to confirm a
seated body; then it goes back to exactly where it was. It does not stay down
while the cue keeps coming -- a face the gate never accepts would otherwise pin
the head low for the rest of the search -- and after raising it ignores the cue
for `cooldown` seconds, so the scan carries on past a face it could not use.

No ROS here: `ros_interfaces.HeadLookDown` feeds this the clock, the cue time and
the neck position, and sends whatever it returns.
"""

# The neck's lowest commandable position (see NECK_* in ros_interfaces).
NECK_MIN_POS = 2.0


class LookDown:
    """The look-down state: up, or down since some time with a pose to return to."""

    def __init__(self, step=0.5, hold=3.0, cooldown=4.0, cue_age=0.5, min_pos=NECK_MIN_POS):
        self.step = float(step)
        self.hold = float(hold)
        self.cooldown = float(cooldown)
        self.cue_age = float(cue_age)
        self.min_pos = float(min_pos)
        self.restore_pos = None
        self._since = None
        self._quiet_until = None

    @property
    def lowered(self):
        """Say whether the head is down on a cue right now."""
        return self.restore_pos is not None

    def update(self, now, cue_time, neck_pos):
        """
        Return the neck position to command now, or None to leave it alone.

        `cue_time` is when the last cue arrived (None if never) and `neck_pos`
        the last commanded neck position (None if the head was never moved, in
        which case there is nothing to go back to and the cue is ignored).
        """
        if self.lowered:
            if now - self._since >= self.hold:
                return self._raise(now)
            return None
        if cue_time is None or now - cue_time > self.cue_age:
            return None
        if self._quiet_until is not None and now < self._quiet_until:
            return None
        if neck_pos is None:
            return None
        self.restore_pos = float(neck_pos)
        self._since = now
        return max(self.min_pos, self.restore_pos - self.step)

    def release(self, now):
        """Raise the head if it is down, whatever the hold says; return the pose or None."""
        if not self.lowered:
            return None
        return self._raise(now)

    def _raise(self, now):
        pos = self.restore_pos
        self.restore_pos = None
        self._since = None
        self._quiet_until = now + self.cooldown
        return pos
