"""
Reading an attention bid and a direction cue out of one person's keypoints.

Two gestures are decoded here, and they are deliberately different in kind:

* the **attention signal** is what makes a bystander into *the* addressee, so it
  has to be something nobody produces by accident. A raised hand is the cheap
  version and a wave -- a raised hand that keeps changing direction -- is the
  one that cannot be triggered by standing still with an arm up, which is why
  both are available and `ANY` accepts either;
* the **direction cue** is read only from the person the robot has already
  locked onto, so it can afford to be a single-frame measurement, stabilised by
  a dwell rather than by movement.

Every threshold is a ratio against the person's own body scale (see
`keypoints.body_scale`), never an absolute number of pixels, so distance from
the camera does not retune the gestures.

What monocular keypoints cannot give us is the depth component of a pointing
arm: an arm pointing straight at the camera and one pointing straight away from
it project to the same short vector. Rather than mapping that to "straight
ahead" and driving somewhere nobody indicated, `pointing_cue` rejects it on
`min_lateral_ratio` -- a cue has to commit to a side to count.

Pure Python, so it is unit-tested without a ROS graph or a camera.
"""

import math

from mecanumbot_ostensive_behaviour.behaviours.keypoints import (
    SIDES,
    arm,
    body_scale,
    hip_line_y,
    torso_centre,
)

# `attention_signal_mode` values.
RAISED_HAND = "raised_hand"
WAVE = "wave"
ANY = "any"
SIGNAL_MODES = (RAISED_HAND, WAVE, ANY)


class AttentionSignal:
    """An attention bid seen on one person: which gesture, and which hand."""

    __slots__ = ("kind", "side")

    def __init__(self, kind, side):
        self.kind = kind
        self.side = side

    def __repr__(self):
        """Return a debugging representation naming the gesture and the hand."""
        return f"AttentionSignal({self.kind}, {self.side})"


class PointingCue:
    """A pointing arm, decoded into an azimuth relative to where the person faces."""

    __slots__ = ("side", "azimuth", "lateral_ratio", "extension")

    def __init__(self, side, azimuth, lateral_ratio, extension):
        self.side = side
        self.azimuth = float(azimuth)
        self.lateral_ratio = float(lateral_ratio)
        self.extension = float(extension)

    def __repr__(self):
        """Return a debugging representation naming the arm and the azimuth."""
        return (
            f"PointingCue({self.side}, azimuth={math.degrees(self.azimuth):+.0f} deg, "
            f"extension={self.extension:.2f})"
        )


def raised_hand(joints, margin_ratio=0.15):
    """
    Return the side of a hand held above its shoulder, or None.

    `margin_ratio` is how far above the shoulder the wrist has to be, in body
    scales, so a wrist that merely draws level with a shoulder does not count.
    Both hands up returns the higher one.
    """
    scale = body_scale(joints)
    if scale is None:
        return None
    margin = margin_ratio * scale

    best_side, best_lift = None, 0.0
    for side in SIDES:
        pair = arm(joints, side)
        if pair is None:
            continue
        shoulder, wrist = pair
        # Image y grows downwards, so a raised wrist sits at a *smaller* y.
        lift = shoulder.y - wrist.y
        if lift > margin and lift > best_lift:
            best_side, best_lift = side, lift
    return best_side


class WaveTracker:
    """
    Watch one person's raised hand for the side-to-side motion of a wave.

    A wave is a raised hand whose horizontal position keeps reversing. The
    reversals are counted with an amplitude hysteresis, so camera noise and the
    small sway of a held-up arm do not accumulate into a wave; only excursions
    of at least `min_amplitude` body scales are turning points.

    One tracker follows one candidate person. Samples older than `window` are
    dropped, so the count always describes the last `window` seconds and a wave
    that stops decays instead of being remembered.
    """

    def __init__(self, window=1.5, min_amplitude=0.25, min_reversals=2):
        self.window = float(window)
        self.min_amplitude = float(min_amplitude)
        self.min_reversals = int(min_reversals)
        self._samples = []

    def update(self, now, joints):
        """Add this frame's sample and return True while a wave is in progress."""
        offset = _hand_offset(joints)
        if offset is None:
            # The hand went down or the person was lost: the wave is over, and
            # keeping the old samples would let a stale one re-fire later.
            self._samples.clear()
            return False

        self._samples.append((float(now), offset))
        cutoff = float(now) - self.window
        self._samples = [entry for entry in self._samples if entry[0] >= cutoff]
        return self.reversals() >= self.min_reversals

    def reversals(self):
        """Return how many direction changes the hand made inside the window."""
        return count_reversals(
            [offset for _, offset in self._samples], self.min_amplitude
        )


def _hand_offset(joints):
    """Return a raised hand's sideways offset from the torso, in body scales."""
    side = raised_hand(joints)
    if side is None:
        return None
    scale = body_scale(joints)
    torso = torso_centre(joints)
    if scale is None or torso is None or scale <= 0.0:
        return None
    return (joints[f"{side}_wrist"].x - torso[0]) / scale


def count_reversals(values, min_amplitude):
    """
    Count direction changes in a series, ignoring excursions below an amplitude.

    A turning point is only registered once the series has moved `min_amplitude`
    away from the last extreme, so a monotone drift with noise on top counts as
    zero reversals rather than one per sample.
    """
    if len(values) < 2:
        return 0

    reversals = 0
    direction = 0
    extreme = values[0]
    for value in values[1:]:
        delta = value - extreme
        if abs(delta) < min_amplitude:
            continue
        step = 1 if delta > 0.0 else -1
        if direction and step != direction:
            reversals += 1
        direction = step
        extreme = value
    return reversals


def attention_signal(joints, tracker, now, mode=ANY, margin_ratio=0.15):
    """
    Return the `AttentionSignal` this person is making, or None.

    `tracker` is the `WaveTracker` following this candidate; it is always
    updated, even in `RAISED_HAND` mode, so the answer does not depend on which
    mode was in force on earlier frames.
    """
    waving = tracker.update(now, joints)
    side = raised_hand(joints, margin_ratio)

    if mode in (WAVE, ANY) and waving and side is not None:
        return AttentionSignal(WAVE, side)
    if mode in (RAISED_HAND, ANY) and side is not None:
        return AttentionSignal(RAISED_HAND, side)
    return None


def pointing_cue(
    joints,
    min_extension_ratio=1.1,
    full_extension_ratio=1.5,
    min_lateral_ratio=0.45,
):
    """
    Decode the pointing arm into an azimuth, or return None if nobody points.

    Three gates have to pass, in body-scale units:

    * `min_extension_ratio` -- the wrist has to be far enough from the shoulder
      that the arm is actually held out rather than bent at the side;
    * the wrist has to be above the hip line, which is what separates a pointing
      arm from one hanging down (a hanging arm clears the extension gate on its
      own, being just as long);
    * `min_lateral_ratio` -- the arm has to reach sideways, not towards or away
      from the camera, where the azimuth is not recoverable.

    The azimuth is measured from the direction the person is facing, positive
    towards *their* left, and comes from how far the wrist reaches sideways
    against `full_extension_ratio`, the sideways reach of a fully outstretched
    arm. Treating that ratio as the sine of the angle makes a half-extended arm
    30 degrees rather than 45, which matches an arm swinging on its shoulder.

    When both arms qualify the more extended one wins, so the gesturing arm is
    picked over one that merely happens to be raised.
    """
    scale = body_scale(joints)
    hips_y = hip_line_y(joints)
    if scale is None or scale <= 0.0:
        return None

    best = None
    for side in SIDES:
        pair = arm(joints, side)
        if pair is None:
            continue
        shoulder, wrist = pair

        if hips_y is not None and wrist.y > hips_y:
            continue  # the arm hangs down; image y grows downwards

        extension = math.hypot(wrist.x - shoulder.x, wrist.y - shoulder.y) / scale
        if extension < min_extension_ratio:
            continue

        lateral_ratio = (wrist.x - shoulder.x) / scale
        if abs(lateral_ratio) < min_lateral_ratio:
            continue

        if best is None or extension > best.extension:
            best = PointingCue(
                side=side,
                azimuth=azimuth_from_lateral_ratio(lateral_ratio, full_extension_ratio),
                lateral_ratio=lateral_ratio,
                extension=extension,
            )
    return best


def azimuth_from_lateral_ratio(lateral_ratio, full_extension_ratio=1.5):
    """
    Convert a sideways arm reach into an azimuth, positive towards the person's left.

    The person's left arm reaches towards image-*right* when they face the
    camera, and their left is also where a positive azimuth points, so the two
    signs already agree and no flip is needed here.
    """
    if full_extension_ratio <= 0.0:
        return 0.0
    sine = max(-1.0, min(1.0, lateral_ratio / full_extension_ratio))
    return math.asin(sine)
