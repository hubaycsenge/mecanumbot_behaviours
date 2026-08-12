"""
COCO-17 keypoint access for the camera people detections.

`mecanumbot_sensorprocess_smart` publishes one `PersonKeypoints` per detected
person: 17 joints in normalized image coordinates, x to the right and y *down*,
both in [0, 1]. This module is the only place that knows that layout, so the
gesture decoders above it work on named joints instead of message fields.

The two camera detectors spell "this joint was not found" differently. The
Ultralytics node stores whatever `keypoints.xyn` returned, which is exactly
`(0.0, 0.0)` for a missing joint; the DeepStream node writes `NaN` for anything
below its visibility threshold. Both have to count as invisible here, and
`(0.0, 0.0)` is safe to reject because it is the top-left pixel -- no real joint
of a framed person lands there.

`mirror_joints` is the other thing this module knows about the image: whether
the camera hands over a mirrored view of the room. Everything downstream --
which way the robot turns to keep somebody centred, which way their arm points
-- is read off image x, so if the image is reflected then every one of those
directions is reflected with it. Undoing it once, here, is what keeps the rest
of the package working in a single convention.

Everything is plain arithmetic on floats, so it is unit-tested without a ROS
graph or a camera.
"""

import math

# COCO-17 order, and at the same time the field names of
# `mecanumbot_msgs/PersonKeypoints`. The detectors fill the message in this
# order, so index and name agree.
KEYPOINT_FIELDS = (
    "nose",
    "left_eye",
    "right_eye",
    "left_ear",
    "right_ear",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
)

# "left" and "right" are the *person's* left and right, as the pose model names
# them. Facing the camera, the person's left arm appears on the right-hand side
# of the image; `gestures.py` is where that flip is turned into a direction.
LEFT = "left"
RIGHT = "right"
SIDES = (LEFT, RIGHT)


class Keypoint:
    """One body joint in normalized image coordinates, or a missing one."""

    __slots__ = ("x", "y", "visible")

    def __init__(self, x, y, visible):
        self.x = float(x)
        self.y = float(y)
        self.visible = bool(visible)

    def __repr__(self):
        """Return a debugging representation naming the coordinates."""
        if not self.visible:
            return "Keypoint(missing)"
        return f"Keypoint(x={self.x:.3f}, y={self.y:.3f})"


MISSING = Keypoint(0.0, 0.0, False)


def keypoint(x, y):
    """Build a keypoint, deciding visibility from the detectors' two conventions."""
    if math.isnan(x) or math.isnan(y):
        return MISSING
    if x == 0.0 and y == 0.0:
        return MISSING
    return Keypoint(x, y, True)


def keypoints_from_msg(message, mirror=False):
    """
    Convert a `mecanumbot_msgs/PersonKeypoints` into a name -> keypoint dict.

    With `mirror` set the pose is reflected on the way in, for a camera whose
    image is a mirrored view of the room; see `mirror_joints`.
    """
    joints = {}
    for field in KEYPOINT_FIELDS:
        position = getattr(message, field).position
        joints[field] = keypoint(position.x, position.y)
    return mirror_joints(joints) if mirror else joints


def opposite_side(name):
    """Return a joint name with its side swapped; unsided names come back as they are."""
    if name.startswith(f"{LEFT}_"):
        return f"{RIGHT}_{name[len(LEFT) + 1:]}"
    if name.startswith(f"{RIGHT}_"):
        return f"{LEFT}_{name[len(RIGHT) + 1:]}"
    return name


def mirror_keypoint(point):
    """Reflect one joint about the vertical centre line of the image."""
    if not point.visible:
        return point  # `MISSING` has no position to reflect
    return Keypoint(1.0 - point.x, point.y, True)


def mirror_joints(joints):
    """
    Return the pose a mirrored image would have shown of the real body.

    Two things are reflected together, and they have to be, because a mirrored
    image mirrors both:

    * the **coordinates** -- a joint seen at image x is really at `1 - x`, which
      is what puts the person back on the side of the robot they stand on;
    * the **side labels** -- the pose model names joints by how the body looks,
      so in a mirrored image it calls the person's left arm their right one.

    Image y is untouched: a horizontal mirror leaves heights alone, and every
    gesture that reads a wrist against a shoulder or a hip is unaffected by it.
    """
    return {
        opposite_side(name): mirror_keypoint(point) for name, point in joints.items()
    }


def visible(joints, *names):
    """Return True when every named joint was found."""
    return all(joints[name].visible for name in names)


def midpoint(joints, first, second):
    """Return the midpoint of two joints, or None if either is missing."""
    if not visible(joints, first, second):
        return None
    a, b = joints[first], joints[second]
    return ((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)


def torso_centre(joints):
    """
    Return the (x, y) the person is tracked by, or None.

    The shoulder midpoint is preferred because it is the steadiest point on a
    standing person and is still there when the legs are cropped or occluded;
    the hips are the fallback for someone turned far enough that one shoulder
    is lost.
    """
    return midpoint(joints, "left_shoulder", "right_shoulder") or midpoint(
        joints, "left_hip", "right_hip"
    )


def shoulder_width(joints):
    """Return the shoulder separation in image units, or None."""
    if not visible(joints, "left_shoulder", "right_shoulder"):
        return None
    left, right = joints["left_shoulder"], joints["right_shoulder"]
    return math.hypot(left.x - right.x, left.y - right.y)


def body_scale(joints):
    """
    Return the image-space size the gesture thresholds are measured in, or None.

    Every gesture threshold is a ratio against this, so a person two metres away
    has to reach as far -- in their own body's units -- as one standing close.
    Shoulder width is the primary measure. A person turned side-on to the camera
    foreshortens it towards zero, so the shoulder-to-hip height takes over below
    a quarter of that height, which is well under any real shoulder width and
    well above the noise on a frontal pose.
    """
    width = shoulder_width(joints)
    shoulders = midpoint(joints, "left_shoulder", "right_shoulder")
    hips = midpoint(joints, "left_hip", "right_hip")
    if shoulders is None or hips is None:
        return width if width and width > 0.0 else None

    height = math.hypot(shoulders[0] - hips[0], shoulders[1] - hips[1])
    if height <= 0.0:
        return width if width and width > 0.0 else None
    if width is None or width < 0.25 * height:
        return 0.25 * height
    return width


def hip_line_y(joints):
    """Return the image y of the hips, or None -- the line a raised arm is above."""
    hips = midpoint(joints, "left_hip", "right_hip")
    return None if hips is None else hips[1]


def image_centroid_x(joints):
    """
    Return the image x the person is centred on, or None.

    The torso is used when it was found, because an outstretched arm drags the
    mean of all joints sideways -- and this value is exactly what the robot
    turns on, so a person who points must not appear to have stepped aside.
    """
    torso = torso_centre(joints)
    if torso is not None:
        return torso[0]
    xs = [joint.x for joint in joints.values() if joint.visible]
    return sum(xs) / len(xs) if xs else None


def arm(joints, side):
    """Return the (shoulder, wrist) keypoints of one arm, or None if incomplete."""
    shoulder = joints[f"{side}_shoulder"]
    wrist = joints[f"{side}_wrist"]
    if not (shoulder.visible and wrist.visible):
        return None
    return (shoulder, wrist)
