"""
Pointing the head at the body the LiDAR says should be there.

The camera sits about 0.23 m off the floor behind a lens that sees 30 degrees
top to bottom, so there is no one tilt that shows it a person. With the head
lifted to the seeking pose it sees a standing person's legs when they are close,
and only the face of somebody sitting low, at the bottom edge of the frame; with
it level it sees somebody on a bean bag and loses every standing head inside
four metres. The bags of 2026-09-30 have both: the robot looking at a pair of
jeans from 0.7 m, and turning past a person on the bean bag for a minute and a
half. The seeking pose has been 7.0 and 6.0 in the constants file, and the robot
ran those trials at 6.5; each is right for one of the two and wrong for the
other.

What decides the right tilt is **how far away the body is and how tall it is
expected to be**, and the robot knows both before the camera has seen anything:

* the LiDAR says how far. Anything it sees that is the size of a person and is
  not part of the mapped room is a place a person may be standing
  (`lidar_candidates`, published by the fusion node), and a person the fusion is
  already tracking is a better answer still. The nearer the body, the further up
  the head has to look to keep its top in the frame;
* a body is expected to be a **standing** one, shoulders at 1.4 m and a head
  above them -- except where the room says otherwise. A **seat** is a known
  place in the map where a body is lower: a chair, a bean bag. Somebody there is
  looked for at the height a sitting person is. A seat is also looked at when
  the LiDAR reports nothing on it at all, because that is exactly the case the
  LiDAR is bad at: a person in a chair against the wall is one return with the
  wall, and a bean bag that was in the room when it was mapped is part of the
  map.

Each thing in view asks for a band of heights to be in the frame -- the part of
that body the pose detector needs, hips to the top of the head -- which at its
range is an interval of tilts. The head takes the tilt in the middle of the
first target's interval, moved as little as it must be to satisfy the others
too. A body too tall for the frame at its range keeps its top: a face and two
shoulders are seven keypoints, which the detection gate accepts, and knees are
not.

No ROS here. `ros_interfaces.BodyGaze` feeds this the robot's pose, the tracked
people and the candidates, and sends what it returns.
"""

import math
from dataclasses import dataclass

STANDING = "standing"
CHAIR = "chair"
BEAN_BAG = "bean_bag"

# The band of heights [m] the pose detector needs of each kind of body: from
# the hips to the top of the head. Standing is sized for an adult; the seated
# ones are what the bags of 2026-09-30 measured, with room above -- shoulders
# at ~0.95 m and the head at ~1.1 m on the chair by the wall, shoulders at
# ~0.4 m and the nose at ~0.55 m on the bean bag. The top of a band matters
# more than its bottom: close in, a body is taller than the frame, the top is
# what is kept, and every centimetre of air above the head is one taken from
# the shoulders -- at 1.1 m the whole frame is 0.55 m tall.
BODY_PROFILES = {
    STANDING: (0.85, 1.80),
    CHAIR: (0.40, 1.30),
    BEAN_BAG: (0.10, 0.75),
}


@dataclass(frozen=True)
class NeckModel:
    """
    Which way the camera looks for a neck position, and back.

    `n_pos` is the servo position in the units the trees command (board ticks
    over 100). The numbers are the ones `mecanumbot_sensorprocess_smart` places
    a ball with (`ball.neck.*`, measured 2026-09-15 and 2026-09-18) and
    `mecanumbot_deep3r` reconstructs with: 0.005061 rad per tick, the lens
    looking 4.7 degrees up at 600 ticks. They describe the same servo, so change
    all three or none.
    """

    level_pos: float = 6.0
    pitch_at_level: float = math.radians(4.7)
    rad_per_pos: float = 0.5061
    min_pos: float = 2.0
    max_pos: float = 8.6
    # Where the lens is: metres above the floor (it moves by a centimetre over
    # the whole tilt range, which is ignored) and metres ahead of the LiDAR,
    # which is where every range here is measured from.
    camera_height: float = 0.23
    camera_forward: float = 0.13
    # The lens: 51 degrees across (measured 2026-09-15), 16:9.
    hfov: float = math.radians(51.0)
    vfov: float = math.radians(30.0)

    def pitch(self, pos):
        """Return the camera's tilt [rad, up positive] at a neck position."""
        return self.pitch_at_level + (float(pos) - self.level_pos) * self.rad_per_pos

    def pos(self, pitch):
        """Return the neck position for a tilt, inside what the servo can do."""
        pos = self.level_pos + (float(pitch) - self.pitch_at_level) / self.rad_per_pos
        return max(self.min_pos, min(self.max_pos, pos))

    def visible_band(self, pos, distance):
        """Return the heights [m] in frame at `distance` from the LiDAR."""
        reach = max(float(distance) - self.camera_forward, 0.05)
        pitch = self.pitch(pos)
        return (
            self.camera_height + reach * math.tan(pitch - self.vfov / 2.0),
            self.camera_height + reach * math.tan(pitch + self.vfov / 2.0),
        )


@dataclass(frozen=True)
class Seat:
    """
    A place in the map where a body is lower than a standing one.

    `kind` names the body profile; `low` and `high` override it for a seat that
    is neither of the usual ones. `radius` is how far from `(x, y)` something
    still counts as being on the seat -- generous, because a bean bag is not
    where it was last week: the one in the ethology lab is 0.5 m from where the
    map has it.
    """

    kind: str
    x: float
    y: float
    radius: float = 0.6
    low: float = None
    high: float = None

    def holds(self, x, y):
        """Say whether a map point is on this seat."""
        return math.hypot(float(x) - self.x, float(y) - self.y) <= self.radius


def seat_from_literal(literal):
    """
    Build a :class:`Seat` from a constants-file entry.

    An entry is `{'seat': 'bean_bag', 'x': -0.7, 'y': 2.25, 'radius': 0.7}`,
    with `radius`, `low` and `high` optional. The coordinates are spelled in
    lower case on purpose: `X`/`Y` is a route checkpoint, and the loader would
    turn the entry into a `Point` and drop everything else in it.
    """
    try:
        return Seat(
            kind=str(literal["seat"]),
            x=float(literal["x"]),
            y=float(literal["y"]),
            radius=float(literal.get("radius", Seat.radius)),
            low=None if literal.get("low") is None else float(literal["low"]),
            high=None if literal.get("high") is None else float(literal["high"]),
        )
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(
            f"a seat is {{'seat': kind, 'x': .., 'y': ..}} and optionally "
            f"'radius', 'low', 'high'; got {literal!r} ({error})"
        ) from None


def profiles_from_literals(literals):
    """
    Build the body profiles from constants-file entries.

    Each entry is `{'body': 'chair', 'low': 0.4, 'high': 1.3}`. Kinds a file
    does not mention keep the packaged band, so a file only lists what it means
    to change.
    """
    profiles = dict(BODY_PROFILES)
    for literal in literals or ():
        try:
            low, high = float(literal["low"]), float(literal["high"])
            kind = str(literal["body"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"a body profile is {{'body': kind, 'low': .., 'high': ..}}; "
                f"got {literal!r} ({error})"
            ) from None
        if not low < high:
            raise ValueError(f"body profile '{kind}': low {low} is not below high {high}")
        profiles[kind] = (low, high)
    return profiles


@dataclass(frozen=True)
class GazeTarget:
    """Something to keep in frame: a body between two heights, at a range."""

    distance: float
    low: float
    high: float
    label: str


@dataclass(frozen=True)
class GazeSettings:
    """The tunables of the gaze; the constants file spells them `body_gaze_*`."""

    # Metres from the robot within which a body is worth tilting the head for.
    # Further off a whole person fits in the frame at any moderate tilt.
    max_range: float = 4.5
    # Nearer than this is the robot's own gripper, or a LiDAR artefact.
    min_range: float = 0.3
    # Radians beyond the edge of the picture a target is still attended to, so
    # the head is already at the right height as a turn brings it into shot.
    view_margin: float = math.radians(12.0)
    # Radians kept clear at the top and bottom of the frame: a body whose head
    # touches the edge is a truncated box.
    frame_margin: float = math.radians(3.0)
    # The neck positions the gaze may command.
    min_pos: float = 5.5
    max_pos: float = 8.2
    # Smallest change worth a command [n_pos; 0.1 is ~3 degrees], and the
    # shortest time between two [s]. Without them the head would follow the
    # range noise of whatever it is looking at.
    deadband: float = 0.1
    min_interval: float = 0.4
    # Seconds with nothing in view before the head goes back to the seeking
    # pose, so a target that drops out for one scan does not make it nod.
    release_delay: float = 1.0


def tilt_interval(neck, target, frame_margin):
    """
    Return `(lowest, highest, ideal)` tilt [rad] that keeps a target in frame.

    The band between `target.low` and `target.high` subtends an interval of
    elevations at its range; any tilt that puts both ends inside the frame
    (less the margin) will do, and the ideal is the one that centres it. A band
    taller than the frame has no such tilt, and then the answer is the one that
    keeps its top -- the head and shoulders -- with the margin above it.
    """
    reach = max(target.distance - neck.camera_forward, 0.2)
    bottom = math.atan2(target.low - neck.camera_height, reach)
    top = math.atan2(target.high - neck.camera_height, reach)
    half = neck.vfov / 2.0 - frame_margin
    lowest = top - half
    highest = bottom + half
    if lowest > highest:
        return lowest, lowest, lowest
    return lowest, highest, (bottom + top) / 2.0


def choose_tilt(neck, targets, frame_margin):
    """
    Return the tilt [rad] for a list of targets, most important first.

    The first target is always satisfied. Each later one narrows the choice if
    it can be satisfied as well and is passed over if it cannot: the head has
    one axis, and a person at 1 m and one at 3 m are not both in a 30 degree
    frame.
    """
    lowest, highest, ideal = tilt_interval(neck, targets[0], frame_margin)
    for target in targets[1:]:
        low, high, _ = tilt_interval(neck, target, frame_margin)
        if max(lowest, low) <= min(highest, high):
            lowest, highest = max(lowest, low), min(highest, high)
    return max(lowest, min(highest, ideal))


def _bearing_offset(robot, x, y):
    """Return how far off the robot's heading a map point is [rad, unsigned]."""
    bearing = math.atan2(y - robot[1], x - robot[0]) - robot[2]
    return abs((bearing + math.pi) % (2.0 * math.pi) - math.pi)


def gaze_targets(
    robot, people, candidates, seats, profiles, neck=None, settings=None
):
    """
    Return what is in view and worth tilting the head for, most important first.

    `robot` is `(x, y, yaw)` in the map; `people` are the map positions the
    fusion is tracking, `candidates` the ones the LiDAR only suspects. Tracked
    people come first, nearest first; then candidates and seats together,
    nearest first -- nearest, because that is where the tilt matters: a body at
    four metres is in frame at almost any of them.

    A point on a seat is looked for at that seat's height. A seat with nothing
    on it is still a target, once: see the module docstring for why.
    """
    neck = neck or NeckModel()
    settings = settings or GazeSettings()
    profiles = profiles or BODY_PROFILES
    half_view = neck.hfov / 2.0 + settings.view_margin

    def visible(x, y):
        distance = math.hypot(x - robot[0], y - robot[1])
        if not settings.min_range <= distance <= settings.max_range:
            return None
        if _bearing_offset(robot, x, y) > half_view:
            return None
        return distance

    def seat_of(x, y):
        on = [seat for seat in seats if seat.holds(x, y)]
        if not on:
            return None
        return min(on, key=lambda seat: math.hypot(x - seat.x, y - seat.y))

    def band(seat):
        if seat is None:
            return profiles[STANDING]
        low, high = profiles.get(seat.kind, profiles[STANDING])
        return (
            low if seat.low is None else seat.low,
            high if seat.high is None else seat.high,
        )

    taken = set()

    def target(x, y, what):
        distance = visible(x, y)
        if distance is None:
            return None
        seat = seat_of(x, y)
        if seat is not None:
            taken.add(seat)
        low, high = band(seat)
        posture = STANDING if seat is None else seat.kind
        return GazeTarget(distance, low, high, f"{what} ({posture}) at {distance:.1f} m")

    tracked = [target(x, y, "a tracked person") for x, y in people]
    tracked = sorted((t for t in tracked if t), key=lambda t: t.distance)

    suspected = [target(x, y, "a LiDAR candidate") for x, y in candidates]
    suspected = [t for t in suspected if t]
    for seat in seats:
        if seat in taken:
            continue
        distance = visible(seat.x, seat.y)
        if distance is None:
            continue
        low, high = band(seat)
        suspected.append(
            GazeTarget(distance, low, high, f"the {seat.kind} seat at {distance:.1f} m")
        )
    suspected.sort(key=lambda t: t.distance)
    return tracked + suspected


class BodyGazeController:
    """
    The gaze's state: what the head was last told, and when it last had a target.

    `update()` returns the neck position to command now, or None to leave the
    head alone, and leaves a description of what it is looking at in `reason`.
    """

    def __init__(self, seats=(), profiles=None, neck=None, settings=None):
        self.seats = tuple(seats)
        self.profiles = dict(profiles or BODY_PROFILES)
        self.neck = neck or NeckModel()
        self.settings = settings or GazeSettings()
        self.reason = ""
        self._last_command = None
        self._last_target = None

    def desired(self, robot, people, candidates):
        """Return `(neck position, why)` for what is in view, or `(None, "")`."""
        targets = gaze_targets(
            robot,
            people,
            candidates,
            self.seats,
            self.profiles,
            self.neck,
            self.settings,
        )
        if not targets:
            return None, ""
        tilt = choose_tilt(self.neck, targets, self.settings.frame_margin)
        pos = self.neck.pos(tilt)
        pos = max(self.settings.min_pos, min(self.settings.max_pos, pos))
        return round(pos, 2), targets[0].label

    def update(self, now, robot, people, candidates, rest_pos, current_pos):
        """
        Return the neck position to command, or None.

        `rest_pos` is where the head goes with nothing to look at -- the
        seeking pose -- and `current_pos` the last position it was commanded
        to, None if it has never been moved.
        """
        if robot is None:
            return None
        wanted, why = self.desired(robot, people, candidates)
        if wanted is None:
            held = self._last_target is not None and (
                now - self._last_target < self.settings.release_delay
            )
            if held:
                return None
            wanted, why = float(rest_pos), "nothing in view, seeking pose"
        else:
            self._last_target = now

        if current_pos is not None and (
            abs(wanted - float(current_pos)) < self.settings.deadband
        ):
            return None
        if self._last_command is not None and (
            now - self._last_command < self.settings.min_interval
        ):
            return None
        self._last_command = now
        self.reason = why
        return wanted
