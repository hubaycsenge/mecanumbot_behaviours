#!/usr/bin/env python3
"""
How the robot looks for a ball: where it drives, and where it points its head.

The ball is small, it is on the floor, and it is somewhere in the room. Those
three facts settle the shape of the search.

## Circles on the ground

The robot walks a **circle** and then a wider circle, out from where it started.
A circle rather than a lawnmower sweep because the constraint here is the
camera, not the floor: the lens sees about 60 degrees, so what matters is that
the robot ends up pointing in every direction from a spread of places, and going
round is the cheapest way to do that with a differential goal at a time. It is
also the pattern `mecanumbot_seek` uses for the same reason, and the same
property holds -- everything at radius *r* is looked at before anything at
*r + step*, so the near floor is searched before the far.

The one difference from the seek rings is which way the robot faces at each
stop, and it is not cosmetic. Seek circles *around a place it was told about*,
so its waypoints face inwards at that place. Fetch is not circling anything: it
does not know where the ball is, and the interesting floor is the floor it has
not looked at. So the default is `tangent` -- face the way you are going -- which
sweeps the camera across new ground continuously as the robot moves round, and
`inward` / `outward` are there because "what does the robot look at while
searching" is a question a trial might want to answer differently.

## Height on the head

The camera is on a tilting neck about 0.2 m up, with a vertical field of view of
roughly 36 degrees. At one tilt it sees a band of floor and nothing else: tilted
down it sees from its own feet out to a couple of metres; level it sees from a
couple of metres to the far wall but not the floor in front of it. A ball
outside the current band is invisible, however good the detector is, so a single
tilt makes the search a function of luck.

`head_sweep` is therefore a triangle wave between two tilts, and it runs the
whole time the robot is circling. It is a search over the *third* dimension the
ground pattern cannot cover -- and it is also the reason `ball.range_source` in
the perception layer defaults to apparent size rather than the ground plane: the
tilt is moving, and nothing tells the fusion node what it currently is.

No ROS types: the caller builds poses from the `(x, y, yaw)` triples and sends
the tilt to the accessory commander.
"""

import math

# Full circle, spelled once.
TAU = 2.0 * math.pi

# Which way the robot faces at each stop on a circle.
FACING_TANGENT = "tangent"
FACING_INWARD = "inward"
FACING_OUTWARD = "outward"
FACINGS = (FACING_TANGENT, FACING_INWARD, FACING_OUTWARD)


def circle(centre, radius, count, phase=0.0, facing=FACING_TANGENT):
    """
    Return `count` stops evenly spaced around a circle, as `(x, y, yaw)`.

    `phase` rotates the whole circle, so successive laps do not all begin at the
    same bearing and re-drive the same first leg.

    The yaw is the search: `tangent` points the camera along the direction of
    travel, which is where the unlooked-at floor is; `inward` points it back at
    the centre, which is right only when the centre is where the ball is thought
    to be; `outward` points it at the walls.
    """
    count = max(1, int(count))
    stops = []
    for index in range(count):
        angle = phase + TAU * index / count
        x = centre[0] + radius * math.cos(angle)
        y = centre[1] + radius * math.sin(angle)
        stops.append((x, y, _facing_yaw(angle, facing)))
    return stops


def circle_count(radius, spacing, minimum=6, maximum=16):
    """
    Return the stops a circle of `radius` needs to stand every `spacing` metres.

    Bounded at both ends: a tight circle still gets enough stops to face all the
    way round, and a wide one does not become a route the robot never finishes.
    The floor is `minimum` rather than 1 because a circle with three stops is
    not a circle, it is a triangle the camera looks along the sides of.
    """
    if radius <= 0.0 or spacing <= 0.0:
        return int(minimum)
    return int(max(minimum, min(maximum, round(TAU * radius / spacing))))


def radii_up_to(limit, first, step):
    """
    Return the circle radii from `first` out to `limit`, `step` apart.

    Always at least one circle: a limit smaller than `first` still means "look
    around where you are", not "do not look".
    """
    first = max(1e-3, float(first))
    step = max(1e-3, float(step))
    radii = []
    radius = first
    while radius <= float(limit) + 1e-9:
        radii.append(radius)
        radius += step
    return radii or [first]


def expanding_circles(
    centre, radii, spacing=1.2, minimum=6, maximum=16, facing=FACING_TANGENT
):
    """
    Build the whole search: one circle per radius, tightest first.

    Each circle is phase-shifted by half its own spacing so the laps do not all
    begin at the same bearing, which would send the robot back and forth along
    one radial line between laps.
    """
    stops = []
    for index, radius in enumerate(radii):
        count = circle_count(radius, spacing, minimum, maximum)
        phase = (TAU / count) * 0.5 * (index % 2)
        stops.extend(circle(centre, radius, count, phase=phase, facing=facing))
    return stops


def nearest_unvisited(waypoints, position, visited, reached_distance):
    """
    Index of the first stop not yet visited, or None when the lap is done.

    "First" and not "nearest": the order *is* the search, because it is what
    puts the near floor before the far. The robot's position is used only to
    write off stops it has already driven past on its way somewhere else, which
    on a circle happens constantly.
    """
    for index, (x, y, _) in enumerate(waypoints):
        if index in visited:
            continue
        if math.hypot(x - position[0], y - position[1]) <= reached_distance:
            visited.add(index)
            continue
        return index
    return None


def sweep_complete(waypoints, visited):
    """Say whether every stop of the search has been visited."""
    return len(visited) >= len(waypoints)


def head_sweep(elapsed, low, high, period):
    """
    Neck tilt at a moment in the sweep: a triangle wave between two poses.

    A triangle and not a sine because the time is what is being spent: a sine
    lingers at both ends and hurries through the middle, and the middle of this
    range is the band a ball two metres away sits in. Linear means every height
    band gets the same share of the search.

    `low` and `high` are neck positions, not angles -- the accessory command is
    in the board's own units (about 2.0 to 8.6, larger looks further up), and
    converting them into angles here would invent a calibration this robot does
    not have.
    """
    period = float(period)
    if period <= 0.0:
        return float(low)
    # Fold the elapsed time into one up-and-down lap, then into [0, 1].
    phase = (float(elapsed) % period) / period
    ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    return float(low) + (float(high) - float(low)) * ramp


def _facing_yaw(angle, facing):
    """Yaw at a stop `angle` radians round the circle, for one facing rule."""
    if facing == FACING_INWARD:
        # Back towards the centre: the outward bearing turned half a circle.
        return _wrap(angle + math.pi)
    if facing == FACING_OUTWARD:
        return _wrap(angle)
    if facing == FACING_TANGENT:
        # The direction of travel, which is a quarter turn ahead of the radius
        # for a circle walked counter-clockwise.
        return _wrap(angle + math.pi / 2.0)
    raise ValueError(f"unknown facing '{facing}', expected one of {FACINGS}")


def _wrap(angle):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
