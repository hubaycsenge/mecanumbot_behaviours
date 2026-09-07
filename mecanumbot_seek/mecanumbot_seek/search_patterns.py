"""
Where to look, once the object is not where it was supposed to be.

The pattern is an **expanding ring search** around the last known place: a lap
at one radius, then a wider lap, then wider again. It is the pattern a person
uses looking for dropped keys, and it has the property that matters here --
every point at radius *r* is visited before any point at radius *r + step*, so
the places the object is most likely to be are searched first.

The rings are driven by the SEEKING circuit rather than by a fixed schedule.
`SeekingDrive.search_radius()` grows as expectancy falls, so a robot that still
believes the object is nearby works a tight ring, and one that has been
disappointed several times widens out. That is the whole behavioural content of
the drive: the same tree searches differently depending on how it has been going.

Two things this module does *not* do. It does not check the map -- a waypoint in
a wall is nav2's to refuse, and filtering here would mean this package carrying
an occupancy grid it has no other use for. And it does not decide when to stop:
that is extinction, and it belongs to the drive.

No ROS types: the poses are built by the caller from the `(x, y, yaw)` triples
these functions return.
"""

import math

# Full circle, spelled once.
TAU = 2.0 * math.pi


def ring(centre, radius, count, phase=0.0):
    """
    Return `count` waypoints evenly spaced around a circle, as `(x, y, yaw)`.

    Each waypoint faces **inwards**, towards the centre. That is the whole point
    of walking a ring around a place: the robot is looking at the region it is
    circling, not out into the room beyond it.

    `phase` rotates the ring, so successive laps do not all start at the same
    bearing and re-drive the same first leg.
    """
    count = max(1, int(count))
    points = []
    for index in range(count):
        angle = phase + TAU * index / count
        x = centre[0] + radius * math.cos(angle)
        y = centre[1] + radius * math.sin(angle)
        # Facing inwards is the bearing back to the centre, which is the
        # outward bearing turned through half a circle.
        points.append((x, y, _wrap(angle + math.pi)))
    return points


def ring_count(radius, spacing, minimum=4, maximum=16):
    """
    Waypoints needed on a ring of `radius` to stand every `spacing` metres.

    Bounded at both ends: a tight ring still gets enough stops to see all the
    way round, and a wide one does not turn into a fifty-waypoint route the
    robot will never finish.
    """
    if radius <= 0.0 or spacing <= 0.0:
        return int(minimum)
    return int(max(minimum, min(maximum, round(TAU * radius / spacing))))


def expanding_search(centre, radii, spacing=1.5, minimum=4, maximum=16):
    """
    Build the whole search: one ring per radius, innermost first.

    Each ring is phase-shifted by half its own spacing so the laps do not all
    begin at the same bearing, which would send the robot back and forth along
    one radial line between laps.
    """
    waypoints = []
    for index, radius in enumerate(radii):
        count = ring_count(radius, spacing, minimum, maximum)
        phase = (TAU / count) * 0.5 * (index % 2)
        waypoints.extend(ring(centre, radius, count, phase=phase))
    return waypoints


def radii_up_to(limit, first, step):
    """
    Return the ring radii from `first` out to `limit`, `step` apart.

    Always at least one ring: a limit smaller than `first` still means "look
    around where it was", not "do not look".
    """
    first = max(1e-3, float(first))
    step = max(1e-3, float(step))
    radii = []
    radius = first
    while radius <= float(limit) + 1e-9:
        radii.append(radius)
        radius += step
    return radii or [first]


def nearest_unvisited(waypoints, position, visited, reached_distance):
    """
    Index of the first waypoint not yet visited, or None when the search is done.

    "First" and not "nearest": the order *is* the search, because it is what
    puts the likely places before the unlikely ones. The position is used only
    to skip waypoints the robot has already driven past on its way somewhere
    else -- free progress, which on an expanding ring happens constantly.
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
    """Say whether every waypoint of the search has been visited."""
    return len(visited) >= len(waypoints)


def _wrap(angle):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
