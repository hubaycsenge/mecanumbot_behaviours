"""
Why the robot cannot have the thing it found, in terms it can tell someone.

Seeking can end three ways: the robot gets the object, the robot never finds it,
or -- the case this module is about -- **the robot finds it and cannot have it**.
That third outcome is not a failure of the search, and treating it as one throws
away the most useful thing the robot learned. It knows where the object is. It
should say so.

The reasons are deliberately few and deliberately about *the robot's body*,
because that is what makes them actionable to a person:

    too_high      above what the grabbers can close on -- on a table, a shelf
    too_low       below the shafts -- in a recess, under something
    no_route      nav2 could not get the robot there at all
    grip_failed   the robot reached it and the grabbers caught nothing
    lost          it was seen once and could not be found again

**`too_high` is the one the point cloud makes possible.** A 2D occupancy grid
has no height axis; from the lidar alone, a mug on a table and a mug on the
floor behind a chair are the same fact -- "something is at (x, y) and the robot
cannot get to it". The Deep3R hypothesis carries a z, so the robot can
distinguish "I cannot reach it because it is up there" from "I cannot reach it
because something is in the way", and those are different things to tell a
person. It is the clearest single payoff of having the reconstruction at all.

The grasp band comes from the hardware: the grabber shafts sit at z ~ 0.034 m
with a 0.116 m clear gap, so anything between roughly 0.03 and 0.15 m fits with
room to close. It is a horizontal pincer with no lift, so height is not
something the robot can do anything about -- which is exactly why it has to ask.

No ROS here: `assess` takes numbers and returns a string, so the whole policy is
testable at a desk.
"""

# The outcomes.
REACHABLE = "reachable"
TOO_HIGH = "too_high"
TOO_LOW = "too_low"
NO_ROUTE = "no_route"
GRIP_FAILED = "grip_failed"
LOST = "lost"

REASONS = (TOO_HIGH, TOO_LOW, NO_ROUTE, GRIP_FAILED, LOST)

# What each reason should be said as. Written to be read out or shown as-is:
# a person being told about it wants to know what to do, not what went wrong.
PHRASES = {
    TOO_HIGH: "it is up out of my reach",
    TOO_LOW: "it is down where my grabbers cannot get under it",
    NO_ROUTE: "I cannot get to it",
    GRIP_FAILED: "I reached it but could not grip it",
    LOST: "I saw it and then lost it",
}


def assess(
    height=None,
    distance=None,
    grasp_height_min=0.03,
    grasp_height_max=0.15,
    grasp_distance=0.30,
):
    """
    Say whether the object can be gripped, and if not, why.

    Height is checked before distance because it is the more informative answer
    and the one that does not change: a robot standing right next to a mug on a
    table is close enough and still cannot have it, and reporting that as
    "I could not get close enough" would send a person looking for an obstacle
    that is not there.

    An unknown height (`None`) is *not* treated as unreachable. The server may
    not have placed the object in z, and refusing to try on that basis would
    make the robot give up on everything on the floor whenever the cloud was
    thin. The grasp itself is the check that always runs.
    """
    if height is not None:
        if height > grasp_height_max:
            return TOO_HIGH
        if height < grasp_height_min:
            return TOO_LOW
    if distance is not None and distance > grasp_distance:
        return NO_ROUTE
    return REACHABLE


def reachable(reason):
    """Say whether an assessment means the robot may go ahead and grip."""
    return reason == REACHABLE


def describe(reason, object_class="", height=None):
    """
    Build the line the robot alerts a person with.

    Names the object where one is known, because "I found it but cannot reach
    it" is much less use than "I found the mug"; and gives the height for
    `too_high`, which is the number that tells somebody whether it is a table
    or a shelf.
    """
    thing = object_class or "it"
    phrase = PHRASES.get(reason, "I cannot get it")
    line = f"I found the {thing} but {phrase}"
    if reason == TOO_HIGH and height is not None:
        line += f" -- it is {height:.2f} m up"
    return line


def grasp_band(grasp_height_min=0.03, grasp_height_max=0.15):
    """Return the height band the grabbers can close on, for a log line."""
    return f"{grasp_height_min:.2f}..{grasp_height_max:.2f} m"
