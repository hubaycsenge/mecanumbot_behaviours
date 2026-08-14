"""
When the dog looks back, and how far it drives between two look backs.

A dog leading somebody does not check on them at every step, and it does not
ignore them for the whole walk either: it looks back when enough ground has been
covered, when enough time has passed, or when something about the human says it
should -- they have gone quiet, or they have dropped behind. Everything else is
walking.

That decision is the pacing of the whole leading loop, so it lives here as plain
functions: no ROS, no py_trees, no blackboard. `DogCheckInDue` measures the
world and asks `check_in_due()`; `FollowRoute` asks `route_leg()` how far it may
drive before the answer turns to yes and sends exactly that many waypoints in
one nav2 goal. Keeping the two answers in one module is what stops the robot
from planning past a look back it is about to owe.

This module deliberately imports nothing: it is the part of the leading
behaviour that can be reasoned about, and tested, on its own.
"""


def check_in_due(
    checkpoints_since,
    seconds_since,
    subject_age,
    subject_distance,
    every_checkpoints,
    interval,
    follow_threshold,
    stale_after,
):
    """
    Decide whether the robot should look back for its human now.

    Return `(due, reason)`, the reason being the phrase that goes in the log so
    that a run can be read back afterwards.

    The three "something is wrong" conditions come first, because they are the
    reason to look back *now* rather than at the next checkpoint: the human was
    never seen, was last seen longer than `stale_after` seconds ago, or stands
    further away than `follow_threshold`. After them come the two habits, a look
    back every `every_checkpoints` checkpoints and every `interval` seconds;
    either counter set to zero or less switches that habit off.

    `subject_distance` may be None when nothing is known about where the human
    is, which is not by itself a reason to check -- `subject_age` already covers
    a human who has not been seen.
    """
    if subject_age is None:
        return True, "the human has not been seen at all yet"
    if stale_after is not None and subject_age > stale_after:
        return True, f"the human was last seen {subject_age:.1f} s ago"
    if subject_distance is not None and subject_distance > follow_threshold:
        return True, (
            f"the human is {subject_distance:.2f} m away, "
            f"more than the {follow_threshold:.2f} m allowed"
        )
    if every_checkpoints > 0 and checkpoints_since >= every_checkpoints:
        return True, f"{int(checkpoints_since)} checkpoint(s) since the last look back"
    if interval > 0.0 and seconds_since >= interval:
        return True, f"{seconds_since:.0f} s since the last look back"
    return False, "the human is following along"


def checkpoint_budget(checkpoints_since, every_checkpoints, route_lookahead):
    """
    How many checkpoints one drive may cover before a look back falls due.

    Never less than one: a robot that owes a look back still has to get to the
    checkpoint it is going to make it from.
    """
    budget = int(route_lookahead) if route_lookahead > 0 else 1
    if every_checkpoints > 0:
        budget = min(budget, int(every_checkpoints) - int(checkpoints_since))
    return max(1, budget)


def route_leg(current, last, checkpoints_since, every_checkpoints, route_lookahead):
    """
    Checkpoint indices one nav2 waypoint goal covers, starting at `current`.

    The leg is as long as the checkpoint budget allows and stops at the end of
    the route. It always contains at least one index, so a leg is always
    something to drive.
    """
    last = max(0, int(last))
    current = max(0, min(int(current), last))
    budget = checkpoint_budget(checkpoints_since, every_checkpoints, route_lookahead)
    return list(range(current, min(last, current + budget - 1) + 1))


def sweep_pattern(step):
    """
    Rotations of the look-around sweep, relative to the heading it starts on.

    `+1, -2, +2, -1` steps: the robot looks to one side of where it last saw the
    human, then across to the other, and ends facing the middle again -- so a
    sweep that finds nobody leaves the robot pointing where the human ought to
    have been, which is where the recovery patrol wants to set off from.
    """
    return (step, -2.0 * step, 2.0 * step, -step)
