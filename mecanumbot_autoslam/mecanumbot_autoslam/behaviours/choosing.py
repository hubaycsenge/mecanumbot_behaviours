"""
Where to go next: the best frontier, or a region the server is unsure about.

The whole decision is `select()`, and it takes no ROS types, so it is a
function to test rather than a run to watch.

**Why interleave rather than prioritise.** The server's uncertain regions are
about reconstruction *quality* in space the robot has already mapped. Servicing
them first sounds right and is not: a feed-forward reconstruction keeps finding
new things to be unsure about as it goes, so a robot that always answers the
newest complaint stalls in the first room while the rest of the building stays
unmapped. Every `uncertain_every` goals, one of them is the server's. That is
the only feedback a feed-forward model can use -- more frames from a better
viewpoint -- and it is capped so it cannot eat the pass.

**Which** regions are worth a look, and when one counts as looked at, are not
decided here at all. `mecanumbot_map_agreement` publishes the list and drops a
region once the robot has been near it, because the same policy has to hold in
T2, when no explorer is running.
"""

#: The goal came from the frontier detector.
FRONTIER = "frontier"

#: The goal came from the server's uncertain-region list.
UNCERTAIN = "uncertain"


def select(best, revisit_points, goals_sent, uncertain_every=3,
           revisit_uncertain=True):
    """
    Return `(point, source)` for the next goal, or `(None, "")` if there is none.

    `best` is the top-scoring frontier or None; `revisit_points` is the
    server's list, best first; `goals_sent` is how many goals this pass has
    sent, which is what makes every Nth one the server's.

    A frontier is preferred on every other tick, and the uncertain list is
    still used when the detector has nothing -- a pass with no frontiers left
    but regions outstanding should go and look at them, not stop early.
    """
    uncertain = list(revisit_points) if revisit_uncertain else []
    every = max(1, int(uncertain_every))

    if uncertain and goals_sent % every == 0:
        return uncertain[0], UNCERTAIN
    if best is not None:
        return best.point, FRONTIER
    if uncertain:
        return uncertain[0], UNCERTAIN
    return None, ""
