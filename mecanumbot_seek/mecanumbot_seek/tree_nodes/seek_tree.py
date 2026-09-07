"""
The seek behaviour tree: be told what to find, go and find it, pick it up.

One pass through `create_episode()` is one complete seeking episode, and the
tree is that episode repeated for ever:

    1. the robot is told what to look for and where the server last saw it;
    2. it looks -- two branches at once:
         a. it watches for the object, everywhere, the whole time;
         b. it drives to where the object was, scans there, and if it is not
            there walks a widening search around it;
    3. the moment (a) succeeds, (b) is abandoned mid-drive and the robot
       approaches what it can see;
    4. then one of two things:
         - it closes the grabbers on it, or
         - it cannot -- the object is on a table, in a recess, or behind
           something nav2 could not route around -- and it goes and **tells a
           person**, by driving up to them and alternating its orientation
           between them and the object;
    5. it releases the episode and waits to be asked for the next thing.

## Why an episode has two endings rather than one

An object located and not obtained is not a failed search. The robot found the
thing; what it could not do is pick it up, and it now knows something a person
would want to know. Ending that run as a failure throws away the most useful
result the robot produced.

The branch is `CheckObjectGraspable`, and it turns on the object's **height** --
the one number the 2D map cannot supply. From the lidar alone, a mug on a table
and a mug on the floor behind a chair are the same fact: "something is at (x, y)
and the robot cannot get to it". The Deep3R hypothesis carries a z, so those
become different things to say. It is the clearest payoff of having the
reconstruction at all, and it is why `mecanumbot_seek` needs the server rather
than merely tolerating it.

## Why the two branches are a parallel and not a sequence

The user-facing requirement is that the robot *"stops along the way if it finds
the item"*. In a sequence that would mean a check for the object between every
pair of steps, and the check would only fire at the seams -- the robot would
drive right past the thing it is looking for and notice on arrival.

As a parallel, `WatchForObject` is doing nothing else for the whole episode.
Wherever the robot happens to be in the goal-directed branch -- halfway to the
remembered place, spinning at it, three rings out into the search -- a sighting
ends the parallel, `terminate()` cancels the nav2 goal that was in flight, and
the approach starts from wherever the robot is standing.

## Why `SuccessOnSelected` and not `SuccessOnOne`

With `SuccessOnOne` the parallel would also succeed if the *search* branch
succeeded, and the tree would fall into the approach with nothing sighted.
`SuccessOnSelected([watch])` makes the sighting the only thing that can end the
parallel successfully, which is exactly the claim being made: after this
parallel, the object has been seen. The search branch can only stay RUNNING or
fail, and its failure -- extinction -- fails the parallel and ends the episode.

`TickSeekingDrive` is the third child and always returns RUNNING, so the circuit
advances for as long as the episode lasts and neither ends it nor fails it.

## Where the modelled SEEKING sits

The circuit is not a decoration on this tree; three of its leaves are wired to
it and it changes what they do:

* `WatchForObject` and `TickSeekingDrive` take their detection threshold from
  `drive.detection_threshold()`, so a strongly seeking robot acts on a weaker
  detection -- incentive salience, and the feedback loop that makes a first
  glimpse easier to confirm;
* `SearchAround` takes its rings from `drive.search_radius()`, so falling
  expectancy widens the search;
* `SearchAround` also *ends* on `drive.extinguished` -- the robot gives up
  because it has stopped believing the object is findable, not because a counter
  ran out;
* `GraspObject` calls `drive.consummate()`, because SEEKING is appetitive and
  switches off on receipt.

The parameter loader has to be the first leaf in the tree. `setup()` runs
depth-first in tree order, and every other behaviour reads its constants -- and
the circuit itself -- off the blackboard during its own `setup()`.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ConfiguredTimer
from mecanumbot_movement_behaviours.targets import SUBJECT
from mecanumbot_movement_behaviours.turning import FindPeople, Spin360, TurnToward

from mecanumbot_seek import reachability
from mecanumbot_seek.behaviours.alerting import (
    AnnounceUnreachable,
    CheckObjectGraspable,
    CountAlternation,
    RecordUnreachable,
    SomeoneToTell,
)
from mecanumbot_seek.behaviours.approach import ApproachObject, GraspObject
from mecanumbot_seek.behaviours.blackboard_managers import (
    ClearSeekEpisode,
    SeekParamsToBlackboard,
)
from mecanumbot_seek.behaviours.drive import TickSeekingDrive
from mecanumbot_seek.behaviours.incentive import AcquireSeekTarget, WatchForObject
from mecanumbot_seek.behaviours.search import SearchAround
from mecanumbot_seek.defaults import TUNABLES
from mecanumbot_seek.keys import SEEK_KEYS, SeekApproach
from mecanumbot_seek.tree_nodes.tree_common import (
    build_params,
    resolve_yaml_path,
    run_tree,
)
from mecanumbot_movement_behaviours.targets import TARGET

TREE_NAME = "seek_tree"
DEFAULT_YAML_FILENAME = "seek_setting_constants.yaml"


class SeekTurnToward(TurnToward):
    """`TurnToward`, bound to this package's key spelling."""

    KEYS = SEEK_KEYS


class SeekFindPeople(FindPeople):
    """`FindPeople`, bound to this package's key spelling."""

    KEYS = SEEK_KEYS


class SeekScan(Spin360):
    """
    A full revolution on the spot, looking for an object rather than a person.

    `Spin360` already spins without acting on what it sees -- it is built for
    the recovery parallel, where a sibling branch is the one that reacts -- so
    all this does is bind the seek key spelling and lower the head. The sibling
    here is `WatchForObject`, which is watching throughout anyway.
    """

    KEYS = SEEK_KEYS

    def __init__(self, name="SeekScan", **kwargs):
        super().__init__(name, head=None, **kwargs)


def get_yaml_path():
    """Resolve the constants YAML the same way the launch file would."""
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_goal_directed_search():
    """
    Go where the object was, look around, then search outwards from there.

    A memory-guided sequence, and it is the branch that fails: if the object is
    not at the remembered place, the scan finds nothing and `SearchAround` runs
    until the drive extinguishes. It has no success path at all -- reaching the
    end of a search without finding anything is a failure of the episode, and
    finding something is the *other* branch's business.
    """
    search = py_trees.composites.Sequence(name="GoalDirectedSearch", memory=True)
    search.add_children(
        [
            SeekApproach(
                name="GoToWhereItWas",
                target_type=TARGET,
                mode="exact",
            ),
            SeekScan(name="LookAroundWhereItWas"),
            SearchAround(name="SearchOutwards"),
        ]
    )
    return search


def create_seek():
    """
    Watch for the object while searching for it, with the circuit running.

    Ends SUCCESS only when `WatchForObject` succeeds; ends FAILURE when the
    goal-directed branch gives up. See the module docstring for why the policy
    is `SuccessOnSelected` rather than `SuccessOnOne`.
    """
    watch = WatchForObject(name="WatchForObject")
    seek = py_trees.composites.Parallel(
        name="SeekUntilSighted",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[watch], synchronise=False
        ),
    )
    seek.add_children(
        [
            TickSeekingDrive(name="SeekingCircuit"),
            watch,
            create_goal_directed_search(),
        ]
    )
    return seek


def create_showing_gesture(alternations, turn_timeout):
    """
    Alternate orientation between the object and the person, `alternations` times.

    The showing gesture. Each lap is: face the object, hold, face the person,
    hold, and count one. Built out of the movement library's `TurnToward`
    because that is exactly what it is -- and because that behaviour picks the
    head pose from the target type on its own, so the neck lifts to address the
    person and drops to indicate the object without this file saying so.

    `alternations` and `turn_timeout` are pre-read from the constants file
    rather than taken off the blackboard, because `Repeat` wants its count while
    the tree is being built, which is before any parameter has been loaded. It
    is the same reason the tick period is read that way. `InPlaceTurn` does take
    its timeout as a number, and passing it here rather than letting it resolve
    `turn_timeout` is what keeps a showing turn shorter than a search turn.
    """
    lap = py_trees.composites.Sequence(name="ShowOnce", memory=True)
    lap.add_children(
        [
            SeekTurnToward(
                name="LookAtObject", target_type=TARGET, timeout=turn_timeout
            ),
            ConfiguredTimer("HoldOnObject", key="seek_show_hold", tunables=TUNABLES),
            SeekTurnToward(
                name="LookAtPerson", target_type=SUBJECT, timeout=turn_timeout
            ),
            ConfiguredTimer("HoldOnPerson", key="seek_show_hold", tunables=TUNABLES),
            CountAlternation(name="CountAlternation"),
        ]
    )
    return py_trees.decorators.Repeat(
        name="ShowingGesture", child=lap, num_success=max(1, int(alternations))
    )


def create_alert(alternations, turn_timeout):
    """
    Go and tell somebody about an object the robot found and cannot have.

    Finding an audience is a scan if nobody is already in view. Approaching them
    first matters: a showing gesture performed from across a room is a robot
    turning back and forth for no visible reason, and the whole point is that a
    person can see which way it is orienting.

    The announcement is the *last* step and runs whatever happened before it, so
    an alert is published even when nobody could be found -- "found it, could
    not have it, nobody to tell" is an outcome, and losing it because the room
    was empty would be losing the most interesting runs.
    """
    audience = py_trees.composites.Selector("FindSomeoneToTell", memory=True)
    audience.add_children(
        [
            SomeoneToTell(name="SomebodyIsHere"),
            SeekFindPeople(name="LookForSomebody"),
        ]
    )

    tell = py_trees.composites.Sequence(name="TellSomebody", memory=True)
    tell.add_children(
        [
            audience,
            SeekApproach(name="GoToThePerson", target_type=SUBJECT, mode="fixed_distance"),
            create_showing_gesture(alternations, turn_timeout),
        ]
    )

    alert = py_trees.composites.Sequence(name="AlertToUnreachable", memory=True)
    alert.add_children(
        [
            py_trees.decorators.FailureIsSuccess(name="TellIfAnyoneIsThere", child=tell),
            AnnounceUnreachable(name="AnnounceUnreachable"),
        ]
    )
    return alert


def create_secure_or_alert(alternations, turn_timeout):
    """
    Pick the object up, or go and tell somebody why the robot cannot.

    The branch point of the whole design. The robot is standing next to
    something it was sent for; either it can close its grabbers on it, or it
    knows something a person would want to know and should say so.

    `CheckObjectGraspable` is what decides, and the number it decides on is the
    object's **height** -- which is the one thing the 2D map cannot supply and
    the point cloud can. From the lidar alone, a mug on a table and a mug on the
    floor behind a chair are the same fact. With a z, they are different things
    to tell a person.
    """
    secure = py_trees.composites.Sequence(name="SecureObject", memory=True)
    secure.add_children(
        [
            CheckObjectGraspable(name="CheckObjectGraspable"),
            GraspObject(name="GraspObject"),
        ]
    )

    choice = py_trees.composites.Selector("SecureOrAlert", memory=True)
    choice.add_children([secure, create_alert(alternations, turn_timeout)])
    return choice


def create_approach():
    """
    Drive up to the sighted object, or record that the robot could not.

    A `Selector`, so a drive that fails becomes a *reason* rather than a failed
    episode: `RecordUnreachable` succeeds, and the tree carries on into
    `SecureOrAlert`, where the graspable check fails on distance and the alert
    runs. A robot that gets within sight of a thing and cannot reach it has
    learned something worth passing on.
    """
    approach = py_trees.composites.Selector("ApproachOrRecord", memory=True)
    approach.add_children(
        [
            ApproachObject(name="ApproachObject"),
            RecordUnreachable(name="CouldNotGetThere", reason=reachability.NO_ROUTE),
        ]
    )
    return approach


def create_episode(alternations, turn_timeout):
    """Build one full seeking episode, from being asked to holding the thing."""
    episode = py_trees.composites.Sequence(name="SeekEpisode", memory=True)
    episode.add_children(
        [
            AcquireSeekTarget(name="AcquireTarget"),
            create_seek(),
            create_approach(),
            create_secure_or_alert(alternations, turn_timeout),
            ClearSeekEpisode(name="EndEpisode", reason="episode complete"),
        ]
    )

    # Absorb a failed episode so the loop above never sees one. Whatever went
    # wrong -- nobody asked, the server never answered, the drive extinguished,
    # nav2 could not get there -- the answer is the same: let the object go and
    # wait to be asked for the next one.
    attempt = py_trees.composites.Selector("EpisodeOrRelease", memory=True)
    attempt.add_children(
        [
            episode,
            ClearSeekEpisode(name="AbandonEpisode", reason="episode failed"),
        ]
    )
    return attempt


def create_root(yaml_path=None):
    """Build the tree root: load the constants, then seek for ever."""
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # How many times the showing gesture alternates is settled while the tree is
    # being built, which is before `ParamsToBlackboard` has run -- so it is read
    # straight out of the file, the same way the tick period is.
    params = build_params(yaml_path)
    alternations = TUNABLES.file_constant(params, "seek_show_alternations")
    turn_timeout = float(TUNABLES.file_constant(params, "seek_show_turn_timeout"))

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            SeekParamsToBlackboard(name="LoadSeekParams", yaml_path=yaml_path),
            py_trees.decorators.Repeat(
                name="SeekLoop", child=create_episode(alternations, turn_timeout), num_success=-1
            ),
        ]
    )
    return root


def main(args=None):
    """Run the seek behaviour tree."""
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
