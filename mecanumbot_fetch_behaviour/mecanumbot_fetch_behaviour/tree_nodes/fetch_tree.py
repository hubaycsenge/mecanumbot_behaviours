"""
The fetch behaviour tree: find a ball, get it, and take it to somebody.

One pass through `create_episode()` is one round of fetch, and the tree is that
round repeated for ever:

    1. the robot looks for a ball -- two branches at once:
         a. it watches for one, everywhere, the whole time;
         b. it turns full circles on the spot (`fetch_search_strategy: spin`,
            the default) or drives widening circles around where it started
            (`circles`), sweeping its head up and down either way;
    2. the moment (a) succeeds, (b) is abandoned mid-drive;
    3. it drives up to the ball, checks the ball is on the floor and not on a
       table or in a hand, and closes the grabbers -- retried as a whole,
       because the usual way a grab fails is that the ball rolls off the
       shafts as they close;
    4. it looks for the first person it can see, drives to them, faces them,
       and opens the grabbers;
    5. it backs off, waits, and starts looking again.

## Why the search is a parallel and not a sequence

The same argument `mecanumbot_seek` makes. In a sequence, "have we seen the
ball?" would be a check between search steps, and the check would only fire at
the seams -- the robot would drive right past a ball it could see and notice on
arrival. As a parallel, `WatchForBall` is doing nothing else for the whole
search, so wherever the robot happens to be, a sighting ends the parallel,
`terminate()` cancels the nav2 goal in flight, and the approach starts from
where the robot is standing.

`SuccessOnSelected([watch])` rather than `SuccessOnOne`, for the same reason
too: with `SuccessOnOne` the parallel would also succeed if the *search* branch
succeeded, and the tree would fall into the approach with nothing sighted. Here
the search branch cannot succeed at all -- it only ever runs or gives up -- so
the policy is a statement about what is true afterwards: a ball has been seen.

`SweepHead` is the third child and always returns RUNNING, so the head keeps
moving for as long as the search lasts and the sweep neither ends it nor fails
it.

## Why this tree does not model SEEKING

`mecanumbot_seek` runs a Panksepp SEEKING circuit, and everything about the
shape of this tree -- an appetitive search, a widening pattern, giving up --
looks like it should too. It deliberately does not, and the reason is
taxonomic rather than technical. Fetch with a person is **PLAY**: it is social,
it is reciprocal, it needs a partner, and in Panksepp's scheme it is a separate
primary-process system with its own neurochemistry and its own developmental
story. Driving it with a SEEKING circuit would make the thesis claim that
fetching a ball for somebody is the same motivation as foraging for an object,
which is exactly the kind of collapse the wiki records as one not to make.

So the search here is a plain pattern with a lap count, honestly a piece of
engineering, and the emotional model is left open. If a PLAY circuit is
modelled later it belongs in this package as `seeking.py`'s counterpart, and
`CircleSearch`'s lap limit and `WatchForBall`'s threshold are the two places it
would attach -- exactly where `SeekingDrive` attaches in the other tree.

The parameter loader has to be the first leaf in the tree. `setup()` runs
depth-first in tree order, and every other behaviour reads its constants off the
blackboard during its own `setup()`.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ConfiguredTimer
from mecanumbot_movement_behaviours.targets import SUBJECT

from mecanumbot_fetch_behaviour.behaviours.approach import (
    ApproachBall,
    CheckBallReachable,
    GraspBall,
)
from mecanumbot_fetch_behaviour.behaviours.blackboard_managers import (
    ClearFetchEpisode,
    FetchParamsToBlackboard,
)
from mecanumbot_fetch_behaviour.behaviours.delivery import (
    BackAway,
    ReleaseBall,
    SomeoneToGiveTo,
)
from mecanumbot_fetch_behaviour.behaviours.searching import (
    CircleSearch,
    SweepHead,
    WatchForBall,
)
from mecanumbot_fetch_behaviour.behaviours.signalling import AnnouncePhase
from mecanumbot_fetch_behaviour.defaults import TUNABLES
from mecanumbot_fetch_behaviour.keys import (
    FetchApproach,
    FetchFindPeople,
    FetchScan,
    FetchTurnToward,
)
from mecanumbot_fetch_behaviour.search_patterns import (
    SEARCH_CIRCLES,
    SEARCH_SPIN,
    SEARCH_STRATEGIES,
)
from mecanumbot_fetch_behaviour.tree_nodes.tree_common import (
    build_params,
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "fetch_tree"
DEFAULT_YAML_FILENAME = "fetch_setting_constants.yaml"


def get_yaml_path():
    """Resolve the constants YAML the same way the launch file would."""
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_body_search(strategy, laps):
    """
    Build the search branch that moves the robot: spin on the spot, or circles.

    Both only ever run or fail. `CircleSearch` is written that way; the spin is
    made so here, because `Repeat` succeeds after its last revolution and a
    search branch that succeeds would be re-ticked by the parallel and spin for
    ever -- `SuccessIsFailure` turns "went round `laps` times" into "gave up".
    A revolution that times out fails the `Repeat` and so the search, the same
    as `CircleSearch`'s backstop.
    """
    if strategy == SEARCH_CIRCLES:
        return CircleSearch(name="CircleSearch")
    if strategy != SEARCH_SPIN:
        raise ValueError(
            f"fetch_search_strategy '{strategy}', expected one of {SEARCH_STRATEGIES}"
        )
    return py_trees.decorators.SuccessIsFailure(
        name="GiveUpAfterSpinning",
        child=py_trees.decorators.Repeat(
            name="SpinOnTheSpot",
            child=FetchScan(name="FullCircle"),
            num_success=max(1, int(laps)),
        ),
    )


def create_search(strategy=SEARCH_SPIN, laps=3):
    """
    Look for a ball: watch for one, turn or circle for one, and sweep the head.

    Ends SUCCESS only when `WatchForBall` succeeds; ends FAILURE when the body
    search gives up after its laps. See the module docstring for why the
    policy is `SuccessOnSelected` rather than `SuccessOnOne`.
    """
    watch = WatchForBall(name="WatchForBall")
    search = py_trees.composites.Parallel(
        name="SearchUntilSighted",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[watch], synchronise=False
        ),
    )
    search.add_children(
        [
            SweepHead(name="SweepHead"),
            watch,
            create_body_search(strategy, laps),
        ]
    )

    phase = py_trees.composites.Sequence(name="FindBall", memory=True)
    phase.add_children([AnnouncePhase("SaySearching", "searching"), search])
    return phase


def create_secure(attempts):
    """
    Drive up to the ball, check it is on the floor, and close the grabbers.

    Retried as a whole rather than at the grab, and that is the point of the
    decorator being here. The usual way a grab fails is that the ball rolls off
    the shafts as they close, which leaves the robot half a metre from a ball
    that has moved -- so what has to be repeated is the *approach* as well, with
    a fresh detection behind it.

    `attempts` is pre-read from the constants file rather than taken off the
    blackboard, because `Retry` wants its count while the tree is being built,
    which is before any parameter has been loaded. Same reason as the tick
    period.
    """
    attempt = py_trees.composites.Sequence(name="ApproachAndGrab", memory=True)
    attempt.add_children(
        [
            ApproachBall(name="ApproachBall"),
            CheckBallReachable(name="CheckBallReachable"),
            GraspBall(name="GraspBall"),
        ]
    )

    phase = py_trees.composites.Sequence(name="SecureBall", memory=True)
    phase.add_children(
        [
            AnnouncePhase("SayApproaching", "approaching"),
            py_trees.decorators.Retry(
                name="RetryTheGrab", child=attempt, num_failures=max(1, int(attempts))
            ),
        ]
    )
    return phase


def create_delivery(turn_timeout):
    """
    Find the first person around, take them the ball, and give it to them.

    Approaching before releasing matters: a ball let go of across the room is a
    ball dropped, and the whole content of the act is that it was brought. The
    turn onto the person is a separate step from the drive for the same reason
    the seek tree's showing gesture is -- nav2 parks the robot at a distance and
    is under no obligation to leave it facing anybody.

    `FetchTurnToward(SUBJECT)` lifts the head on its own, because the movement
    library picks the head pose from the target type; that is the neck coming up
    off the floor it has been watching since the approach, which is the moment
    the robot stops being a machine collecting an object.
    """
    audience = py_trees.composites.Selector("FindSomeone", memory=True)
    audience.add_children(
        [
            SomeoneToGiveTo(name="SomebodyIsHere"),
            FetchFindPeople(name="LookForSomebody"),
        ]
    )

    handover = py_trees.composites.Sequence(name="HandOver", memory=True)
    handover.add_children(
        [
            AnnouncePhase("SayDelivering", "delivering"),
            audience,
            FetchApproach(
                name="GoToThePerson", target_type=SUBJECT, mode="fixed_distance"
            ),
            FetchTurnToward(
                name="FaceThePerson", target_type=SUBJECT, timeout=turn_timeout
            ),
            ConfiguredTimer("OfferPause", key="fetch_offer_hold", tunables=TUNABLES),
            AnnouncePhase("SayHandover", "handover"),
            ReleaseBall(name="ReleaseBall"),
            ConfiguredTimer(
                "LetThemTakeIt", key="fetch_release_hold", tunables=TUNABLES
            ),
            # Failing to step back does not undo a handover, so the withdraw is
            # allowed to fail without failing the delivery.
            py_trees.decorators.FailureIsSuccess(
                name="BackOffIfWeCan", child=BackAway(name="BackAway")
            ),
        ]
    )
    return handover


def create_episode(attempts, turn_timeout, strategy=SEARCH_SPIN, laps=3):
    """Build one full round of fetch, from looking to letting go."""
    episode = py_trees.composites.Sequence(name="FetchEpisode", memory=True)
    episode.add_children(
        [
            create_search(strategy, laps),
            create_secure(attempts),
            create_delivery(turn_timeout),
            ClearFetchEpisode(name="EndEpisode", reason="ball delivered"),
            ConfiguredTimer("RestBeforeTheNext", key="fetch_rest", tunables=TUNABLES),
        ]
    )

    # Absorb a failed round so the loop above never sees one. Whatever went
    # wrong -- no ball found, the ball was on a table, three grabs missed,
    # nobody to give it to -- the answer is the same: let this ball go and start
    # looking again. A game does not end because a throw went badly.
    attempt = py_trees.composites.Selector("EpisodeOrRelease", memory=True)
    attempt.add_children(
        [
            episode,
            py_trees.composites.Sequence(
                name="AbandonEpisode",
                memory=True,
                children=[
                    AnnouncePhase("SayGivingUp", "abandoned"),
                    ClearFetchEpisode(name="ForgetThisBall", reason="round failed"),
                    ConfiguredTimer(
                        "RestAfterFailure", key="fetch_rest", tunables=TUNABLES
                    ),
                ],
            ),
        ]
    )
    return attempt


def create_root(yaml_path=None):
    """Build the tree root: load the constants, then play fetch for ever."""
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # How many grabs are attempted and how long a turn may take are settled
    # while the tree is being built, which is before `ParamsToBlackboard` has
    # run -- so they are read straight out of the file, the same way the tick
    # period is.
    params = build_params(yaml_path)
    attempts = TUNABLES.file_constant(params, "fetch_grasp_attempts")
    turn_timeout = float(TUNABLES.file_constant(params, "fetch_offer_turn_timeout"))
    # The search's shape is structure too: which subtree the body search is.
    strategy = str(TUNABLES.file_constant(params, "fetch_search_strategy"))
    laps = TUNABLES.file_constant(params, "fetch_search_laps")

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            FetchParamsToBlackboard(name="LoadFetchParams", yaml_path=yaml_path),
            py_trees.decorators.Repeat(
                name="FetchLoop",
                child=create_episode(attempts, turn_timeout, strategy, laps),
                num_success=-1,
            ),
        ]
    )
    return root


def main(args=None):
    """Run the fetch behaviour tree."""
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
