"""
The ostensive behaviour tree: be addressed, acknowledge, be directed, go.

One pass through `create_exchange()` is one complete ostensive exchange, and the
tree is that exchange repeated for ever:

    1. somebody signals for attention and becomes the addressee;
    2. the robot turns to face them;
    3. it nods, so they can see their bid landed;
    4. it holds them in frame while it reads a direction cue off their arms;
    5. it drives where they pointed;
    6. it releases them, and waits to be addressed again.

Steps 2 and 4 both turn on where the person appears in the image, step 5 goes
through nav2, and nothing anywhere drives the wheels for navigation directly --
the same division of labour as the leading trees.

**Why the loop cannot simply retry.** Step 1 waits with a timeout and then gives
up so the robot can turn round and look elsewhere: standing still, staring at an
empty corridor, is not a state this robot should be able to get stuck in. Every
later step fails when the addressee walks off. Both kinds of failure land in the
same place -- `AbandonExchange` drops the lock and the next exchange starts from
scratch -- because an exchange resumed halfway through, against a person who has
already left, is worse than one begun again.

The parameter loader has to be the first leaf in the tree. `setup()` runs
depth-first in tree order, and every other behaviour reads its constants off the
blackboard during its own `setup()`.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.turning import FindPeople

from mecanumbot_ostensive_behaviour.behaviours.acknowledge import NodAcknowledge
from mecanumbot_ostensive_behaviour.behaviours.attention import WaitForAttentionSignal
from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    ClearTargetLock,
    OstensiveParamsToBlackboard,
)
from mecanumbot_ostensive_behaviour.behaviours.focus import (
    FaceTarget,
    KeepTargetInFocus,
)
from mecanumbot_ostensive_behaviour.behaviours.pointing import (
    DecodeDirectionCue,
    FollowDirectionCue,
)
from mecanumbot_ostensive_behaviour.behaviours.ros_interfaces import HEAD_SEEK
from mecanumbot_ostensive_behaviour.tree_nodes.tree_common import (
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "ostensive_tree"
DEFAULT_YAML_FILENAME = "ostensive_setting_constants.yaml"


def get_yaml_path():
    """Resolve the constants YAML the same way the launch file would."""
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_be_addressed():
    """
    Wait to be addressed, and look around once if nobody is there to address us.

    The scan is not a search for a particular person -- there is no addressee yet
    -- so it stops at the first person it sees and hands back to the watch, which
    is the behaviour that decides whether they actually want anything.
    """
    scan_then_watch = py_trees.composites.Sequence(name="ScanThenWatch", memory=True)
    scan_then_watch.add_children(
        [
            FindPeople(name="LookAround", head=HEAD_SEEK),
            WaitForAttentionSignal(name="WatchForBidAfterScan"),
        ]
    )

    be_addressed = py_trees.composites.Selector("BeAddressed", memory=True)
    be_addressed.add_children(
        [
            WaitForAttentionSignal(name="WatchForBid"),
            scan_then_watch,
        ]
    )
    return be_addressed


def create_read_cue():
    """
    Read a direction cue while holding the addressee in frame.

    The parallel ends the moment the cue decoder succeeds. `KeepTargetInFocus`
    never succeeds on its own -- it is the branch that keeps the person inside
    the image the cue is being read from, and the branch that fails when they
    leave it.
    """
    read_cue = py_trees.composites.Parallel(
        name="ReadCueWhileFocused",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    read_cue.add_children(
        [
            KeepTargetInFocus(name="HoldAddresseeInFrame"),
            DecodeDirectionCue(name="ReadDirectionCue"),
        ]
    )
    return read_cue


def create_exchange():
    """Build one full ostensive exchange, from being addressed to arriving."""
    exchange = py_trees.composites.Sequence(name="OstensiveExchange", memory=True)
    exchange.add_children(
        [
            create_be_addressed(),
            FaceTarget(name="FaceAddressee"),
            NodAcknowledge(name="AcknowledgeAddressee"),
            create_read_cue(),
            FollowDirectionCue(name="GoWherePointed"),
            ClearTargetLock(name="EndExchange", reason="exchange completed"),
        ]
    )

    # Absorb a failed exchange so the loop above never sees one: whatever went
    # wrong, the answer is the same -- let the addressee go and start over.
    attempt = py_trees.composites.Selector("ExchangeOrRelease", memory=True)
    attempt.add_children(
        [
            exchange,
            ClearTargetLock(name="AbandonExchange", reason="exchange failed"),
        ]
    )
    return attempt


def create_root(yaml_path=None):
    """Build the tree root: load the constants, then exchange for ever."""
    if yaml_path is None:
        yaml_path = get_yaml_path()

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            OstensiveParamsToBlackboard(
                name="LoadOstensiveParams", yaml_path=yaml_path
            ),
            py_trees.decorators.Repeat(
                name="ExchangeLoop", child=create_exchange(), num_success=-1
            ),
        ]
    )
    return root


def main(args=None):
    """Run the ostensive behaviour tree."""
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
