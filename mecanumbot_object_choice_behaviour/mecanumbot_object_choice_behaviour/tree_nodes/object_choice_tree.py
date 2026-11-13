"""
The object-choice tree: be addressed, read a pointing cue, go to the brick it names.

The dog object-choice paradigm on the robot. Coloured bricks lie on the floor,
a person stands among them facing the robot, catches its attention, and points
at one; the robot's choice is the brick it goes to. One pass through
`create_trial()` is one trial:

    1. wait for the experimenter's go (`/mecanumbot/object_choice/next`);
    2. head down: survey which bricks are on the floor and remember where;
    3. head up: wait for an attention bid, face the person, nod;
    4. read a pointing cue while keeping them in frame;
    5. choose the brick the cue indicates -- or none, which is recorded;
    6. respond (`oc_response_mode`):
         approach -- drive to it and stop facing it, holding there;
         grab     -- close the grabbers on it and bring it to the person;
    7. drive back to the start pose, and publish the trial's record.

Steps 3-4 are the ostensive tree's behaviours and step 6 is the fetch tree's,
unchanged; see `behaviours/ros_interfaces.py` for how the fetch behaviours are
shown only the chosen brick. Two departures from the ostensive tree, both
deliberate: there is no look-around when nobody bids for attention -- a robot
that turns away from the layout would survey it from somewhere else -- and the
head is surveyed down before it is lifted, because one tilt cannot see the floor
and a standing person's arms.

**What this changes in the experiment.** The ostensive tree drives a fixed
distance along the ray; here the ray only selects among objects, and a cue that
selects none ends the trial without movement. The fetch tree picks the nearest
ball; here the nearest brick is irrelevant. Both are the point of the paradigm.

Every trial ends in `FinishTrial`, completed or not, so every trial is recorded.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ConfiguredTimer
from mecanumbot_fetch_behaviour.behaviours.approach import ApproachBall
from mecanumbot_fetch_behaviour.behaviours.blackboard_managers import ClearFetchEpisode
from mecanumbot_fetch_behaviour.behaviours.centring import FaceBall, TrackBallWithHead
from mecanumbot_fetch_behaviour.defaults import TUNABLES as FETCH_TUNABLES
from mecanumbot_fetch_behaviour.tree_nodes.fetch_tree import (
    create_delivery,
    create_secure,
)
from mecanumbot_ostensive_behaviour.behaviours.acknowledge import NodAcknowledge
from mecanumbot_ostensive_behaviour.behaviours.attention import WaitForAttentionSignal
from mecanumbot_ostensive_behaviour.behaviours.focus import FaceTarget
from mecanumbot_ostensive_behaviour.tree_nodes.ostensive_tree import create_read_cue

from mecanumbot_object_choice_behaviour.behaviours.blackboard_managers import (
    ObjectChoiceParamsToBlackboard,
)
from mecanumbot_object_choice_behaviour.behaviours.trial import (
    ChooseBrick,
    FinishTrial,
    LookAtBrick,
    NoteTrial,
    OUTCOME_COMPLETED,
    RememberStartPose,
    ReturnToStart,
    StartTrial,
    SurveyBricks,
    TrialPhase,
    WaitForGo,
)
from mecanumbot_object_choice_behaviour.defaults import (
    RESPONSE_APPROACH,
    RESPONSE_GRAB,
    RESPONSE_MODES,
    TUNABLES,
)
from mecanumbot_object_choice_behaviour.tree_nodes.tree_common import (
    build_params,
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "object_choice_tree"
DEFAULT_YAML_FILENAME = "object_choice_setting_constants.yaml"

# The fetch defaults as this tree sees them, for the build-time reads.
BUILD_TUNABLES = FETCH_TUNABLES.merged_with(TUNABLES.defaults)


def get_yaml_path():
    """Resolve the constants YAML the same way the launch file would."""
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_approach_response():
    """
    Drive to the chosen brick and stop there facing it: the classic response.

    The fetch tree's close-in without the creep or the grab: the approach parks
    `fetch_approach_stop` from the brick, the centring turn faces it, and the
    head tracks it all the way. Then the robot holds still for
    `oc_choice_hold`, so the choice is unmistakable on video.
    """
    close_in = py_trees.composites.Sequence(name="DriveAndFace", memory=True)
    close_in.add_children(
        [ApproachBall(name="ApproachBrick"), FaceBall(name="FaceBrick")]
    )
    watching = py_trees.composites.Parallel(
        name="CloseInOnBrick",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[close_in], synchronise=False
        ),
    )
    watching.add_children([TrackBallWithHead(name="TrackBrickWithHead"), close_in])

    response = py_trees.composites.Sequence(name="ApproachChosenBrick", memory=True)
    response.add_children(
        [
            TrialPhase("SayApproaching", "approaching"),
            LookAtBrick(name="LookAtBrick"),
            watching,
            NoteTrial("ArrivedAtChoice", "responded"),
            TrialPhase("SayAtChoice", "at choice"),
            ConfiguredTimer("HoldAtChoice", key="oc_choice_hold", tunables=TUNABLES),
        ]
    )
    return response


def create_grab_response(attempts, turn_timeout):
    """
    Grip the chosen brick and bring it to the person: the fetch tree's two phases.

    `create_secure` and `create_delivery` are the fetch tree's own builders,
    used as they are; the brick is the only ball they can see.
    """
    response = py_trees.composites.Sequence(name="FetchChosenBrick", memory=True)
    response.add_children(
        [
            LookAtBrick(name="LookAtBrick"),
            create_secure(attempts),
            NoteTrial("GrippedChoice", "gripped"),
            create_delivery(turn_timeout),
            NoteTrial("DeliveredChoice", "responded"),
        ]
    )
    return response


def create_response(mode, attempts, turn_timeout):
    """Build the response subtree `oc_response_mode` names."""
    if mode == RESPONSE_APPROACH:
        return create_approach_response()
    if mode == RESPONSE_GRAB:
        return create_grab_response(attempts, turn_timeout)
    raise ValueError(f"oc_response_mode '{mode}', expected one of {RESPONSE_MODES}")


def create_read_person():
    """
    Be addressed, face the addressee, nod, and read their cue.

    The ostensive exchange up to the cue, minus its look-around: when nobody
    bids within `attention_timeout` the trial ends as `not_addressed`.
    """
    exchange = py_trees.composites.Sequence(name="ReadThePerson", memory=True)
    exchange.add_children(
        [
            TrialPhase("SayWaiting", "waiting for attention"),
            WaitForAttentionSignal(name="WatchForBid"),
            FaceTarget(name="FaceAddressee"),
            NodAcknowledge(name="AcknowledgeAddressee"),
            TrialPhase("SayReading", "reading cue"),
            create_read_cue(),
        ]
    )
    return exchange


def create_trial(mode, attempts, turn_timeout):
    """Build one trial, from the go to the record, with its failure path."""
    trial = py_trees.composites.Sequence(name="ObjectChoiceTrial", memory=True)
    trial.add_children(
        [
            TrialPhase("SayReady", "ready"),
            WaitForGo(name="WaitForGo"),
            StartTrial(name="StartTrial"),
            TrialPhase("SaySurveying", "surveying"),
            SurveyBricks(name="SurveyBricks"),
            create_read_person(),
            ChooseBrick(name="ChooseBrick"),
            create_response(mode, attempts, turn_timeout),
            TrialPhase("SayReturning", "returning"),
            ReturnToStart(name="ReturnToStart"),
            FinishTrial(name="RecordTrial", outcome=OUTCOME_COMPLETED),
            ClearFetchEpisode(name="ForgetTheBrick", reason="trial completed"),
        ]
    )

    # Whatever stopped the trial, record it, let go, and go back to the start.
    abandon = py_trees.composites.Sequence(name="AbandonTrial", memory=True)
    abandon.add_children(
        [
            FinishTrial(name="RecordUnfinishedTrial"),
            ClearFetchEpisode(name="ForgetTheBrickAnyway", reason="trial ended early"),
            TrialPhase("SayReturningAnyway", "returning"),
            py_trees.decorators.FailureIsSuccess(
                name="ReturnIfWeCan", child=ReturnToStart(name="ReturnAfterFailure")
            ),
        ]
    )

    attempt = py_trees.composites.Selector("TrialOrAbandon", memory=True)
    attempt.add_children([trial, abandon])
    return attempt


def create_root(yaml_path=None):
    """Build the tree root: load the constants, note the start, run trials for ever."""
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # Structure, read before any blackboard exists: which response subtree,
    # how many grabs, how long the turn onto the person may take.
    params = build_params(yaml_path)
    mode = str(BUILD_TUNABLES.file_constant(params, "oc_response_mode"))
    attempts = int(BUILD_TUNABLES.file_constant(params, "fetch_grasp_attempts"))
    turn_timeout = float(BUILD_TUNABLES.file_constant(params, "fetch_offer_turn_timeout"))

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ObjectChoiceParamsToBlackboard(
                name="LoadObjectChoiceParams", yaml_path=yaml_path
            ),
            RememberStartPose(name="RememberStartPose"),
            py_trees.decorators.Repeat(
                name="TrialLoop",
                child=create_trial(mode, attempts, turn_timeout),
                num_success=-1,
            ),
        ]
    )
    return root


def main(args=None):
    """Run the object-choice behaviour tree."""
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
