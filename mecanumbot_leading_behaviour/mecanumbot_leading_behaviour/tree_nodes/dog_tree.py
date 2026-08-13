"""Dog-inspired leading behaviour.

The robot leads a human from the start of the route to the target the way a dog
leads a person: it walks a stretch, looks back to check it is still being
followed, waits for them or goes back and fetches them when it is not, and
points the target out once they are close to it.

What the loop is built around is that **walking and checking are different
things, and the walking is most of it**. A dog does not stop at every step to
look round, and it does not ignore its person for the whole walk either.

* **Leading is a leg, not a step.** `FollowRoute` hands nav2 several route
  checkpoints in one `NavigateThroughPoses` goal and lets it drive through them
  without stopping; the waypoints are oriented along the route, so the robot is
  already pointing the right way as it passes each one. The turn that sets a leg
  off is only made when the route really bends (`route_turn_min`) -- rotating on
  the spot for a few degrees is a stop for nothing.
* **A look back is paced, not automatic.** `DogCheckInDue` asks
  `pacing.check_in_due()`: every couple of checkpoints, every so many seconds,
  and immediately when the human goes quiet or drops behind. Between those, the
  robot just leads.
* **The look back is a slow turn onto the human's last known place**
  (`GlanceBack`), not a search spin: slow enough to read as looking, and slow
  enough for the detector to find somebody the robot sweeps past. Only when that
  turn and its sweep find nobody is the human treated as lost, and only then
  does the recovery patrol start.
* **Not being followed is not the same as being lost.** Found but trailing: the
  robot waits a moment for them (`DogWaitForCatchUp`), and if they still do not
  come, walks back, faces them, wiggles and asks for their attention again
  before carrying on. Not found at all: the patrol.

Two habits from the older tree carry the gestures:

* Looking back is a rotation with a direction, and the handedness is remembered
  (`search_spin_sign`), so the attention wiggle flows out of the glance and the
  turn that points the target out unwinds it.
* The head is lifted whenever the robot is dealing with the human and levelled
  again while driving; the head pose comes from the target type, so the
  behaviours pick it up on their own.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    ConfiguredTimer,
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.constants import file_constant
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import (
    DogBehaviourSequence,
    DogCheckFollowing,
    DogCheckInDue,
    DogResumeLeading,
    DogWaitForCatchUp,
)
from mecanumbot_leading_behaviour.behaviours.movement_managers import (
    Approach,
    CheckRobotHasBall,
    CheckSubjectTargetSuccess,
    FollowRoute,
    GlanceBack,
    RelativeTurnPattern,
    TurnToward,
)
from mecanumbot_leading_behaviour.behaviours.targets import CHECKPOINT, SUBJECT, TARGET
from mecanumbot_leading_behaviour.behaviours.turning import SHORTEST
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
    build_params,
    create_recover_lost_sequence,
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "dog_tree"
DEFAULT_YAML_FILENAME = "Eto_behaviour_setting_constants.yaml"


def get_yaml_path():
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_seek_attention(ID):
    """Walk up to the human, face them, wiggle and ask for their attention."""
    sequence = py_trees.composites.Sequence(name=ID + "SeekAttention", memory=True)
    sequence.add_children(
        [
            Approach(
                name=ID + "ApproachSubject", target_type=SUBJECT, mode="fixed_distance"
            ),
            TurnToward(name=ID + "TurnTowardSubject", target_type=SUBJECT),
            RelativeTurnPattern(name=ID + "AttentionTurnPattern"),
            DogBehaviourSequence(ID + "CatchAttention", "catch_attention"),
        ]
    )
    return sequence


def create_check_in(ID="Lead"):
    """Look back for the human, and deal with what the look back found.

    The whole branch is skipped while no check-in is due, which is what keeps
    the robot walking instead of turning round every checkpoint. When one is
    due, the look back decides between three answers: following (carry on),
    trailing (wait, then fetch them), or gone -- which is a FAILURE and the only
    thing that starts the recovery patrol.
    """
    # Waiting did not bring them: walk back, face them and ask again. Resuming
    # picks the checkpoint to lead on from, because the pair have moved.
    fetch_subject = py_trees.composites.Sequence(name=ID + "FetchSubjectSeq", memory=True)
    fetch_subject.add_children(
        [
            create_seek_attention(ID=ID + "Fetch"),
            DogResumeLeading(name=ID + "ResumeAfterFetch"),
        ]
    )

    # Trailing: give them a moment first -- most of the time that is all it
    # takes, and walking back at them for a metre and a half reads as fussing.
    regain_subject = py_trees.composites.Selector(
        name=ID + "WaitOrFetchSelector", memory=True
    )
    regain_subject.add_children(
        [
            DogWaitForCatchUp(name=ID + "WaitForCatchUp"),
            fetch_subject,
        ]
    )

    still_following = py_trees.composites.Selector(
        name=ID + "FollowingOrRegainSelector", memory=True
    )
    still_following.add_children(
        [
            DogCheckFollowing(name=ID + "DogCheckFollowing"),
            regain_subject,
        ]
    )

    check_in = py_trees.composites.Sequence(name=ID + "CheckInSeq", memory=True)
    check_in.add_children(
        [
            GlanceBack(name=ID + "GlanceBackForSubject"),
            still_following,
        ]
    )

    when_due = py_trees.composites.Selector(name=ID + "CheckInIfDueSelector", memory=True)
    when_due.add_children(
        [
            py_trees.decorators.Inverter(
                name=ID + "NoCheckInDue", child=DogCheckInDue(name=ID + "DogCheckInDue")
            ),
            check_in,
        ]
    )
    return when_due


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()
    # Decorator counts are fixed when the tree is built, so this one comes
    # straight from the file; everything else reads the blackboard.
    params = build_params(yaml_path)

    # --- getting the human's attention in the first place -------------------
    # Try it directly; if there is nobody to approach, search the route first
    # and then run the same seek sequence.
    recover_then_seek = py_trees.composites.Sequence(
        name="RecoverThenSeek", memory=True
    )
    recover_then_seek.add_children(
        [
            create_recover_lost_sequence(ID="Init"),
            create_seek_attention(ID="Recov"),
        ]
    )

    seek_or_find = py_trees.composites.Selector("SeekOrFind", memory=True)
    seek_or_find.add_children(
        [
            create_seek_attention(ID="Init"),
            recover_then_seek,
        ]
    )

    # --- the human handed the ball over: find them and say thank you --------
    ball_reaction = py_trees.composites.Sequence(name="BallReactionSeq", memory=True)
    ball_reaction.add_children(
        [
            CheckRobotHasBall(name="CheckIfBallGiven"),
            create_recover_lost_sequence(ID="Ball"),
            Approach(
                name="ApproachSubjectBall", target_type=SUBJECT, mode="fixed_distance"
            ),
            DogBehaviourSequence("DogThank", "thank"),
            ConfiguredTimer(name="ThankDelayTimer", key="thank_delay"),
        ]
    )

    # --- the human is at the target: point it out ---------------------------
    show_target = py_trees.composites.Sequence(
        name="ShowWhileSubjectCloseSeq", memory=True
    )
    show_target.add_children(
        [
            CheckSubjectTargetSuccess(name="CheckSubjectNearTarget"),
            DogBehaviourSequence("DogCatchAttentionShow", "catch_attention"),
            ConfiguredTimer(name="TurnDelayTimer", key="show_turn_delay"),
            TurnToward(name="TurnTowardTargetShow", target_type=TARGET),
            DogBehaviourSequence("DogShowTarget", "indicate_target"),
        ]
    )

    # --- one leg of leading -------------------------------------------------
    # Face the way the route goes -- but only if it really goes another way,
    # `route_turn_min` -- and then drive the leg. The turn takes the shortest
    # way round rather than the `TurnToward` route-target default of unwinding
    # the glance: a checkpoint is somewhere to get to, not a gesture, and
    # unwinding it only made the robot swing the long way before setting off.
    lead_leg = py_trees.composites.Sequence(name="LeadLegSequence", memory=True)
    lead_leg.add_children(
        [
            TurnToward(
                name="TurnTowardCheckpointStep",
                target_type=CHECKPOINT,
                direction=SHORTEST,
                min_rotation_key="route_turn_min",
            ),
            FollowRoute(name="FollowRouteLeg"),
        ]
    )

    # --- one cycle of the loop: check in if it is time, then lead on ---------
    show_or_lead_step = py_trees.composites.Selector(
        "ShowOrLeadStepSelector", memory=True
    )
    show_or_lead_step.add_children([show_target, lead_leg])

    lead_cycle = py_trees.composites.Sequence(name="LeadCycleSequence", memory=True)
    lead_cycle.add_children(
        [
            create_check_in(ID="Lead"),
            show_or_lead_step,
        ]
    )

    # --- the human dropped out of a lead cycle ------------------------------
    # Patrol the route until somebody turns up, walk up to them and ask for
    # their attention again, then work out where leading has to carry on from.
    # The recovery parallel succeeds on its first tick when a person is already
    # visible, so this is only reached once the look back has properly failed to
    # find them -- by then they really are gone, not merely behind the robot.
    recover_and_resume = py_trees.composites.Sequence(
        name="RecoverAndResumeSeq", memory=True
    )
    recover_and_resume.add_children(
        [
            create_recover_lost_sequence(ID="Patrol"),
            create_seek_attention(ID="Regain"),
            DogResumeLeading(name="ResumeLeadingCheckpoint"),
        ]
    )

    # The human is most likely to slip out of view again during the walk up to
    # them, so a failed re-engagement goes back to patrolling rather than
    # unwinding the whole tree.
    lead_or_recover = py_trees.composites.Selector("LeadOrRecoverSelector", memory=True)
    lead_or_recover.add_children(
        [
            lead_cycle,
            py_trees.decorators.Retry(
                name="RecoverAndResumeRetry",
                child=recover_and_resume,
                num_failures=int(file_constant(params, "recover_retries")),
            ),
        ]
    )

    # The ball check is cheap and needs no look back of its own, so it stays
    # ahead of the cycle.
    show_or_lead = py_trees.composites.Selector("ShowOrLeadSelector", memory=True)
    show_or_lead.add_children(
        [
            py_trees.decorators.Repeat(
                name="BallReactionRepeat", child=ball_reaction, num_success=-1
            ),
            lead_or_recover,
        ]
    )

    # Losing the human is handled inside the loop, so the root only fails when
    # even the patrol turns up nobody; it is then ticked again from the top.
    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(name="LoadConstantParams", yaml_path=yaml_path),
            seek_or_find,
            py_trees.decorators.Repeat("ShowOrLeadLoop", show_or_lead, num_success=-1),
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
