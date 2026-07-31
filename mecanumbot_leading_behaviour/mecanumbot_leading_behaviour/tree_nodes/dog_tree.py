"""Dog-inspired leading behaviour.

The robot leads a human along a route of checkpoints the way a dog leads a
person: it walks a stretch, glances back to check it is still being followed,
catches the human's attention if it has to, and points out the target once they
are close to it.

Three habits keep that reading natural:

* Looking back is a rotation with a direction. The glance turns towards where
  the human was last seen and the attention wiggle carries that handedness on,
  so the gestures flow into one another instead of reversing abruptly. Getting
  back to the route is not a gesture, though: the turn to the next checkpoint
  asks for `direction="shortest"` and takes the smaller angle, rather than
  unwinding the glance the long way round the way route targets do by default.
* The head is lifted whenever the robot is dealing with the human (which also
  gives the pose detector a full-body view) and levelled again while driving.
* There is one glance back per cycle of the loop, no more. Looking for the
  human only scans when nobody is in view, but doing it once for the closeness
  check and again for the lead step could still leave the robot sweeping twice
  over the same glance, which reads as dithering rather than checking.

The head pose comes from the target type, so `TurnToward` picks it up on its
own; only the checkpoint turn's direction is spelled out at the call site.

Losing the human is part of leading rather than an error: a lead step that fails
drops into a patrol of the route -- scan a full circle, walk to the next
checkpoint, scan again -- which stops the moment somebody is seen. The robot then
walks up to them, asks for their attention again, and `DogResumeLeading` decides
which checkpoint leading carries on from, so the pair is not sent back over a
stretch the human already walked.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.dog_behaviours import (
    DogBehaviourSequence,
    DogCheckFollowing,
    DogResumeLeading,
)
from mecanumbot_leading_behaviour.behaviours.movement_managers import (
    Approach,
    CheckRobotHasBall,
    CheckSubjectTargetSuccess,
    FindPeople,
    RelativeTurnPattern,
    TurnToward,
)
from mecanumbot_leading_behaviour.behaviours.targets import CHECKPOINT, SUBJECT, TARGET
from mecanumbot_leading_behaviour.behaviours.turning import SHORTEST
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
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
            RelativeTurnPattern(name=ID + "AttentionTurnPattern", step_angle_deg=15.0),
            DogBehaviourSequence(ID + "CatchAttention", "catch_attention"),
        ]
    )
    return sequence


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

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
            py_trees.timers.Timer(name="ThankDelayTimer", duration=1),
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
            py_trees.timers.Timer(name="TurnDelayTimer", duration=2),
            TurnToward(name="TurnTowardTargetShow", target_type=TARGET),
            DogBehaviourSequence("DogShowTarget", "indicate_target"),
        ]
    )

    # --- one stretch of leading ---------------------------------------------
    # Check the human kept up, then face the next checkpoint and drive there.
    # That turn takes the shortest way round rather than the `TurnToward`
    # route-target default of unwinding the glance: a checkpoint is somewhere to
    # get to, not a gesture, and unwinding it only made the robot swing the long
    # way before setting off.
    lead_step = py_trees.composites.Sequence(name="LeadStepSequence", memory=True)
    lead_step.add_children(
        [
            DogCheckFollowing(name="DogCheckFollowing"),
            TurnToward(
                name="TurnTowardCheckpointStep",
                target_type=CHECKPOINT,
                direction=SHORTEST,
            ),
            Approach(name="ApproachCheckpoint", target_type=CHECKPOINT),
        ]
    )

    # --- one cycle of the loop: glance back once, then act on what it found --
    # Both the "are they at the target yet" check and the lead step need to see
    # the human, and each used to open with its own `FindPeople`. That scans
    # only when nobody is in view, but on a cycle where the closeness check
    # failed the robot could still end up sweeping twice for the same glance.
    # One glance now feeds both, and when it finds nobody the cycle fails
    # straight through to recovery instead of scanning again first.
    show_or_lead_step = py_trees.composites.Selector(
        "ShowOrLeadStepSelector", memory=True
    )
    show_or_lead_step.add_children([show_target, lead_step])

    glance_then_act = py_trees.composites.Sequence(
        name="GlanceThenActSequence", memory=True
    )
    glance_then_act.add_children(
        [
            FindPeople(name="GlanceBackForPerson"),
            show_or_lead_step,
        ]
    )

    # --- the human dropped out of a lead cycle ------------------------------
    # Patrol the route until somebody turns up, walk up to them and ask for
    # their attention again, then work out where leading has to carry on from.
    # The recovery parallel succeeds on its first tick when a person is already
    # visible, so a human who merely lagged behind is simply fetched, and only a
    # real disappearance turns into a checkpoint patrol.
    # The two halves are tagged for what they do -- patrol the route, then win
    # the human back -- rather than for the lead step they were reached from.
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
            glance_then_act,
            py_trees.decorators.Retry(
                name="RecoverAndResumeRetry", child=recover_and_resume, num_failures=3
            ),
        ]
    )

    # The ball check is cheap and needs no glance of its own, so it stays ahead
    # of the cycle; showing the target and leading now share one, inside it.
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
