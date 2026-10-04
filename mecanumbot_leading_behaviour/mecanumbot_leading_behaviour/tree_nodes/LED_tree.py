"""
LED leading behaviour: same route, signalled with light patterns.

This is the non-animal comparison condition, so the neck is left alone
(`head=None` on every turn): the camera keeps the lifted tilt the parameter
loader sets at startup, but the robot never gestures with its head.

The lights are on for the whole drive, not only before and after it: they flow
the way the robot is going and fill with a second colour as it gets there
(`LEDLeadSignal`). The signals meant for the person -- asking for attention,
thanking -- are shown on the half of the robot that faces them.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import (
    LEAD_KEYS,
    LEDBehaviourSequence,
    LEDLeadSignal,
)
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    LED_SCRIPTS,
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.route_behaviours import (
    Approach,
    CheckRobotAtLastCheckpoint,
    CheckRobotHasBall,
    CheckSubjectTargetSuccess,
    FindPeople,
    TurnToward,
)
from mecanumbot_movement_behaviours.targets import (
    LAST_CHECKPOINT,
    SUBJECT,
    TARGET,
)
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
    create_recover_lost_sequence,
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "LED_tree"
DEFAULT_YAML_FILENAME = "Eto_behaviour_setting_constants.yaml"


def get_yaml_path():
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # --- reach the human: directly, or search the route for them first ------
    recover_then_approach = py_trees.composites.Sequence(
        name="RecoverThenApproach", memory=True
    )
    recover_then_approach.add_children(
        [
            create_recover_lost_sequence(ID="Init"),
            Approach(
                name="ApproachSubjectRecov", target_type=SUBJECT, mode="fixed_distance"
            ),
        ]
    )

    seek_or_find = py_trees.composites.Selector("SeekOrFind", memory=True)
    seek_or_find.add_children(
        [
            Approach(name="ApproachSubject", target_type=SUBJECT),
            recover_then_approach,
        ]
    )

    # --- keep signalling the target while the human stays near it ------------
    show_while_close = py_trees.composites.Sequence(
        name="ShowWhileSubjectClose", memory=True
    )
    show_while_close.add_children(
        [
            FindPeople(name="FindPersonClose", head=None),
            CheckSubjectTargetSuccess(name="CheckSubjectNearTarget"),
            LEDBehaviourSequence("LCatch", "catch_attention", addressed=True),
            TurnToward(name="TurnTowardTarget", target_type=TARGET, head=None),
            LEDBehaviourSequence("LNear", "indicate_close_target"),
        ]
    )

    # --- the human handed the ball over -------------------------------------
    ball_reaction = py_trees.composites.Sequence(name="BallReactionSeq", memory=True)
    ball_reaction.add_children(
        [
            CheckRobotHasBall(name="CheckIfHasBall"),
            FindPeople(name="FindPersonBallReaction", head=None),
            LEDBehaviourSequence("LThank", "thank", addressed=True),
        ]
    )

    # The ball is asked about first on every cycle, as in the dog tree. It used
    # to come after the show loop in the root sequence, and that loop fails
    # whenever the human is not at the target -- restarting the root -- so the
    # ball reaction could never be reached. Neither applying still restarts the
    # root: find the human, ask for attention and show the target again.
    ball_or_show = py_trees.composites.Selector("BallOrShowSelector", memory=True)
    ball_or_show.add_children(
        [
            py_trees.decorators.Repeat(
                name="BallReactionRepeat", child=ball_reaction, num_success=-1
            ),
            show_while_close,
        ]
    )

    # --- lead: the lights run for the whole drive ----------------------------
    # The drive is what ends the parallel; the signal beside it never finishes
    # by itself, which is why the policy names the drive rather than "one".
    drive = Approach(name="ApproachTarget", target_type=LAST_CHECKPOINT)
    lead = py_trees.composites.Parallel(
        name="LeadWithLights",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[drive], synchronise=False
        ),
    )
    lead.add_children(
        [LEDLeadSignal(name="LLead", target_type=LAST_CHECKPOINT), drive]
    )

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(
                name="LoadConstantParams",
                yaml_path=yaml_path,
                scripts=LED_SCRIPTS,
                required=tuple(LEAD_KEYS.values()),
            ),
            seek_or_find,
            TurnToward(name="TurnTowardSubject", target_type=SUBJECT, head=None),
            LEDBehaviourSequence("LCatchO", "catch_attention", addressed=True),
            # Drive to the last checkpoint of the route, then face the target itself
            # to signal it -- the robot stops short of where the human should end up.
            lead,
            # Nothing asks for the ball from anywhere but the end of the route.
            # The drive only succeeds there, so this holds already; it is here
            # so that it goes on holding. A robot that is not there fails the
            # root, which finds the human and leads again. The route index is
            # left out because this tree never walks it.
            CheckRobotAtLastCheckpoint(name="CheckRobotAtRouteEnd", by_index=False),
            LEDBehaviourSequence("LShow", "indicate_target"),
            py_trees.decorators.Repeat(
                name="BallOrShowLoop", child=ball_or_show, num_success=-1
            ),
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
