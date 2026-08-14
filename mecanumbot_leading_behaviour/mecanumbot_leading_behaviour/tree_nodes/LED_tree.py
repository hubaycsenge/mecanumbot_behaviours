"""
LED leading behaviour: same route, signalled with light patterns.

This is the non-animal comparison condition, so the neck is left alone
(`head=None` on every turn): the camera keeps the lifted tilt the parameter
loader sets at startup, but the robot never gestures with its head.
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    LED_SCRIPTS,
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.route_behaviours import (
    Approach,
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
            LEDBehaviourSequence("LCatch", "catch_attention"),
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
            LEDBehaviourSequence("LThank", "thank"),
        ]
    )

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(
                name="LoadConstantParams",
                yaml_path=yaml_path,
                scripts=LED_SCRIPTS,
            ),
            seek_or_find,
            TurnToward(name="TurnTowardSubject", target_type=SUBJECT, head=None),
            LEDBehaviourSequence("LCatchO", "catch_attention"),
            # Drive to the last checkpoint of the route, then face the target itself
            # to signal it -- the robot stops short of where the human should end up.
            Approach(name="ApproachTarget", target_type=LAST_CHECKPOINT),
            LEDBehaviourSequence("LShow", "indicate_target"),
            py_trees.decorators.Repeat(
                name="ShowWhileCloseLoop", child=show_while_close, num_success=-1
            ),
            py_trees.decorators.Repeat(
                name="BallReactionRepeat", child=ball_reaction, num_success=-1
            ),
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
