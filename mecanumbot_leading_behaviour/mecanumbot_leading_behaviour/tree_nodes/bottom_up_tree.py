"""Baseline sequence: approach the human, drive the route, indicate with LEDs."""

import py_trees

from mecanumbot_leading_behaviour.behaviours.LED_behaviours import LEDBehaviourSequence
from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.movement_managers import (
    Approach,
    CheckSubjectTargetSuccess,
    TurnToward,
)
from mecanumbot_leading_behaviour.behaviours.targets import (
    LAST_CHECKPOINT,
    SUBJECT,
    TARGET,
)
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "bottom_up_tree"
DEFAULT_YAML_FILENAME = "behaviour_setting_constants.yaml"


def get_yaml_path():
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    # Keeps signalling the target until the human is close enough to it.
    show_until_close = py_trees.composites.Selector(
        name="ShowUntilSubjectClose", memory=True
    )
    show_until_close.add_children(
        [
            CheckSubjectTargetSuccess(name="CheckSubjectNearTarget"),
            TurnToward(name="TurnTowardSubject", target_type=SUBJECT, head=None),
            LEDBehaviourSequence("LCatch", "catch_attention"),
            TurnToward(name="TurnTowardTarget", target_type=TARGET, head=None),
            LEDBehaviourSequence("LNear", "indicate_close_target"),
        ]
    )

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(name="LoadConstantParams", yaml_path=yaml_path),
            py_trees.timers.Timer(name="DelayTimer", duration=2),
            Approach(name="ApproachSubject", target_type=SUBJECT),
            # Stops at the last checkpoint rather than on the target itself.
            Approach(name="ApproachTarget", target_type=LAST_CHECKPOINT),
            LEDBehaviourSequence("LShow", "indicate_target"),
            show_until_close,
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
