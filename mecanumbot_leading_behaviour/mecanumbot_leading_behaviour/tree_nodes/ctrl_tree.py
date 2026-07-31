"""Control condition: drive the route with no signalling at all.

Start -> target, no gestures, no LEDs, and the neck is left where the parameter
loader parked it (`head=None` on the turns).
"""

import py_trees

from mecanumbot_leading_behaviour.behaviours.blackboard_managers import (
    ConstantParamsToBlackboard,
)
from mecanumbot_leading_behaviour.behaviours.movement_managers import (
    Approach,
    TurnToward,
)
from mecanumbot_leading_behaviour.behaviours.targets import (
    LAST_CHECKPOINT,
    START,
    TARGET,
)
from mecanumbot_leading_behaviour.tree_nodes.tree_common import (
    resolve_yaml_path,
    run_tree,
)

TREE_NAME = "ctrl_tree"
DEFAULT_YAML_FILENAME = "behaviour_setting_constants.yaml"


def get_yaml_path():
    return resolve_yaml_path(TREE_NAME, DEFAULT_YAML_FILENAME)


def create_root(yaml_path=None):
    if yaml_path is None:
        yaml_path = get_yaml_path()

    root = py_trees.composites.Sequence("ROOT", memory=True)
    root.add_children(
        [
            ConstantParamsToBlackboard(name="LoadConstantParams", yaml_path=yaml_path),
            TurnToward(name="TurnTowardStart", target_type=START, head=None),
            Approach(name="ApproachStart", target_type=START),
            TurnToward(name="TurnTowardTarget", target_type=TARGET, head=None),
            # Stops at the last checkpoint rather than on the target itself.
            Approach(name="ApproachTarget", target_type=LAST_CHECKPOINT),
        ]
    )
    return root


def main(args=None):
    run_tree(create_root, TREE_NAME, DEFAULT_YAML_FILENAME, args=args)


if __name__ == "__main__":
    main()
