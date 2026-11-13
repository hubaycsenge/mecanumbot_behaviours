"""
Loading the object-choice constants, which are three trees' constants at once.

The tree runs the ostensive package's exchange and the fetch package's approach
and grab, unchanged, so the blackboard has to hold what each of them reads:

* the ostensive keys, all **required** exactly as in that package -- a field of
  view or a gesture threshold invented here would make the cue a different cue;
* the fetch keys, with that package's defaults except `FETCH_OVERRIDES`, and
  its required list (grippers, approach and grasp distances);
* the `oc_*` keys of this package.

The run state is the union of the two borrowed trees' state plus a trial's own
(`oc_*`), because their behaviours expect their keys to exist before they write
them.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_fetch_behaviour.behaviours.blackboard_managers import (
    FETCH_REQUIRED,
    RUN_STATE as FETCH_RUN_STATE,
    check_fetch_geometry,
)
from mecanumbot_fetch_behaviour.defaults import FETCH_DEFAULTS
from mecanumbot_movement_behaviours.defaults import (
    MOVEMENT_DEFAULTS,
    configure_accessories,
)
from mecanumbot_ostensive_behaviour.behaviours.blackboard_managers import (
    OSTENSIVE_DEFAULTS,
    OSTENSIVE_PARAMS,
    RUN_STATE as OSTENSIVE_RUN_STATE,
    check_nod,
    report_mirroring,
)

from mecanumbot_object_choice_behaviour.behaviours.ros_interfaces import TargetRelay
from mecanumbot_object_choice_behaviour.defaults import (
    FETCH_OVERRIDES,
    OC_DEFAULTS,
    RESPONSE_MODES,
)

LOADED_DEFAULTS = (
    RUNTIME_DEFAULTS,
    MOVEMENT_DEFAULTS,
    OSTENSIVE_DEFAULTS,
    FETCH_DEFAULTS,
    FETCH_OVERRIDES,
    OC_DEFAULTS,
)

OC_REQUIRED = OSTENSIVE_PARAMS + FETCH_REQUIRED

# A trial's own state. `oc_trial` is the record being written as the trial goes
# -- every stage adds what it decided -- and is what `FinishTrial` publishes.
OC_RUN_STATE = {
    "oc_start_pose": None,
    "oc_bricks": dict,
    "oc_choice": None,
    "oc_trial": dict,
    "oc_trial_index": 0,
}

RUN_STATE = {**OSTENSIVE_RUN_STATE, **FETCH_RUN_STATE, **OC_RUN_STATE}


def check_object_choice(node, blackboard, values):
    """Warn rather than fail when the constants cannot produce the trial."""
    mode = values["oc_response_mode"]
    if mode not in RESPONSE_MODES:
        node.get_logger().warn(
            f"oc_response_mode '{mode}' is not one of {RESPONSE_MODES}; the tree "
            "will refuse to build"
        )
    colours = list(values["oc_brick_colours"] or [])
    if colours and len(colours) < 2:
        node.get_logger().warn(
            f"oc_brick_colours names one colour ({colours}); a choice between "
            "one brick is not a choice"
        )
    seek = values["neck_seek_pos"]
    survey = values["oc_survey_head"]
    if survey >= seek:
        node.get_logger().warn(
            f"oc_survey_head ({survey}) is not below neck_seek_pos ({seek}); the "
            "bricks are surveyed looking at least as high as the person is read, "
            "and the floor they stand on will be out of frame"
        )
    node.get_logger().info(
        f"object choice: response '{mode}', bricks "
        + (", ".join(colours) if colours else f"any {values['oc_min_bricks']}+ colours")
        + f", survey head {survey}, person head {seek}"
    )


def start_relay(node, blackboard, values):
    """Start the relay that shows the fetch behaviours only the chosen brick."""
    TargetRelay.start(node)


class ObjectChoiceParamsToBlackboard(ParamsToBlackboard):
    """Load the ostensive, fetch and object-choice constants together."""

    def __init__(self, name, yaml_path):
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS,
            required=OC_REQUIRED,
            state=RUN_STATE,
            on_loaded=(
                configure_accessories,
                check_nod,
                report_mirroring,
                check_fetch_geometry,
                check_object_choice,
                start_relay,
            ),
        )


def register_trial_keys(blackboard, access=py_trees.common.Access.WRITE):
    """Give a blackboard client access to a trial's own state."""
    for key in OC_RUN_STATE:
        blackboard.register_key(key=key, access=access)
