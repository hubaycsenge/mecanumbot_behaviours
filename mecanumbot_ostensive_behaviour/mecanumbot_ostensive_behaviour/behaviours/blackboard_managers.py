"""
Loading the ostensive constants and owning the shared interaction state.

The loading itself is `mecanumbot_bt_config`'s and knows no key names: it writes
whatever the YAML declares, turns a `_deg` parameter into radians under the name
without the suffix, and fills in a packaged default for anything left out. This
module only says what the file cannot say about itself.

Nearly every constant here is **required**: the ostensive schema is flat -- one
block of scalars, no gesture or LED scripts, because this condition signals with
movement rather than with the timed colour and neck sequences the leading
conditions use -- and every one of those scalars is the experiment rather than a
tuning of it. `OSTENSIVE_PARAMS` is therefore both the list the file must
declare and the list a behaviour registers to read.

The file may also set the `mecanumbot_movement_behaviours` tunables this tree
exercises -- it borrows that package's scanning and turning behaviours, and they
read their constants off the blackboard under fixed names. Setting them here
under those same names is what puts the ostensive condition's turn speeds and
head poses in the ostensive file. Anything it does not mention keeps that
package's default, so the file only has to carry what it actually means to set.
"""

import py_trees

from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_movement_behaviours.defaults import (
    MOVEMENT_DEFAULTS,
    configure_accessories,
)

# Every constant the ostensive tree acts on, under the name it reaches the
# blackboard as -- so an angle appears here without the `_deg` the YAML spells
# it with. All of them are required: the tree has no sensible stand-in for a
# field of view or a dwell time it was not told.
OSTENSIVE_PARAMS = (
    # seeing people at all
    "detection_timeout",
    "camera_hfov",
    # a bid for attention
    "attention_signal_mode",
    "attention_dwell",
    "attention_timeout",
    "wrist_above_shoulder_margin",
    "wave_window",
    "wave_min_amplitude",
    "wave_min_reversals",
    # holding on to whoever made it
    "target_max_image_jump",
    "target_bearing_tolerance",
    "track_loss_timeout",
    "focus_turn_speed",
    "focus_tolerance",
    "focus_timeout",
    # reading where they point
    "cue_min_extension",
    "cue_full_extension",
    "cue_min_lateral",
    "cue_stability",
    "cue_dwell",
    "cue_timeout",
    "cue_distance",
    "cue_goal_timeout",
    # the acknowledgement nod
    "ack_neck_seq",
    "ack_neck_times",
)

# The one constant with a default, because it describes how the robot is built
# rather than how the experiment is run. `mirror_image_x` says the camera hands
# over a mirrored view of the room, which reflects every direction read off the
# image; see `keypoints.mirror_joints`.
OSTENSIVE_DEFAULTS = {"mirror_image_x": True}

LOADED_DEFAULTS = (RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS, OSTENSIVE_DEFAULTS)

PARAM_KEYS = OSTENSIVE_PARAMS + tuple(OSTENSIVE_DEFAULTS)

# Interaction state, seeded so no behaviour has to test whether a key exists
# yet, and cleared on every entry so an exchange interrupted halfway through
# does not resume against a stale addressee. `search_spin_sign` belongs to the
# movement package's `InPlaceTurn`, which the turning behaviours here inherit.
RUN_STATE = {
    "ostensive_target": None,
    "ostensive_cue": None,
    "search_spin_sign": 0,
}


def register_param_keys(blackboard):
    """Give a blackboard client read access to every constant loaded from the YAML."""
    for key in PARAM_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


def check_nod(node, blackboard, values):
    """Warn rather than fail when the acknowledgement nod does not agree with itself."""
    positions = values["ack_neck_seq"]
    times = values["ack_neck_times"]
    if len(positions) != len(times):
        node.get_logger().warn(
            f"ack_neck_seq has {len(positions)} entries but ack_neck_times has "
            f"{len(times)}; the nod will stop at the shorter one"
        )
    # The nod leaves the head wherever its last step put it, and the rest of the
    # exchange expects the seeking pose. Both being in this file, the two can be
    # checked against each other.
    seek = values["neck_seek_pos"]
    if positions and abs(float(positions[-1]) - seek) > 1e-3:
        node.get_logger().warn(
            f"the nod ends at n_pos={positions[-1]}, not at the seeking pose "
            f"neck_seek_pos={seek}; the head will stay there"
        )


def report_mirroring(node, blackboard, values):
    """Log how the camera image is read, which is the constant that hides a mirror bug."""
    # Worth a line of its own: set the wrong way round it produces a robot that
    # turns and drives to the mirror image of where it was told, with nothing
    # else looking wrong.
    node.get_logger().info(
        "the camera image is treated as "
        + (
            "mirrored -- every image column is reflected before it is used"
            if values["mirror_image_x"]
            else "not mirrored"
        )
    )


class OstensiveParamsToBlackboard(ParamsToBlackboard):
    """Load the ostensive constants onto the blackboard and clear the interaction state."""

    def __init__(self, name, yaml_path):
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS,
            required=OSTENSIVE_PARAMS,
            state=RUN_STATE,
            on_loaded=(configure_accessories, check_nod, report_mirroring),
        )


class ClearTargetLock(py_trees.behaviour.Behaviour):
    """
    Forget the addressee and the cue, and return SUCCESS.

    Used twice in the tree: at the end of a completed exchange, so the next one
    starts from a fresh bid for attention rather than carrying on with whoever
    spoke last, and as the fallback that absorbs a failed exchange so the loop
    can begin again.
    """

    def __init__(self, name="ClearTargetLock", reason=""):
        super().__init__(name)
        self.reason = reason
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "ostensive_target", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "ostensive_cue", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        """Keep the node handle for logging."""
        self.node = kwargs["node"]
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def update(self):
        """Drop the lock and the cue."""
        if self.blackboard.ostensive_target is not None:
            self.node.get_logger().info(
                f"{self.name}: releasing the addressee"
                + (f" ({self.reason})" if self.reason else "")
            )
        self.blackboard.ostensive_target = None
        self.blackboard.ostensive_cue = None
        return py_trees.common.Status.SUCCESS
