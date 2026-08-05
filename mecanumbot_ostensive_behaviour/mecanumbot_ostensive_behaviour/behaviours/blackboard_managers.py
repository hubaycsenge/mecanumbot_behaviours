"""
Loading the ostensive constants and owning the shared interaction state.

The YAML schema is flat -- one block of scalars, no gesture or LED scripts --
because the ostensive condition signals with movement rather than with the timed
colour and neck sequences the leading conditions use. The only sequence here is
the acknowledgement nod.

Angles are declared in the YAML in degrees, because that is how anybody tuning a
field of view or a deadband thinks about them, and are written to the blackboard
in radians under the name without the `_deg` suffix. No behaviour converts an
angle a second time.
"""

import math

import py_trees
import yaml

# Every tree in this package registers under this node name, and the YAML is
# nested under the same key -- the convention the leading package established.
YAML_ROOT_KEYS = ("ostensive_bt_node", "ros__parameters")

# Straight through as floats.
SCALAR_PARAMS = (
    "detection_timeout",
    "attention_dwell",
    "attention_timeout",
    "wrist_above_shoulder_margin",
    "wave_window",
    "wave_min_amplitude",
    "target_max_image_jump",
    "track_loss_timeout",
    "focus_turn_speed",
    "cue_min_extension",
    "cue_full_extension",
    "cue_min_lateral",
    "cue_dwell",
    "cue_timeout",
    "cue_distance",
    "cue_goal_timeout",
)

INTEGER_PARAMS = ("wave_min_reversals",)

STRING_PARAMS = ("attention_signal_mode",)

# YAML name in degrees -> blackboard name in radians.
ANGLE_PARAMS = {
    "camera_hfov_deg": "camera_hfov",
    "target_bearing_tolerance_deg": "target_bearing_tolerance",
    "focus_tolerance_deg": "focus_tolerance",
    "cue_stability_deg": "cue_stability",
}

LIST_PARAMS = ("ack_neck_seq", "ack_neck_times")

# Interaction state, seeded so no behaviour has to test whether a key exists yet.
# `search_spin_sign` belongs to the leading package's `InPlaceTurn`, which the
# turning behaviours here inherit from.
STATE_KEYS = ("ostensive_target", "ostensive_cue", "search_spin_sign")

PARAM_KEYS = (
    SCALAR_PARAMS
    + INTEGER_PARAMS
    + STRING_PARAMS
    + LIST_PARAMS
    + tuple(ANGLE_PARAMS.values())
)


def register_param_keys(blackboard):
    """Give a blackboard client read access to every constant loaded from the YAML."""
    for key in PARAM_KEYS:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)


class OstensiveParamsToBlackboard(py_trees.behaviour.Behaviour):
    """
    Load the ostensive constants onto the blackboard and clear the interaction state.

    Loading happens in `setup`, so the constants are there before any other
    behaviour is ticked. Clearing happens in `initialise`, so restarting the
    tree also drops whoever it was talking to -- an exchange interrupted halfway
    through should not resume against a stale addressee.
    """

    def __init__(self, name, yaml_path):
        super().__init__(name=name)
        self.yaml_path = yaml_path
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in PARAM_KEYS + STATE_KEYS:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Read the YAML and write every constant to the blackboard."""
        self.node = kwargs["node"]
        params = self._load_params()

        for key in SCALAR_PARAMS:
            setattr(self.blackboard, key, float(params[key]))
        for key in INTEGER_PARAMS:
            setattr(self.blackboard, key, int(params[key]))
        for key in STRING_PARAMS:
            setattr(self.blackboard, key, str(params[key]))
        for key in LIST_PARAMS:
            setattr(self.blackboard, key, [float(entry) for entry in params[key]])
        for yaml_key, blackboard_key in ANGLE_PARAMS.items():
            setattr(
                self.blackboard, blackboard_key, math.radians(float(params[yaml_key]))
            )

        self._check_nod()
        self.feedback_message = "constants loaded"
        self.node.get_logger().info(
            f"{self.name}: constants loaded from {self.yaml_path}"
        )
        return True

    def initialise(self):
        """Start the run with nobody addressed and no cue read."""
        self.blackboard.ostensive_target = None
        self.blackboard.ostensive_cue = None
        self.blackboard.search_spin_sign = 0

    def update(self):
        """Return SUCCESS -- the work was done in setup and initialise."""
        return py_trees.common.Status.SUCCESS

    # --- internals ------------------------------------------------------------

    def _load_params(self):
        with open(self.yaml_path, "r") as handle:
            params = yaml.safe_load(handle)
        for key in YAML_ROOT_KEYS:
            params = params[key]
        return params

    def _check_nod(self):
        """Warn rather than fail when the nod sequence and its timings disagree."""
        positions = self.blackboard.ack_neck_seq
        times = self.blackboard.ack_neck_times
        if len(positions) != len(times):
            self.node.get_logger().warn(
                f"{self.name}: ack_neck_seq has {len(positions)} entries but "
                f"ack_neck_times has {len(times)}; the nod will stop at the shorter one"
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
