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

The file also sets the handful of `mecanumbot_leading_behaviour` tunables this
tree exercises -- it borrows that package's scanning and turning behaviours, and
they read their constants off the blackboard under fixed names. Declaring them
here under those same names is what puts the ostensive condition's turn speeds
and head poses in the ostensive file, instead of leaving them to the leading
package's defaults. Anything not declared falls back to those defaults, so this
file only has to mention what it actually means to set.
"""

import math

import py_trees

from mecanumbot_leading_behaviour.behaviours.constants import (
    TUNABLE_DEFAULTS as SHARED_DEFAULTS,
    load_params,
)
from mecanumbot_leading_behaviour.behaviours.ros_interfaces import AccessoryCommander

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
    "focus_timeout",
    "cue_min_extension",
    "cue_full_extension",
    "cue_min_lateral",
    "cue_dwell",
    "cue_timeout",
    "cue_distance",
    "cue_goal_timeout",
)

# Tunables owned by `mecanumbot_leading_behaviour` that this file may set, under
# exactly the blackboard names that package's behaviours read. Optional: a
# missing one keeps the value in that package's `TUNABLE_DEFAULTS`.
SHARED_SCALARS = (
    # the tree runner, read from the file rather than from the blackboard
    "tick_period_ms",
    "setup_timeout",
    # FindPeople, the look-around when nobody is there to be addressed
    "sight_timeout",
    "scan_spin_speed",
    "scan_timeout",
    # the SmoothTurner profile behind every in-place turn, including the
    # re-centring ones (whose top speed is `focus_turn_speed` above)
    "turn_max_speed",
    "turn_accel",
    "turn_decel_gain",
    "turn_min_speed",
    "turn_timeout",
    "turn_nav2_wait",
    # the neck and gripper poses the nod moves between
    "neck_seek_pos",
    "neck_level_pos",
    "gripper_left_neutral",
    "gripper_right_neutral",
)

# Shared tunables declared in degrees, under the same rule as ANGLE_PARAMS.
SHARED_ANGLE_PARAMS = {
    "turn_tolerance_deg": "turn_tolerance",
    "facing_epsilon_deg": "facing_epsilon",
}

INTEGER_PARAMS = ("wave_min_reversals",)

STRING_PARAMS = ("attention_signal_mode",)

# Optional, with the default that applies to the robot as it is currently built.
# `mirror_image_x` says the camera hands over a mirrored view of the room, which
# reflects every direction read off the image; see `keypoints.mirror_joints`.
BOOL_PARAMS = {"mirror_image_x": True}

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
    + tuple(BOOL_PARAMS)
    + tuple(ANGLE_PARAMS.values())
)

# Written to the blackboard as well, but read there by the leading package's
# behaviours through its own `register_param_keys`, not by anything here.
SHARED_KEYS = SHARED_SCALARS + tuple(SHARED_ANGLE_PARAMS.values())


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
        for key in PARAM_KEYS + SHARED_KEYS + STATE_KEYS:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Read the YAML and write every constant to the blackboard."""
        self.node = kwargs["node"]
        params = load_params(self.yaml_path, YAML_ROOT_KEYS)

        for key in SCALAR_PARAMS:
            setattr(self.blackboard, key, float(params[key]))
        for key in INTEGER_PARAMS:
            setattr(self.blackboard, key, int(params[key]))
        for key in STRING_PARAMS:
            setattr(self.blackboard, key, str(params[key]))
        for key, default in BOOL_PARAMS.items():
            setattr(self.blackboard, key, bool(params.get(key, default)))
        for key in LIST_PARAMS:
            setattr(self.blackboard, key, [float(entry) for entry in params[key]])
        for yaml_key, blackboard_key in ANGLE_PARAMS.items():
            setattr(
                self.blackboard, blackboard_key, math.radians(float(params[yaml_key]))
            )

        self._write_shared(params)
        self._check_nod()
        self.feedback_message = "constants loaded"
        self.node.get_logger().info(
            f"{self.name}: constants loaded from {self.yaml_path}"
        )
        # Worth a line of its own in the log: it is the one constant that, set
        # the wrong way round, produces a robot that turns and drives to the
        # mirror image of where it was told, with nothing else looking wrong.
        self.node.get_logger().info(
            f"{self.name}: the camera image is treated as "
            + (
                "mirrored -- every image column is reflected before it is used"
                if self.blackboard.mirror_image_x
                else "not mirrored"
            )
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

    def _write_shared(self, params):
        """Write the leading library's tunables, defaulting the undeclared ones."""
        for key in SHARED_SCALARS:
            setattr(self.blackboard, key, float(params.get(key, SHARED_DEFAULTS[key])))
        for yaml_key, blackboard_key in SHARED_ANGLE_PARAMS.items():
            default = math.degrees(SHARED_DEFAULTS[blackboard_key])
            setattr(
                self.blackboard,
                blackboard_key,
                math.radians(float(params.get(yaml_key, default))),
            )

        # The head poses belong to the commander rather than to any one
        # behaviour; the nod in `acknowledge.py` moves the same neck.
        AccessoryCommander.configure(
            seek_pos=self.blackboard.neck_seek_pos,
            level_pos=self.blackboard.neck_level_pos,
            gripper_left=self.blackboard.gripper_left_neutral,
            gripper_right=self.blackboard.gripper_right_neutral,
        )

    def _check_nod(self):
        """Warn rather than fail when the nod does not agree with itself."""
        positions = self.blackboard.ack_neck_seq
        times = self.blackboard.ack_neck_times
        if len(positions) != len(times):
            self.node.get_logger().warn(
                f"{self.name}: ack_neck_seq has {len(positions)} entries but "
                f"ack_neck_times has {len(times)}; the nod will stop at the shorter one"
            )
        # The nod leaves the head wherever its last step put it, and the rest of
        # the exchange expects the seeking pose. Now that both are in this file,
        # the two can be checked against each other.
        seek = self.blackboard.neck_seek_pos
        if positions and abs(positions[-1] - seek) > 1e-3:
            self.node.get_logger().warn(
                f"{self.name}: the nod ends at n_pos={positions[-1]}, not at the "
                f"seeking pose neck_seek_pos={seek}; the head will stay there"
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
