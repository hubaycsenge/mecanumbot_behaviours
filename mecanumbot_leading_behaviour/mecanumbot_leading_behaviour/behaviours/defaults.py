"""
The tunables the leading conditions add to the movement library's own.

Four numbers are about *this* experiment rather than about moving: the pauses
that pace the signalling, how many recovery attempts a lost human is worth, and
how far past a checkpoint somebody has to be standing before that stretch counts
as walked. Everything else a leading tree tunes belongs to the behaviours it
borrows, and its default lives with them in
`mecanumbot_movement_behaviours.defaults`.

`LOADED_DEFAULTS` is what a tree hands to `ParamsToBlackboard`, and `REQUIRED`
is what no constants file may leave out: the distances and the route are the
experiment, and inventing one silently would be worse than failing.
"""

from mecanumbot_bt_config.blackboard import Tunables
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_movement_behaviours.defaults import MOVEMENT_DEFAULTS

LEADING_DEFAULTS = {
    # --- getting the human back -----------------------------------------------
    # How far past a checkpoint (as a fraction of the stretch to the next one)
    # the human has to be before that stretch counts as walked.
    "resume_passed_margin": 0.15,
    # Attempts at "patrol, re-engage, resume" before the tree gives up and
    # unwinds to the top.
    "recover_retries": 3,
    # --- pacing of the signalling ---------------------------------------------
    "thank_delay": 1.0,
    "show_turn_delay": 2.0,
}

# The order matters: a key declared in more than one of these takes the value
# from the last mapping that mentions it.
LOADED_DEFAULTS = (RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS, LEADING_DEFAULTS)

# Keys a leading constants file must declare, because they are the experiment
# rather than a tuning of it. The signalling scripts are added by the trees that
# actually signal -- see `blackboard_managers.SCRIPT_KEYS`.
REQUIRED = (
    "init_delay",
    "robot_closeness_threshold",
    "robot_approach_distance",
    "target_reached_threshold",
    "visibility_time_threshold",
    "Dog_following_max_threshold",
    "Dog_max_wander_allowed",
    "Dog_checkpoints",
)

TUNABLES = Tunables(*LOADED_DEFAULTS)

constant = TUNABLES.constant
resolve = TUNABLES.resolve
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys
