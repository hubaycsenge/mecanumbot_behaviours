"""
The tunables the leading conditions add to the movement library's own.

Eight values are about *this* experiment rather than about moving: the pauses
that pace the signalling, how many recovery attempts a lost human is worth, how
far past a checkpoint somebody has to be standing before that stretch counts
as walked, and the four settings of the LED condition's lead signal. Everything else a leading tree tunes belongs to the behaviours it
borrows, and its default lives with them in
`mecanumbot_movement_behaviours.defaults`.

`LOADED_DEFAULTS` is what a tree hands to `ParamsToBlackboard`, and `REQUIRED`
is what no constants file may leave out: the distances and the route are the
experiment, and inventing one silently would be worse than failing.
"""

import math

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
    # --- the LED condition's signal while it leads -----------------------------
    # What the flow of the light follows: "target" sends it towards the side
    # the destination lies on once that is off the robot's heading, "none"
    # keeps it flowing ahead throughout.
    "LED_lead_direction": "target",
    # How far off the heading the destination has to be before the light stops
    # flowing ahead and flows towards its side.
    "LED_lead_straight_band": math.radians(30.0),
    # Colour the panels fill with as the drive progresses (2 is green).
    "LED_lead_progress_color": 2,
    # Whether the signals meant for the person light only the half of the robot
    # that faces them.
    "LED_address_person": True,
    # --- pointing at the target with the lights, at the end of the route ------
    # The panels on the target's side blink `LED_direction_color` (7 is yellow)
    # in `LED_direction_mode` (5 is the fast blink) for `LED_direction_hold`
    # seconds, and the others show `LED_direction_base_color` (0 is black). A
    # panel points at the target when the target is within
    # `LED_direction_spread` of the diagonal the panel faces.
    "LED_direction_mode": 5,
    "LED_direction_color": 7,
    "LED_direction_base_color": 0,
    "LED_direction_spread": math.radians(67.5),
    "LED_direction_hold": 6.0,
    # --- when the trial starts (trial_start.py) --------------------------------
    # Every leading tree holds still after loading until `people_fusion` tracks
    # a person (`trial_start_camera`) or the LiDAR sees something new standing
    # in front of the room it saw first (`trial_start_lidar`).
    # `trial_start_enabled: false` starts at once, as before.
    "trial_start_enabled": True,
    "trial_start_camera": True,
    "trial_start_lidar": True,
    # The scan comparison (trial_start.py): the nearest return in each
    # direction over the first `baseline_scans` is the room; a return
    # `min_change` m nearer than every room return within `angle_tolerance`,
    # within `max_range`, is new; `min_beams` neighbouring new returns are an
    # object; and one that holds its bearing within `persist_gate` for
    # `persist_scans` scans running starts the trial.
    "trial_start_baseline_scans": 20,
    "trial_start_min_change": 0.3,
    "trial_start_angle_tolerance": math.radians(2.0),
    "trial_start_max_range": 4.5,
    "trial_start_min_beams": 3,
    "trial_start_persist_scans": 5,
    "trial_start_persist_gate": math.radians(10.0),
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
