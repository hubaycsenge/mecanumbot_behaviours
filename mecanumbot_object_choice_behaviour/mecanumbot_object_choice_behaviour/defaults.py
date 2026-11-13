"""
The tunables of the object-choice tree, and how it borrows two other trees' keys.

This tree is the ostensive exchange and the fetch approach joined by a choice,
and it reuses both packages' behaviours unchanged. Those behaviours read their
constants off the blackboard under their own names, so the object-choice
constants file carries ostensive keys, fetch keys and its own `oc_*` keys side
by side, and the loader writes all three. `OC_DEFAULTS` is only what is new
here, plus `FETCH_OVERRIDES`: the few fetch defaults that are wrong for a brick.

Neck tilts are the accessory board's units (ticks / 100). They were worked out
from the neck calibration of 2026-10-21 -- 0.29 deg per tick, the lens level at
~505 ticks -- for a person standing ~2.5 m away and bricks 1-3 m out on the
floor. They are computed, not measured on the robot, and are the first numbers
to check: `cam_brick_detections/debug_image/compressed` shows whether the
survey tilt sees the bricks, and the pose detector's debug image whether the
person tilt sees shoulders and wrists.
"""

import math

from mecanumbot_bt_config.blackboard import Tunables

RESPONSE_APPROACH = "approach"
RESPONSE_GRAB = "grab"
RESPONSE_MODES = (RESPONSE_APPROACH, RESPONSE_GRAB)

OC_DEFAULTS = {
    # ===== What the robot does with its choice ================================
    # approach | grab. `approach` drives to the chosen brick and stops facing it
    # at `fetch_approach_stop`, which is the classic object-choice response: the
    # choice is the first object the subject goes to. `grab` closes the
    # grabbers on it and carries it to the person, the fetch tree's secure and
    # delivery phases unchanged. A research choice about what is being scored,
    # and read while the tree is built, so changing it needs a restart.
    "oc_response_mode": RESPONSE_APPROACH,
    # How long the robot holds still at the chosen brick before returning [s],
    # so the choice is unambiguous on video.
    "oc_choice_hold": 3.0,

    # ===== The bricks =========================================================
    # Which colours a trial must find, e.g. [red, blue]. Empty: any colours,
    # at least `oc_min_bricks` of them. Naming them is what lets the survey
    # refuse a trial whose layout is not the one the protocol says.
    "oc_brick_colours": [],
    "oc_min_bricks": 2,

    # ===== Surveying them before the trial ===================================
    # The neck tilt the bricks are surveyed at: the lens ~8 deg below level,
    # which puts the floor from ~0.5 m out to beyond 3 m in frame. 480 ticks.
    "oc_survey_head": 4.8,
    # Seconds to let the head settle, then to collect sightings over, and the
    # sightings a colour needs in that window to count as surveyed.
    "oc_survey_settle": 1.0,
    "oc_survey_window": 2.0,
    "oc_survey_min_samples": 3,
    # Give up on the survey after this [s]: the bricks are not where the camera
    # can see them, and the trial is recorded as such.
    "oc_survey_timeout": 20.0,

    # ===== Deciding which brick was meant =====================================
    # See choice_geometry.py. The most a brick may lie off the pointing ray, as
    # seen from the person, and the least the best brick must beat the second
    # by. Below the margin the trial is recorded as ambiguous, not guessed.
    "oc_max_deviation": math.radians(35.0),
    "oc_min_margin": math.radians(10.0),
    # A brick nearer the person than this is left out: its bearing from them is
    # dominated by the torso estimate the ray is anchored at [m].
    "oc_min_brick_distance": 0.3,

    # ===== Between trials =====================================================
    # Wait for a message on /mecanumbot/object_choice/next before each trial,
    # so the experimenter resets the bricks with the robot standing still.
    # False: start the next trial `oc_rest` seconds after the last.
    "oc_wait_for_go": True,
    "oc_rest": 5.0,
    # How long the drive back to the start pose may take [s].
    "oc_return_timeout": 60.0,
}

# Fetch defaults that are wrong for a brick. Later defaults win in the loader,
# so these replace the fetch package's.
FETCH_OVERRIDES = {
    # The relay already passes only the chosen brick, under its colour, so the
    # fetch behaviours must not filter by a label of their own.
    "fetch_ball_class": "",
    # No lights: the object-choice response is movement only, so nothing but
    # the choice itself can cue the person.
    "fetch_led_enabled": False,
    # Brick scores are the blob's fill, which the detector floors at 0.45.
    "fetch_detection_threshold": 0.4,
    # The head the fetch tracker starts from and falls back to: the survey tilt
    # (see above), and low enough to see a brick under the lens.
    "fetch_head_search": 4.8,
    "fetch_head_low": 3.6,
    "fetch_head_high": 5.6,
}

TUNABLES = Tunables(OC_DEFAULTS)

constant = TUNABLES.constant
file_constant = TUNABLES.file_constant
register_param_keys = TUNABLES.register_param_keys

PARAM_KEYS = TUNABLES.keys
