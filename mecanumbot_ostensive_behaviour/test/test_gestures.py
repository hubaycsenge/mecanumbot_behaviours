"""
Unit tests for the gesture decoding.

These exercise the layer that turns keypoints into "somebody is asking for
attention" and "they are pointing that way". Nothing here touches ROS or
py_trees, so it runs under bare pytest:

    python3 -m pytest \
        src/mecanumbot_behaviours/mecanumbot_ostensive_behaviour/test/test_gestures.py

The poses are built by `person()`, which places a plausible upper body in
normalized image coordinates. Remember that image y grows *downwards*, so a
raised hand has a smaller y than the shoulder it belongs to, and that `left_*`
is the person's own left, which appears on the right of the image.
"""

import math

from mecanumbot_ostensive_behaviour.behaviours.gestures import (
    RAISED_HAND,
    WAVE,
    WaveTracker,
    attention_signal,
    azimuth_from_lateral_ratio,
    count_reversals,
    pointing_cue,
    raised_hand,
)
from mecanumbot_ostensive_behaviour.behaviours.keypoints import (
    body_scale,
    image_centroid_x,
    keypoint,
    keypoints_from_msg,
    shoulder_width,
)

# A person facing the camera, centred, one shoulder width = 0.20 image units.
SHOULDER_Y = 0.30
HIP_Y = 0.55
HALF_WIDTH = 0.10


def person(
    centre_x=0.5,
    left_wrist=None,
    right_wrist=None,
    scale=1.0,
    missing=(),
):
    """
    Build a keypoint dict for a plausible upper body.

    Wrists default to hanging by the hips. `scale` shrinks or grows the whole
    body about its centre, which is how "the same gesture, further away" is
    written in these tests.
    """
    half = HALF_WIDTH * scale
    shoulder_y = 0.5 - (0.5 - SHOULDER_Y) * scale
    hip_y = 0.5 + (HIP_Y - 0.5) * scale

    # The person's left is on the right-hand side of the image.
    left_shoulder = (centre_x + half, shoulder_y)
    right_shoulder = (centre_x - half, shoulder_y)
    left_hip = (centre_x + half * 0.8, hip_y)
    right_hip = (centre_x - half * 0.8, hip_y)

    joints = {
        "nose": (centre_x, shoulder_y - 0.08 * scale),
        "left_eye": (centre_x + 0.02 * scale, shoulder_y - 0.10 * scale),
        "right_eye": (centre_x - 0.02 * scale, shoulder_y - 0.10 * scale),
        "left_ear": (centre_x + 0.04 * scale, shoulder_y - 0.09 * scale),
        "right_ear": (centre_x - 0.04 * scale, shoulder_y - 0.09 * scale),
        "left_shoulder": left_shoulder,
        "right_shoulder": right_shoulder,
        "left_elbow": (centre_x + half * 1.1, shoulder_y + 0.12 * scale),
        "right_elbow": (centre_x - half * 1.1, shoulder_y + 0.12 * scale),
        "left_wrist": left_wrist if left_wrist else (centre_x + half, hip_y),
        "right_wrist": right_wrist if right_wrist else (centre_x - half, hip_y),
        "left_hip": left_hip,
        "right_hip": right_hip,
        "left_knee": (centre_x + half * 0.8, hip_y + 0.20 * scale),
        "right_knee": (centre_x - half * 0.8, hip_y + 0.20 * scale),
        "left_ankle": (centre_x + half * 0.8, hip_y + 0.40 * scale),
        "right_ankle": (centre_x - half * 0.8, hip_y + 0.40 * scale),
    }
    return {
        name: keypoint(0.0, 0.0) if name in missing else keypoint(*xy)
        for name, xy in joints.items()
    }


# ---------------------------------------------------------------------------
# Body scale and the missing-keypoint conventions
# ---------------------------------------------------------------------------
def test_shoulder_width_is_the_body_scale_for_a_frontal_pose():
    joints = person()
    assert math.isclose(shoulder_width(joints), 2 * HALF_WIDTH, abs_tol=1e-9)
    assert math.isclose(body_scale(joints), 2 * HALF_WIDTH, abs_tol=1e-9)


def test_body_scale_falls_back_to_torso_height_when_turned_side_on():
    # Shoulders almost on top of each other: a person turned towards the side.
    joints = person()
    joints["left_shoulder"] = keypoint(0.5 + 0.005, SHOULDER_Y)
    joints["right_shoulder"] = keypoint(0.5 - 0.005, SHOULDER_Y)
    assert body_scale(joints) > shoulder_width(joints)


def test_zero_coordinates_and_nan_both_read_as_missing():
    # The Ultralytics detector writes (0, 0), DeepStream writes NaN.
    assert not keypoint(0.0, 0.0).visible
    assert not keypoint(float("nan"), float("nan")).visible
    assert keypoint(0.0, 0.4).visible


def test_keypoints_from_msg_reads_the_message_fields():
    class _Position:
        def __init__(self, x, y):
            self.x, self.y = x, y

    class _Pose:
        def __init__(self, x, y):
            self.position = _Position(x, y)

    class _Keypoints:
        pass

    message = _Keypoints()
    for index, field in enumerate(
        (
            "nose",
            "left_eye",
            "right_eye",
            "left_ear",
            "right_ear",
            "left_shoulder",
            "right_shoulder",
            "left_elbow",
            "right_elbow",
            "left_wrist",
            "right_wrist",
            "left_hip",
            "right_hip",
            "left_knee",
            "right_knee",
            "left_ankle",
            "right_ankle",
        )
    ):
        setattr(message, field, _Pose(0.1 * index + 0.01, 0.5))

    joints = keypoints_from_msg(message)
    assert len(joints) == 17
    assert math.isclose(joints["left_shoulder"].x, 0.51, abs_tol=1e-9)


def test_centroid_ignores_an_outstretched_arm():
    # An arm thrown out sideways must not read as the person having moved.
    neutral = person(centre_x=0.5)
    pointing = person(centre_x=0.5, left_wrist=(0.95, 0.28))
    assert math.isclose(
        image_centroid_x(neutral), image_centroid_x(pointing), abs_tol=1e-9
    )


# ---------------------------------------------------------------------------
# Attention: raised hand
# ---------------------------------------------------------------------------
def test_hanging_arms_are_not_a_raised_hand():
    assert raised_hand(person()) is None


def test_a_wrist_above_the_shoulder_is_a_raised_hand():
    assert raised_hand(person(left_wrist=(0.62, 0.15))) == "left"
    assert raised_hand(person(right_wrist=(0.38, 0.15))) == "right"


def test_a_wrist_level_with_the_shoulder_is_not_raised():
    assert raised_hand(person(left_wrist=(0.62, SHOULDER_Y - 0.001))) is None


def test_the_higher_hand_wins_when_both_are_up():
    joints = person(left_wrist=(0.62, 0.20), right_wrist=(0.38, 0.10))
    assert raised_hand(joints) == "right"


def test_the_raise_threshold_scales_with_distance():
    # The same gesture in body units, at half the apparent size, still counts.
    near = person(left_wrist=(0.5 + HALF_WIDTH, SHOULDER_Y - 0.12))
    far = person(
        scale=0.5,
        left_wrist=(0.5 + HALF_WIDTH * 0.5, (0.5 - (0.5 - SHOULDER_Y) * 0.5) - 0.06),
    )
    assert raised_hand(near) == "left"
    assert raised_hand(far) == "left"


# ---------------------------------------------------------------------------
# Attention: waving
# ---------------------------------------------------------------------------
def test_count_reversals_ignores_movement_below_the_amplitude():
    assert count_reversals([0.0, 0.01, -0.01, 0.02, -0.02], 0.25) == 0


def test_count_reversals_counts_turning_points():
    assert count_reversals([0.0, 0.5, 0.0, 0.5], 0.25) == 2


def test_count_reversals_ignores_a_monotone_drift():
    assert count_reversals([0.0, 0.3, 0.6, 0.9, 1.2], 0.25) == 0


def _wave_frames(times, offsets):
    """Build a hand-up person for each sideways wrist offset, in body scales."""
    frames = []
    for time, offset in zip(times, offsets):
        wrist_x = 0.5 + offset * (2 * HALF_WIDTH)
        frames.append((time, person(left_wrist=(wrist_x, SHOULDER_Y - 0.12))))
    return frames


def test_a_still_raised_hand_is_not_a_wave():
    tracker = WaveTracker(window=1.5, min_amplitude=0.25, min_reversals=2)
    waving = False
    for time, joints in _wave_frames(
        [0.0, 0.1, 0.2, 0.3, 0.4, 0.5], [0.0, 0.01, 0.0, 0.01, 0.0, 0.01]
    ):
        waving = tracker.update(time, joints)
    assert not waving


def test_a_hand_moving_side_to_side_is_a_wave():
    tracker = WaveTracker(window=1.5, min_amplitude=0.25, min_reversals=2)
    waving = False
    for time, joints in _wave_frames(
        [0.0, 0.1, 0.2, 0.3, 0.4], [0.0, 0.5, -0.5, 0.5, -0.5]
    ):
        waving = tracker.update(time, joints)
    assert waving


def test_dropping_the_hand_ends_the_wave_immediately():
    tracker = WaveTracker(window=1.5, min_amplitude=0.25, min_reversals=2)
    for time, joints in _wave_frames(
        [0.0, 0.1, 0.2, 0.3, 0.4], [0.0, 0.5, -0.5, 0.5, -0.5]
    ):
        tracker.update(time, joints)
    assert not tracker.update(0.5, person())  # arms back down


def test_the_wave_window_forgets_old_reversals():
    tracker = WaveTracker(window=0.5, min_amplitude=0.25, min_reversals=2)
    for time, joints in _wave_frames([0.0, 0.1, 0.2], [0.0, 0.5, -0.5]):
        tracker.update(time, joints)
    # Two seconds later the hand is up but has been still: the old reversals
    # must have aged out of the window.
    assert not tracker.update(2.0, person(left_wrist=(0.5, SHOULDER_Y - 0.12)))


def test_attention_signal_modes_pick_the_right_gesture():
    tracker = WaveTracker(window=1.5, min_amplitude=0.25, min_reversals=2)
    still = person(left_wrist=(0.5 + HALF_WIDTH, SHOULDER_Y - 0.12))

    assert attention_signal(still, tracker, 0.0, mode=WAVE) is None
    signal = attention_signal(still, tracker, 0.1, mode=RAISED_HAND)
    assert signal is not None and signal.kind == RAISED_HAND


def test_a_wave_is_reported_as_a_wave_in_any_mode():
    tracker = WaveTracker(window=1.5, min_amplitude=0.25, min_reversals=2)
    signal = None
    for time, joints in _wave_frames(
        [0.0, 0.1, 0.2, 0.3, 0.4], [0.0, 0.5, -0.5, 0.5, -0.5]
    ):
        signal = attention_signal(joints, tracker, time, mode="any")
    assert signal is not None and signal.kind == WAVE


# ---------------------------------------------------------------------------
# Direction cue
# ---------------------------------------------------------------------------
def test_hanging_arms_are_not_a_pointing_cue():
    # An arm at the side is as long as a pointing one; the hip line is what
    # separates them.
    assert pointing_cue(person()) is None


def test_an_arm_held_out_sideways_is_a_cue():
    cue = pointing_cue(person(left_wrist=(0.5 + HALF_WIDTH + 0.30, SHOULDER_Y)))
    assert cue is not None
    assert cue.side == "left"
    assert cue.azimuth > 0.0


def test_the_persons_right_arm_gives_a_negative_azimuth():
    cue = pointing_cue(person(right_wrist=(0.5 - HALF_WIDTH - 0.30, SHOULDER_Y)))
    assert cue is not None
    assert cue.side == "right"
    assert cue.azimuth < 0.0


def test_an_arm_pointing_at_the_camera_is_refused_not_guessed():
    # Foreshortened straight towards the lens: barely any sideways reach, so the
    # azimuth is unrecoverable and must not be reported as "straight ahead".
    cue = pointing_cue(person(left_wrist=(0.5 + HALF_WIDTH + 0.02, SHOULDER_Y - 0.02)))
    assert cue is None


def test_a_bent_arm_is_refused():
    cue = pointing_cue(person(left_wrist=(0.5 + HALF_WIDTH + 0.05, SHOULDER_Y - 0.05)))
    assert cue is None


def test_the_more_extended_arm_wins():
    joints = person(
        left_wrist=(0.5 + HALF_WIDTH + 0.12, SHOULDER_Y - 0.10),
        right_wrist=(0.5 - HALF_WIDTH - 0.34, SHOULDER_Y),
    )
    cue = pointing_cue(joints)
    assert cue is not None and cue.side == "right"


def test_the_cue_is_the_same_at_half_the_apparent_size():
    near = pointing_cue(person(left_wrist=(0.5 + HALF_WIDTH + 0.30, SHOULDER_Y)))
    far_shoulder_y = 0.5 - (0.5 - SHOULDER_Y) * 0.5
    far = pointing_cue(
        person(
            scale=0.5,
            left_wrist=(0.5 + HALF_WIDTH * 0.5 + 0.15, far_shoulder_y),
        )
    )
    assert near is not None and far is not None
    assert math.isclose(near.azimuth, far.azimuth, abs_tol=1e-6)


def test_azimuth_saturates_at_a_quarter_turn():
    assert math.isclose(azimuth_from_lateral_ratio(1.5, 1.5), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(azimuth_from_lateral_ratio(9.9, 1.5), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(
        azimuth_from_lateral_ratio(-9.9, 1.5), -math.pi / 2, abs_tol=1e-9
    )


def test_half_a_reach_is_thirty_degrees_not_forty_five():
    # The ratio is treated as a sine, which is what an arm swinging on its
    # shoulder actually does.
    azimuth = azimuth_from_lateral_ratio(0.75, 1.5)
    assert math.isclose(math.degrees(azimuth), 30.0, abs_tol=1e-6)
