#!/usr/bin/env python3
"""
Unit tests for the head following the body the LiDAR expects.

The claims are the ones asked for on 2026-10-02: a nearer LiDAR signal makes the
head look further up, a known seat makes it look lower instead, and the head is
not left chasing the range noise of whatever it is looking at. The scenes are
the ones in the bags of 2026-09-30. Pure Python.
"""

import math
import os

import pytest

from mecanumbot_movement_behaviours.body_gaze import (
    BEAN_BAG,
    BODY_PROFILES,
    CHAIR,
    STANDING,
    BodyGazeController,
    GazeSettings,
    GazeTarget,
    NeckModel,
    Seat,
    choose_tilt,
    gaze_targets,
    profiles_from_literals,
    seat_from_literal,
    tilt_interval,
)

NECK = NeckModel()
SETTINGS = GazeSettings()
# The robot at the map origin, looking down +x.
ROBOT = (0.0, 0.0, 0.0)
REST = 6.5

BEAN_BAG_SEAT = Seat(BEAN_BAG, 1.5, 0.0, 0.7)
CHAIR_SEAT = Seat(CHAIR, 3.9, 0.0, 0.6)


def standing_at(distance):
    low, high = BODY_PROFILES[STANDING]
    return GazeTarget(distance, low, high, "somebody")


def pos_for(candidates=(), people=(), seats=(), robot=ROBOT):
    controller = BodyGazeController(seats=seats)
    return controller.desired(robot, list(people), list(candidates))[0]


def in_frame(pos, distance, height):
    low, high = NECK.visible_band(pos, distance)
    return low <= height <= high


class TestNeckModel:
    def test_the_measured_calibration(self):
        # 4.7 degrees up at 600 ticks, 0.29 degrees a tick.
        assert math.degrees(NECK.pitch(6.0)) == pytest.approx(4.7)
        assert math.degrees(NECK.pitch(6.5)) == pytest.approx(19.2, abs=0.1)
        assert math.degrees(NECK.pitch(7.0)) == pytest.approx(33.7, abs=0.1)

    def test_position_and_tilt_are_inverses(self):
        for pos in (5.5, 6.0, 6.84, 8.0):
            assert NECK.pos(NECK.pitch(pos)) == pytest.approx(pos)

    def test_the_servo_range_is_respected(self):
        assert NECK.pos(math.radians(120.0)) == NECK.max_pos
        assert NECK.pos(math.radians(-120.0)) == NECK.min_pos

    def test_what_the_trials_of_2026_09_30_saw(self):
        # Head at 6.5: a pair of jeans from 0.7 m, and at 1.43 m a frame whose
        # bottom edge is 0.33 m up -- so a face at 0.55 m is at the very bottom.
        low, high = NECK.visible_band(6.5, 0.7)
        assert high < 0.7
        low, high = NECK.visible_band(6.5, 1.43)
        assert low == pytest.approx(0.33, abs=0.03)


class TestCloserMeansHigher:
    def test_a_nearer_body_tilts_the_head_further_up(self):
        positions = [pos_for(candidates=[(d, 0.0)]) for d in (3.5, 2.0, 1.2, 0.7)]
        assert positions == sorted(positions)
        assert positions[-1] - positions[0] > 1.0

    def test_a_standing_head_is_in_frame_at_every_range(self):
        for distance in (0.6, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0):
            pos = pos_for(candidates=[(distance, 0.0)])
            assert in_frame(pos, distance, 1.45), distance  # shoulders
            assert in_frame(pos, distance, 1.70), distance  # head

    def test_the_seeking_pose_alone_loses_the_head_up_close(self):
        # Why one fixed pose was never going to do.
        assert not in_frame(REST, 0.7, 1.45)
        assert not in_frame(7.0, 0.7, 1.45)

    def test_a_body_too_tall_for_the_frame_keeps_its_top(self):
        # Somebody on a bean bag from 0.8 m: floor to head is more than the
        # frame holds, and it is the head and shoulders the detector needs.
        low, high = BODY_PROFILES[BEAN_BAG]
        target = GazeTarget(0.8, low, high, "somebody")
        lowest, highest, ideal = tilt_interval(NECK, target, SETTINGS.frame_margin)
        assert lowest == highest == ideal
        top = NECK.pitch(NECK.pos(ideal)) + NECK.vfov / 2.0
        top_of_head = math.atan2(high - NECK.camera_height, 0.8 - NECK.camera_forward)
        assert top - top_of_head == pytest.approx(SETTINGS.frame_margin, abs=1e-6)

    def test_a_body_that_fits_is_centred(self):
        low, high, ideal = tilt_interval(NECK, standing_at(3.0), SETTINGS.frame_margin)
        assert low < ideal < high


class TestSeats:
    def test_somebody_on_the_bean_bag_is_looked_at_lower(self):
        # The 08:46 run: a LiDAR return 1.43 m away, where the bean bag is.
        standing = pos_for(candidates=[(1.43, 0.0)])
        seated = pos_for(candidates=[(1.43, 0.0)], seats=[BEAN_BAG_SEAT])
        assert seated < standing - 0.5
        assert in_frame(seated, 1.43, 0.55)  # the face
        assert in_frame(seated, 1.43, 0.40)  # the shoulders
        assert not in_frame(standing, 1.43, 0.40)

    def test_a_chair_is_between_the_two(self):
        seats = [Seat(CHAIR, 1.5, 0.0, 0.6)]
        standing = pos_for(candidates=[(1.5, 0.0)])
        bean_bag = pos_for(candidates=[(1.5, 0.0)], seats=[BEAN_BAG_SEAT])
        chair = pos_for(candidates=[(1.5, 0.0)], seats=seats)
        assert bean_bag < chair < standing

    def test_a_seat_is_looked_at_with_no_lidar_return_on_it(self):
        # The chair against the wall in the 09:02 run: the person's legs and
        # the wall were one return, so the LiDAR reported nothing there.
        pos = pos_for(seats=[CHAIR_SEAT])
        assert pos is not None
        assert in_frame(pos, 3.9, 0.95) and in_frame(pos, 3.9, 0.5)

    def test_a_seat_with_a_return_on_it_is_one_target_not_two(self):
        targets = gaze_targets(ROBOT, [], [(1.4, 0.1)], [BEAN_BAG_SEAT], BODY_PROFILES)
        assert len(targets) == 1
        assert "bean_bag" in targets[0].label

    def test_a_seat_behind_the_robot_is_not_looked_at(self):
        assert pos_for(seats=[Seat(CHAIR, -2.0, 0.0)]) is None

    def test_a_seat_can_carry_its_own_heights(self):
        low_stool = Seat(CHAIR, 1.5, 0.0, 0.6, low=0.2, high=0.9)
        assert pos_for(seats=[low_stool]) < pos_for(seats=[Seat(CHAIR, 1.5, 0.0, 0.6)])

    def test_an_unknown_kind_is_treated_as_standing(self):
        assert pos_for(seats=[Seat("hammock", 1.5, 0.0)]) == pos_for(
            candidates=[(1.5, 0.0)]
        )


class TestWhatIsLookedAt:
    def test_nothing_in_view_is_nothing_to_look_at(self):
        assert pos_for() is None

    def test_out_of_the_picture_is_ignored(self):
        assert pos_for(candidates=[(0.0, 2.0)]) is None  # 90 degrees to the left
        assert pos_for(candidates=[(-2.0, 0.0)]) is None  # behind

    def test_just_outside_the_picture_is_already_attended_to(self):
        # 30 degrees off axis: outside the 25.5 degree half-view, inside the
        # margin, so the head is ready as a turn brings the body into shot.
        bearing = math.radians(30.0)
        assert pos_for(candidates=[(2 * math.cos(bearing), 2 * math.sin(bearing))])

    def test_too_far_and_too_near_are_ignored(self):
        assert pos_for(candidates=[(6.0, 0.0)]) is None
        assert pos_for(candidates=[(0.1, 0.0)]) is None

    def test_the_view_turns_with_the_robot(self):
        facing_left = (0.0, 0.0, math.pi / 2.0)
        assert pos_for(candidates=[(0.0, 2.0)], robot=facing_left) is not None

    def test_a_tracked_person_comes_before_a_nearer_candidate(self):
        # A table leg at 1 m must not take the head off the person at 3 m.
        both = pos_for(people=[(3.0, 0.0)], candidates=[(1.0, 0.3)])
        alone = pos_for(people=[(3.0, 0.0)])
        assert in_frame(both, 3.0, 1.70)
        assert both == pytest.approx(alone, abs=0.35)

    def test_of_two_candidates_the_nearer_decides(self):
        near_only = pos_for(candidates=[(1.0, 0.0)])
        assert pos_for(candidates=[(3.5, 0.2), (1.0, 0.0)]) == near_only

    def test_two_far_bodies_are_both_kept_in_frame(self):
        targets = [standing_at(3.0), standing_at(4.0)]
        tilt = choose_tilt(NECK, targets, SETTINGS.frame_margin)
        for target in targets:
            low, high, _ = tilt_interval(NECK, target, SETTINGS.frame_margin)
            assert low - 1e-9 <= tilt <= high + 1e-9

    def test_the_gaze_stays_inside_its_own_limits(self):
        assert pos_for(candidates=[(0.35, 0.0)]) <= SETTINGS.max_pos
        far_low = pos_for(seats=[Seat(BEAN_BAG, 4.4, 0.0)])
        assert far_low >= SETTINGS.min_pos


class TestController:
    def test_a_target_moves_the_head(self):
        gaze = BodyGazeController()
        pos = gaze.update(0.0, ROBOT, [], [(0.8, 0.0)], REST, REST)
        assert pos > REST + 0.5
        assert "candidate" in gaze.reason

    def test_range_noise_does_not_move_it(self):
        gaze = BodyGazeController()
        pos = gaze.update(0.0, ROBOT, [], [(2.0, 0.0)], REST, REST)
        assert gaze.update(1.0, ROBOT, [], [(2.05, 0.0)], REST, pos) is None

    def test_commands_are_spaced_out(self):
        gaze = BodyGazeController()
        pos = gaze.update(0.0, ROBOT, [], [(3.0, 0.0)], REST, REST - 1.0)
        assert pos is not None
        assert gaze.update(0.1, ROBOT, [], [(0.7, 0.0)], REST, pos) is None
        assert gaze.update(0.6, ROBOT, [], [(0.7, 0.0)], REST, pos) > pos

    def test_a_dropped_scan_does_not_make_the_head_nod(self):
        gaze = BodyGazeController()
        pos = gaze.update(0.0, ROBOT, [], [(0.8, 0.0)], REST, REST)
        assert gaze.update(0.5, ROBOT, [], [], REST, pos) is None

    def test_with_nothing_left_the_head_returns_to_the_seeking_pose(self):
        gaze = BodyGazeController()
        pos = gaze.update(0.0, ROBOT, [], [(0.8, 0.0)], REST, REST)
        assert gaze.update(2.0, ROBOT, [], [], REST, pos) == REST
        assert "seeking pose" in gaze.reason

    def test_a_head_never_moved_is_taken_to_the_seeking_pose(self):
        assert BodyGazeController().update(0.0, ROBOT, [], [], REST, None) == REST

    def test_without_a_robot_pose_nothing_is_commanded(self):
        assert BodyGazeController().update(0.0, None, [], [(1.0, 0.0)], REST, REST) is None


class TestConstantsFile:
    def test_a_seat_entry(self):
        seat = seat_from_literal({"seat": "bean_bag", "x": -0.8, "y": 2.2, "radius": 0.7})
        assert seat == Seat(BEAN_BAG, -0.8, 2.2, 0.7)
        assert seat.holds(-0.6, 2.33) and not seat.holds(1.0, 2.2)

    def test_a_seat_without_a_radius_gets_the_default(self):
        assert seat_from_literal({"seat": "chair", "x": 1, "y": 2}).radius == 0.6

    def test_a_malformed_seat_says_what_a_seat_is(self):
        with pytest.raises(ValueError, match="a seat is"):
            seat_from_literal({"seat": "chair", "X": 1.0, "Y": 2.0})

    def test_a_seat_decoded_as_a_point_is_refused(self):
        # What the loader makes of an entry spelled with 'X' and 'Y'.
        with pytest.raises(ValueError, match="a seat is"):
            seat_from_literal(object())

    def test_profiles_override_only_what_they_name(self):
        profiles = profiles_from_literals([{"body": "chair", "low": 0.3, "high": 1.2}])
        assert profiles[CHAIR] == (0.3, 1.2)
        assert profiles[STANDING] == BODY_PROFILES[STANDING]

    def test_an_upside_down_profile_is_refused(self):
        with pytest.raises(ValueError, match="not below"):
            profiles_from_literals([{"body": "chair", "low": 1.2, "high": 0.3}])

    @pytest.mark.parametrize(
        "name", ["Eto_behaviour_setting_constants.yaml", "behaviour_setting_constants.yaml"]
    )
    def test_the_shipped_leading_constants_describe_a_gaze(self, name):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "mecanumbot_leading_behaviour",
            "config",
            name,
        )
        if not os.path.exists(path):
            pytest.skip("the leading constants are not checked out beside this package")
        params = pytest.importorskip("mecanumbot_bt_config.params")
        defaults = pytest.importorskip("mecanumbot_movement_behaviours.defaults")
        values = params.blackboard_values(
            params.load_params(path), defaults.MOVEMENT_DEFAULTS
        )
        controller = defaults.build_body_gaze(values)
        assert values["body_gaze_enabled"] is True
        assert set(controller.profiles) >= {STANDING, CHAIR, BEAN_BAG}
        for seat in controller.seats:
            assert seat.kind in controller.profiles
