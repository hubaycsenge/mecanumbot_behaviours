#!/usr/bin/env python3
"""
Unit tests for tilting the head down onto a face at the bottom of the frame.

The claims are the ones the user asked for on 2026-09-30: a face at the bottom
of the frame makes the head look down, and the head always comes back up, so
the search is never stuck looking too low. Pure Python.
"""

import pytest

from mecanumbot_movement_behaviours.look_down import LookDown

SEEK = 6.5


def test_a_fresh_cue_lowers_the_head_by_the_step():
    look = LookDown(step=0.5, hold=3.0, cooldown=4.0, cue_age=0.5)
    assert look.update(10.0, 9.9, SEEK) == pytest.approx(6.0)
    assert look.lowered


def test_no_cue_or_a_stale_one_leaves_the_head_alone():
    look = LookDown(cue_age=0.5)
    assert look.update(10.0, None, SEEK) is None
    assert look.update(10.0, 9.0, SEEK) is None
    assert not look.lowered


def test_the_head_comes_back_up_after_the_hold_even_if_the_cue_persists():
    look = LookDown(step=0.5, hold=3.0, cooldown=4.0, cue_age=0.5)
    look.update(10.0, 10.0, SEEK)
    assert look.update(12.0, 12.0, 6.0) is None
    assert look.update(13.0, 13.0, 6.0) == pytest.approx(SEEK)
    assert not look.lowered


def test_after_raising_the_cue_is_ignored_for_the_cooldown():
    look = LookDown(step=0.5, hold=3.0, cooldown=4.0, cue_age=0.5)
    look.update(10.0, 10.0, SEEK)
    look.update(13.0, 13.0, 6.0)
    assert look.update(15.0, 15.0, SEEK) is None
    assert look.update(17.1, 17.0, SEEK) == pytest.approx(6.0)


def test_release_raises_the_head_straight_away():
    look = LookDown()
    look.update(10.0, 10.0, SEEK)
    assert look.release(10.5) == pytest.approx(SEEK)
    assert look.release(10.6) is None


def test_the_head_never_goes_below_the_neck_range():
    look = LookDown(step=1.0, min_pos=2.0)
    assert look.update(10.0, 10.0, 2.4) == pytest.approx(2.0)


def test_nothing_to_return_to_means_no_look_down():
    assert LookDown().update(10.0, 10.0, None) is None
