"""
Which way a frontier goal asks the robot to end up facing.

nav2 has no way to be told "any orientation will do", so this is not a
cosmetic detail: whatever quaternion goes into the goal is a heading the robot
will turn in place to reach, and turning in place is the motion a T1 pass pays
most for -- the lidar sweeps for 100 ms undeskewed and mecanum rollers slip
under yaw, so a spin is where the map is most likely to be handed a smeared
scan and a confidently wrong odometry prior at once.

The bug these guard against was an identity quaternion in a `map`-framed goal,
which is map yaw 0 rather than "no preference": every leg ended with a turn to
face map-east.

`_bearing` is a pure function; the goal-sending path around it needs `rclpy`
and is skipped without it.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

driving = pytest.importorskip("mecanumbot_autoslam.behaviours.driving")


def test_the_goal_faces_the_frontier():
    assert driving._bearing((1.0, 1.0), (3.0, 1.0)) == pytest.approx(0.0)
    assert driving._bearing((1.0, 1.0), (1.0, 4.0)) == pytest.approx(math.pi / 2)
    assert driving._bearing((1.0, 1.0), (0.0, 1.0)) == pytest.approx(math.pi)


def test_no_pose_means_no_heading_rather_than_map_east():
    assert driving._bearing(None, (3.0, 1.0)) is None


def test_standing_on_the_frontier_means_no_heading():
    # atan2(0, 0) is 0.0, which is the map-east heading this exists to avoid.
    assert driving._bearing((2.0, 2.0), (2.0, 2.0)) is None
    assert driving._bearing((2.0, 2.0), (2.0 + 1e-9, 2.0)) is None
