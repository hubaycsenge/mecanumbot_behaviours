#!/usr/bin/env python3
"""
Unit tests for the detector gate in front of the first tick.

DR-SPAAM takes about seven seconds to load torch and warm CUDA up. Until
2026-09-23 the tree ticked straight through that, so a leading run opened by
asking whether anybody was there, waiting three seconds, and concluding that
nobody was -- all before the detector had published anything at all.

No ROS graph: `rclpy.spin_once` is replaced and the node is a stand-in that
records what was subscribed to, so this runs on a development machine.
"""


import pytest

from mecanumbot_bt_config import tree_runner
from mecanumbot_bt_config.tree_runner import PERCEPTION_TOPIC, wait_for_perception


class FakeNode:
    """Records subscriptions, and delivers messages after a set number of spins."""

    def __init__(self, deliver_after=None):
        self.deliver_after = deliver_after
        self.spins = 0
        self.subscriptions = []
        self.destroyed = []
        self._callback = None

    def create_subscription(self, msg_type, topic, callback, qos):
        handle = (msg_type, topic, qos)
        self.subscriptions.append(handle)
        self._callback = callback
        return handle

    def destroy_subscription(self, handle):
        self.destroyed.append(handle)

    def spin_once(self):
        self.spins += 1
        if self.deliver_after is not None and self.spins >= self.deliver_after:
            self._callback(object())


@pytest.fixture
def fake_spin(monkeypatch):
    """Point `rclpy.spin_once` at whichever FakeNode it is handed."""

    def _spin_once(node, timeout_sec=None):
        node.spin_once()

    monkeypatch.setattr(tree_runner.rclpy, "spin_once", _spin_once)


class TestWaiting:
    def test_it_waits_for_the_detector_and_then_returns(self, fake_spin):
        node = FakeNode(deliver_after=3)
        assert wait_for_perception(node, timeout=5.0) is True
        assert node.spins == 3

    def test_it_subscribes_to_the_detector_topic(self, fake_spin):
        node = FakeNode(deliver_after=1)
        wait_for_perception(node, timeout=5.0)
        assert node.subscriptions[0][1] == PERCEPTION_TOPIC

    def test_the_topic_is_the_one_that_publishes_when_nobody_is_there(self):
        # `dets` is published on every scan, empty or not, so its first message
        # means "the detector is up". `people_fusion` is only published when
        # somebody is being tracked, so waiting on it would hang in an empty
        # room -- which is exactly the situation a trial starts in.
        assert PERCEPTION_TOPIC.endswith("/dets")
        assert "people_fusion" not in PERCEPTION_TOPIC

    def test_it_gives_up_rather_than_blocking_forever(self, fake_spin):
        # A run with `use_perception:=false` is legitimate; the tree still has
        # to start, and the caller warns.
        node = FakeNode(deliver_after=None)
        assert wait_for_perception(node, timeout=0.3) is False
        assert node.spins > 0

    def test_the_subscription_is_always_cleaned_up(self, fake_spin):
        node = FakeNode(deliver_after=None)
        wait_for_perception(node, timeout=0.2)
        assert node.destroyed == node.subscriptions

    def test_zero_does_not_wait_at_all(self, fake_spin):
        # The old behaviour, kept reachable so a run can opt out of the gate.
        node = FakeNode(deliver_after=None)
        assert wait_for_perception(node, timeout=0.0) is True
        assert node.spins == 0
        assert node.subscriptions == []

    def test_a_negative_timeout_does_not_wait_either(self, fake_spin):
        node = FakeNode(deliver_after=None)
        assert wait_for_perception(node, timeout=-1.0) is True
        assert node.subscriptions == []


class TestDefaults:
    def test_the_wait_is_on_by_default(self):
        assert tree_runner.RUNTIME_DEFAULTS["perception_wait_timeout"] > 0.0

    def test_the_default_outlasts_the_detector_start_up(self):
        # DR-SPAAM took ~7 s from launch to "detector node started" in the runs
        # of 2026-09-23; the default has to clear that with room to spare.
        assert tree_runner.RUNTIME_DEFAULTS["perception_wait_timeout"] >= 15.0

    def test_it_can_be_set_from_the_constants_file(self):
        value = tree_runner.RUNTIME.file_constant(
            {"perception_wait_timeout": 3.0}, "perception_wait_timeout"
        )
        assert float(value) == 3.0
