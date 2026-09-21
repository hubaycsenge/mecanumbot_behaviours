"""
A replaced nav2 goal must not report on the goal that replaced it.

nav2 aborts a preempted goal, and its result arrives *after* the new goal has
been sent. When the navigator let that late ABORTED land on the new goal, every
search stop after the first was written off within a tick, and every re-aim of
the fetch approach counted as nav2 dropping the goal -- the robot turned towards
each goal and never drove. No ROS graph: the action client is a fake.
"""

import pytest

ros_interfaces = pytest.importorskip("mecanumbot_movement_behaviours.ros_interfaces")

from geometry_msgs.msg import Pose  # noqa: E402


class _Future:
    def __init__(self):
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def finish(self, result):
        self._result = result
        for callback in self._callbacks:
            callback(self)


class _Handle:
    def __init__(self):
        self.accepted = True
        self.result_future = _Future()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        return _Future()


class _Result:
    def __init__(self, status):
        self.status = status


class _Client:
    def __init__(self, node, action_type, action_name):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        future = _Future()
        self.sent.append((future, feedback_callback))
        return future


class _Clock:
    def now(self):
        from rclpy.time import Time

        return Time(nanoseconds=1)


class _Node:
    def get_clock(self):
        return _Clock()

    def get_logger(self):
        class _Logger:
            def warn(self, *_):
                pass

        return _Logger()


@pytest.fixture
def navigator(monkeypatch):
    monkeypatch.setattr(ros_interfaces, "ActionClient", _Client)
    return ros_interfaces.Nav2PoseNavigator(_Node())


def _accept(navigator, index):
    future, _ = navigator._client.sent[index]
    handle = _Handle()
    future.finish(handle)
    return handle


def test_preempted_goals_result_does_not_land_on_the_new_goal(navigator):
    navigator.go_to(Pose())
    first = _accept(navigator, 0)
    navigator.go_to(Pose())
    _accept(navigator, 1)

    first.result_future.finish(_Result(ros_interfaces.STATUS_ABORTED))

    assert navigator.status() == ros_interfaces.STATUS_ACCEPTED


def test_late_response_to_a_replaced_goal_is_ignored(navigator):
    navigator.go_to(Pose())
    navigator.go_to(Pose())
    second = _accept(navigator, 1)
    rejected = _Handle()
    rejected.accepted = False
    navigator._client.sent[0][0].finish(rejected)

    assert navigator.status() == ros_interfaces.STATUS_ACCEPTED
    second.result_future.finish(_Result(ros_interfaces.STATUS_SUCCEEDED))
    assert navigator.status() == ros_interfaces.STATUS_SUCCEEDED


def test_feedback_from_a_replaced_goal_is_ignored(navigator):
    navigator.go_to(Pose())
    _accept(navigator, 0)
    navigator.go_to(Pose())
    navigator._client.sent[0][1](type("Message", (), {"feedback": "old"})())

    assert navigator.feedback is None
    assert navigator.status() is None


def test_the_current_goals_own_result_still_arrives(navigator):
    navigator.go_to(Pose())
    handle = _accept(navigator, 0)
    handle.result_future.finish(_Result(ros_interfaces.STATUS_ABORTED))

    assert navigator.status() == ros_interfaces.STATUS_ABORTED
