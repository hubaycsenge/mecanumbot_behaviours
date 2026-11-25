"""WaitForPerson: the recovery search ends only on somebody seen again, not on a stale detection."""

import types

import py_trees
import pytest

routes = pytest.importorskip("mecanumbot_movement_behaviours.routes")


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns


class FakePeople:
    """A PeopleTracker with `sight_timeout` 20 s: anything younger than that is 'fresh'."""

    def __init__(self, stamp_ns=None, age=None):
        self.last_seen_time = None if stamp_ns is None else FakeTime(stamp_ns)
        self.age = age
        self.last_seen_pose = None

    def has_fresh_detection(self):
        return self.age is not None and self.age <= 20.0

    def see(self, stamp_ns):
        self.last_seen_time, self.age = FakeTime(stamp_ns), 0.1


def waiter(people):
    py_trees.blackboard.Blackboard.enable_activity_stream()
    wait = routes.WaitForPerson(name="TestWait", sight_timeout=20.0, look_down=False)
    wait.people = people
    wait.look_down = None
    wait.velocity = types.SimpleNamespace(stop=lambda: None)
    wait.accessories = types.SimpleNamespace(look=lambda where: None)
    logger = types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    wait.node = types.SimpleNamespace(get_logger=lambda: logger)
    wait._remember_search_direction = lambda: None
    wait.initialise()
    return wait


RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS


def test_a_detection_from_before_the_search_does_not_end_it():
    # last seen 8 s ago -- 'fresh' by the 20 s sight_timeout, but not in view
    wait = waiter(FakePeople(stamp_ns=1_000, age=8.0))
    assert wait.update() == RUNNING


def test_a_new_detection_ends_it():
    people = FakePeople(stamp_ns=1_000, age=8.0)
    wait = waiter(people)
    people.see(2_000)
    assert wait.update() == SUCCESS


def test_somebody_in_view_at_the_start_still_ends_it_at_once():
    wait = waiter(FakePeople(stamp_ns=1_000, age=0.3))
    assert wait.update() == SUCCESS


def test_never_seen_waits():
    wait = waiter(FakePeople())
    assert wait.update() == RUNNING
