"""
Update fear_level from /mecanumbot/scan.
"""

from __future__ import annotations

import py_trees

from mecanumbot_fear_hri_behaviour.behaviours import fear_logic, keys
from mecanumbot_fear_hri_behaviour.behaviours.ros_interfaces import LidarRangeTracker


class UpdateFearFromScan(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "UpdateFearFromScan"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=keys.MIN_LIDAR_RANGE, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.FEAR_LEVEL, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.FEAR_RISK, access=py_trees.common.Access.WRITE)
        for param in (
            "safe_distance_m",
            "hit_distance_m",
            "fear_decay_per_step",
            "fear_spike_on_event",
            "fear_rise_per_risk",
        ):
            self.blackboard.register_key(key=param, access=py_trees.common.Access.READ)
        self._scan: LidarRangeTracker | None = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._scan = LidarRangeTracker(self.node)

    def update(self) -> py_trees.common.Status:
        min_range = self._scan.min_range if self._scan else float(self.blackboard.min_lidar_range)
        closing = self._scan.closing_speed if self._scan else 0.0
        self.blackboard.min_lidar_range = min_range

        risk = fear_logic.compute_risk(
            min_range=min_range,
            safe_distance=float(self.blackboard.safe_distance_m),
            hit_distance=float(self.blackboard.hit_distance_m),
            closing_speed=closing,
        )
        near = fear_logic.is_near_collision(min_range, float(self.blackboard.hit_distance_m))
        fear = fear_logic.update_fear_level(
            float(self.blackboard.fear_level),
            risk=risk,
            spike_event=near,
            decay_per_step=float(self.blackboard.fear_decay_per_step),
            spike_on_event=float(self.blackboard.fear_spike_on_event),
            rise_per_risk=float(self.blackboard.fear_rise_per_risk),
        )
        self.blackboard.fear_risk = risk
        self.blackboard.fear_level = fear
        return py_trees.common.Status.SUCCESS


class SelectFearAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "SelectFearAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.MIN_LIDAR_RANGE, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.FEAR_RISK, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.SELECTED_ACTION, access=py_trees.common.Access.WRITE)
        for param in (
            "hit_distance_m",
            "safe_distance_m",
            "freeze_fear_threshold",
            "avoid_fear_threshold",
            "catch_fear_low",
            "catch_fear_high",
        ):
            self.blackboard.register_key(key=param, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        self.blackboard.selected_action = fear_logic.select_action(
            fear_level=float(self.blackboard.fear_level),
            min_range=float(self.blackboard.min_lidar_range),
            risk=float(self.blackboard.fear_risk),
            hit_distance=float(self.blackboard.hit_distance_m),
            safe_distance=float(self.blackboard.safe_distance_m),
            freeze_threshold=float(self.blackboard.freeze_fear_threshold),
            avoid_threshold=float(self.blackboard.avoid_fear_threshold),
            catch_low=float(self.blackboard.catch_fear_low),
            catch_high=float(self.blackboard.catch_fear_high),
        )
        return py_trees.common.Status.SUCCESS
