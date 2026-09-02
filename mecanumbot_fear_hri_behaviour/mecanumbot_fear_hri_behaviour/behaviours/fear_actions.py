"""Execute freeze / avoid / catch_attention / task from selected_action."""

from __future__ import annotations

import py_trees
from mecanumbot_movement_behaviours.ros_interfaces import Nav2PoseNavigator, VelocityCommander

from mecanumbot_fear_hri_behaviour.behaviours import keys


class ExecuteSelectedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "ExecuteSelectedAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=keys.SELECTED_ACTION, access=py_trees.common.Access.READ)
        self._nav2: Nav2PoseNavigator | None = None
        self._velocity: VelocityCommander | None = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._nav2 = Nav2PoseNavigator(self.node)
        self._velocity = VelocityCommander(self.node)

    def update(self) -> py_trees.common.Status:
        action = str(self.blackboard.selected_action)
        if action == "freeze":
            self._nav2.cancel()
            self._velocity.stop()
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class LogFearState(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "LogFearState"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.FEAR_RISK, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.MIN_LIDAR_RANGE, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.SELECTED_ACTION, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        self.node.get_logger().info(
            "fear=%.1f risk=%.2f range=%.2f action=%s"
            % (
                float(self.blackboard.fear_level),
                float(self.blackboard.fear_risk),
                float(self.blackboard.min_lidar_range),
                str(self.blackboard.selected_action),
            )
        )
        return py_trees.common.Status.SUCCESS
