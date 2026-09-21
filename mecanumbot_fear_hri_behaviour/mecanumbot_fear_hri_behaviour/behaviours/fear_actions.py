"""Execute freeze / avoid / catch_attention / task from selected_action."""

from __future__ import annotations

import math

import py_trees
from geometry_msgs.msg import Twist

from mecanumbot_fear_hri_behaviour.behaviours import fear_logic, keys

try:
    from mecanumbot_movement_behaviours.ros_interfaces import (
        Nav2PoseNavigator,
        VelocityCommander,
    )
except ImportError:  # pragma: no cover - allows unit checks without full stack
    Nav2PoseNavigator = None  # type: ignore
    VelocityCommander = None  # type: ignore


class ExecuteSelectedAction(py_trees.behaviour.Behaviour):
    """Apply the selected Fear HRI action once per tick and return SUCCESS.

    Returning SUCCESS (not RUNNING) keeps the fear update / log loop alive so
    ``fear_level`` visibly changes in simulation even when avoid or task is
    selected. Full Nav2 Approach retreat can be layered later; V1 activation
    uses ``/cmd_vel`` stop and a short reverse for avoid.
    """

    def __init__(self, name: str = "ExecuteSelectedAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=keys.SELECTED_ACTION, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ)
        self._nav2 = None
        self._velocity = None
        self._cmd_pub = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        if Nav2PoseNavigator is not None:
            self._nav2 = Nav2PoseNavigator(self.node)
        if VelocityCommander is not None:
            self._velocity = VelocityCommander(self.node)
        self._cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def _publish_twist(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def update(self) -> py_trees.common.Status:
        action = str(self.blackboard.selected_action)
        fear = float(self.blackboard.fear_level)

        if action == "freeze":
            if self._nav2 is not None:
                self._nav2.cancel()
            if self._velocity is not None:
                self._velocity.stop()
            else:
                self._publish_twist(0.0, 0.0)
            return py_trees.common.Status.SUCCESS

        if action == "avoid":
            # Fear-scaled reverse on cmd_vel (Nav2 Approach can replace this).
            speed = -min(0.35, 0.12 + 0.002 * fear)
            if self._nav2 is not None:
                self._nav2.cancel()
            self._publish_twist(speed, 0.0)
            return py_trees.common.Status.SUCCESS

        if action == "catch_attention":
            # In-place yaw oscillation cue.
            self._publish_twist(0.0, 0.6 * math.sin(fear))
            return py_trees.common.Status.SUCCESS

        # task: leave velocity free for other stacks / teleop; do not hold RUNNING
        return py_trees.common.Status.SUCCESS


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
        retreat = fear_logic.avoid_retreat_distance_m(float(self.blackboard.fear_level))
        self.node.get_logger().info(
            "fear=%.1f risk=%.2f range=%.2f action=%s avoid_retreat=%.2fm"
            % (
                float(self.blackboard.fear_level),
                float(self.blackboard.fear_risk),
                float(self.blackboard.min_lidar_range),
                str(self.blackboard.selected_action),
                retreat,
            )
        )
        return py_trees.common.Status.SUCCESS
