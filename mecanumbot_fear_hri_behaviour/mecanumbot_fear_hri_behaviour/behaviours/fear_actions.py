"""Execute freeze / avoid / catch_attention / task from selected_action.

Aligned with Csenge Hubay's Mecanumbot stack (TECHNICAL_REPORT / INTEGRATION):
- avoid → Nav2 Approach retreat (empty-handed) or reactive clear (while carrying)
- catch_attention → ostensive yaw toward the person (IRL only; remapped in sim)
- task → brick-carry pipeline (full speed; fear does not throttle)
- freeze → stop when person catches robot (≤ ~0.55 m) or RL selects freeze
- post-freeze → brief hold, then ease cmd_vel back in
"""

from __future__ import annotations

import math
import os
import time

import py_trees
from geometry_msgs.msg import Pose, Twist

from mecanumbot_fear_hri_behaviour.behaviours import fear_logic, keys
from mecanumbot_fear_hri_behaviour.behaviours.brick_carry import BrickCarryTask

try:
    from mecanumbot_movement_behaviours.geometry import quaternion_from_yaw
    from mecanumbot_movement_behaviours.ros_interfaces import (
        STATUS_ABORTED,
        STATUS_CANCELED,
        STATUS_CANCELING,
        STATUS_SUCCEEDED,
        Nav2PoseNavigator,
        VelocityCommander,
    )
except ImportError:  # pragma: no cover
    Nav2PoseNavigator = None  # type: ignore
    VelocityCommander = None  # type: ignore
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6
    STATUS_CANCELING = 3

    def quaternion_from_yaw(yaw: float):
        from geometry_msgs.msg import Quaternion

        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


class ExecuteSelectedAction(py_trees.behaviour.Behaviour):
    """Fear actions + brick-carry. Freeze/avoid remain real while carrying."""

    def __init__(self, name: str = "ExecuteSelectedAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=keys.SELECTED_ACTION, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.SELECTED_ROUTE, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.HAS_BRICK, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=keys.TARGET_DIST_M, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=keys.TARGET_Y, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="avoid_distance_m", access=py_trees.common.Access.READ
        )
        self._nav2 = None
        self._velocity = None
        self._cmd_pub = None
        self._brick_task = None
        self._last_avoid_goal = None
        self._last_avoid_send_t = 0.0
        self._test_thaw = os.environ.get("FEAR_TEST_THAW", "").strip().lower() in (
            "1",
            "true",
            "on",
            "yes",
        )
        self._test_freeze_until = 0.0  # wall time; 0 = arm on first update
        self._test_thaw_armed = False

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        if Nav2PoseNavigator is not None:
            self._nav2 = Nav2PoseNavigator(self.node)
        if VelocityCommander is not None:
            self._velocity = VelocityCommander(self.node)
        self._cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self._brick_task = BrickCarryTask(self.node)

    def _publish_twist(
        self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0
    ) -> None:
        s = 1.0
        if self._brick_task is not None:
            s = float(self._brick_task.post_freeze_scale())
        msg = Twist()
        msg.linear.x = float(linear_x) * s
        msg.linear.y = float(linear_y) * s
        msg.angular.z = float(angular_z) * s
        self._cmd_pub.publish(msg)

    def _retreat_pose(self, fear: float) -> Pose | None:
        """Pose away from the person by fear-scaled distance (Csenge Nav2 Approach)."""
        if self._brick_task is None:
            return None
        px, py = self._brick_task._person_xy
        ax, ay = self._brick_task._away_from_person()
        base = float(getattr(self.blackboard, "avoid_distance_m", 0.8))
        retreat = fear_logic.avoid_retreat_distance_m(fear, base=base, scale=0.01)
        goal_x = px + ax * (self._brick_task._person_dist() + retreat)
        goal_y = py + ay * (self._brick_task._person_dist() + retreat)
        goal_x = max(-1.7, min(1.7, goal_x))
        goal_y = max(-1.5, min(1.5, goal_y))
        pose = Pose()
        pose.position.x = float(goal_x)
        pose.position.y = float(goal_y)
        pose.position.z = 0.0
        pose.orientation = quaternion_from_yaw(math.atan2(ay, ax))
        return pose

    def _nav2_dead(self) -> bool:
        if self._nav2 is None or not self._nav2.goal_sent:
            return True
        st = self._nav2.status()
        return st in (
            STATUS_SUCCEEDED,
            STATUS_CANCELED,
            STATUS_ABORTED,
            STATUS_CANCELING,
        )

    def _nav2_avoid(self, fear: float) -> bool:
        """Send Nav2 Approach retreat at most once per 3s (no goal spam)."""
        if self._nav2 is None or not self._nav2.server_ready():
            return False
        pose = self._retreat_pose(fear)
        if pose is None:
            return False
        now = self.node.get_clock().now().nanoseconds / 1e9
        if (now - self._last_avoid_send_t) < 3.0 and self._last_avoid_goal is not None:
            return bool(self._nav2.goal_sent) and not self._nav2_dead()
        key = (round(pose.position.x, 1), round(pose.position.y, 1))
        try:
            self._nav2.go_to(pose, frame_id="map")
            self._last_avoid_goal = key
            self._last_avoid_send_t = now
            return True
        except Exception as exc:  # pragma: no cover
            self.node.get_logger().warn(
                "Nav2 avoid failed (%s) — cmd_vel fallback" % exc
            )
            return False

    def _cancel_nav2(self) -> None:
        if self._nav2 is not None and self._nav2.goal_sent:
            try:
                self._nav2.cancel()
            except Exception:
                pass
        self._last_avoid_goal = None

    def update(self) -> py_trees.common.Status:
        action = str(self.blackboard.selected_action)
        fear = float(self.blackboard.fear_level)
        route = "middle"
        try:
            route = str(self.blackboard.selected_route or "middle")
        except (KeyError, AttributeError, TypeError):
            route = "middle"

        if self._brick_task is not None:
            self._brick_task.fear_level = fear
            self._brick_task.set_rl_route(route)
            self.blackboard.target_dist_m = float(self._brick_task.target_dist_m())
            ty = self._brick_task.target_xy()[1]
            self.blackboard.target_y = float(ty)

        # RL emits task_middle|task_north|task_south — all are brick task
        if action.startswith("task"):
            action = "task"

        # Demo/test: after ~2 s running, force freeze for 1.5 s so thaw is visible
        if self._test_thaw:
            now = time.monotonic()
            if not self._test_thaw_armed:
                self._test_freeze_until = now + 2.0
                self._test_thaw_armed = True
                self.node.get_logger().info(
                    "FEAR_TEST_THAW: will force freeze in 2s for thaw demo"
                )
            elif now < self._test_freeze_until:
                pass  # waiting to start freeze window
            elif now < self._test_freeze_until + 1.5:
                action = "freeze"
            # after that, normal RL actions + post-freeze ramp on cmd_vel

        carrying = self._brick_task is not None and self._brick_task.has_brick
        phase = getattr(self._brick_task, "phase", "done") if self._brick_task else "done"
        in_pipeline = phase not in ("done",)

        if action == "freeze":
            # Real freeze — stop. Keep grippers closed if carrying so the brick stays.
            self._cancel_nav2()
            if self._brick_task is not None:
                self._brick_task.notify_freeze()
            if carrying:
                self._brick_task.ensure_grip_closed()
            if self._velocity is not None:
                self._velocity.stop()
            else:
                self._publish_twist(0.0, 0.0, 0.0)
            if self._brick_task is not None:
                self.blackboard.has_brick = bool(self._brick_task.has_brick)
            return py_trees.common.Status.SUCCESS

        if action == "avoid":
            # Avoid = pause delivery, move away from the person (Csenge).
            # During brick run: reactive retreat (Nav2 would fight /cmd_vel).
            # Idle (done): Nav2 Approach retreat.
            if carrying or in_pipeline:
                self._cancel_nav2()
                if carrying:
                    self._brick_task.ensure_grip_closed()
                self._brick_task.avoid_person(fear)  # pure retreat; task paused
                if self._brick_task is not None:
                    self.blackboard.has_brick = bool(self._brick_task.has_brick)
                return py_trees.common.Status.SUCCESS

            used_nav2 = self._nav2_avoid(fear)
            if not used_nav2 and self._brick_task is not None:
                self._brick_task.avoid_person(fear)
            if self._brick_task is not None:
                self.blackboard.has_brick = bool(self._brick_task.has_brick)
            return py_trees.common.Status.SUCCESS

        if action == "catch_attention":
            self._cancel_nav2()
            # While carrying: keep the brick pipeline moving (brief social cue otherwise)
            if carrying:
                has_brick, _label = self._brick_task.tick()
                self.blackboard.has_brick = bool(has_brick)
                return py_trees.common.Status.SUCCESS
            if self._brick_task is not None:
                rx, ry = self._brick_task._robot_xy
                px, py = self._brick_task._person_xy
                desired = math.atan2(py - ry, px - rx)
                err = (desired - self._brick_task._robot_yaw + math.pi) % (
                    2.0 * math.pi
                ) - math.pi
                self._publish_twist(0.0, 0.0, 1.2 * err)
                self.blackboard.has_brick = bool(self._brick_task.has_brick)
            else:
                self._publish_twist(0.0, 0.0, 0.55 * math.sin(fear))
            return py_trees.common.Status.SUCCESS

        # task — reactive brick_carry owns /cmd_vel
        self._cancel_nav2()
        if self._brick_task is None:
            return py_trees.common.Status.SUCCESS

        has_brick, label = self._brick_task.tick()
        self.blackboard.has_brick = bool(has_brick)
        prev = getattr(self, "_last_task_label", "")
        now_m = time.monotonic()
        last_log = getattr(self, "_last_task_log_t", 0.0)
        # Log phase changes immediately; also heartbeat every 2s so "done"/idle is visible
        if label != prev or (now_m - last_log) > 2.0:
            self._last_task_label = label
            self._last_task_log_t = now_m
            self.node.get_logger().info(
                "%s has_brick=%s delivered=%d/%d cubes=%d"
                % (
                    label,
                    has_brick,
                    self._brick_task._delivered,
                    4,
                    len(getattr(self._brick_task, "_cubes", []) or []),
                )
            )
        return py_trees.common.Status.SUCCESS


class LogFearState(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "LogFearState"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.FEAR_RISK, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.MIN_LIDAR_RANGE, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.SELECTED_ACTION, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.HAS_BRICK, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.POLICY_SOURCE, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=keys.PERSON_RANGE_M, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        retreat = fear_logic.avoid_retreat_distance_m(float(self.blackboard.fear_level))
        has_brick = bool(getattr(self.blackboard, "has_brick", False))
        policy = str(getattr(self.blackboard, "policy_source", "?"))
        self.node.get_logger().info(
            "fear=%.1f risk=%.2f range=%.2f person=%.2f action=%s policy=%s avoid_retreat=%.2fm has_brick=%s"
            % (
                float(self.blackboard.fear_level),
                float(self.blackboard.fear_risk),
                float(self.blackboard.min_lidar_range),
                float(getattr(self.blackboard, "person_range_m", -1.0)),
                str(self.blackboard.selected_action),
                policy,
                retreat,
                has_brick,
            )
        )
        return py_trees.common.Status.SUCCESS
