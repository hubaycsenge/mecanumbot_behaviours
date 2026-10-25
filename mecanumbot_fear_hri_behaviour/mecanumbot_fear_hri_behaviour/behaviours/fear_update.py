"""
Update fear_level from /mecanumbot/scan.
Select actions via trained RL policy (DQN preferred), not hardcoded thresholds.
"""

from __future__ import annotations

import os

import py_trees

from mecanumbot_fear_hri_behaviour.behaviours import fear_logic, keys
from mecanumbot_fear_hri_behaviour.behaviours.completion_cost import (
    all_route_costs,
    cheapest_route,
    normalized_cost_features,
)
from mecanumbot_fear_hri_behaviour.behaviours.rl_policy import (
    action_to_route,
    catch_attention_allowed,
    get_policy,
    policy_kind,
    select_action_rl,
)
from mecanumbot_fear_hri_behaviour.behaviours.ros_interfaces import LidarRangeTracker


def _catch_attention_enabled(node) -> bool:
    """Csenge: catch_attention is IRL-only — sim subject cannot react."""
    env = os.environ.get("FEAR_CATCH_ATTENTION", "").strip().lower()
    if env in ("1", "true", "on", "irl", "yes"):
        return True
    if env in ("0", "false", "off", "sim", "no"):
        return False
    try:
        if node.has_parameter("use_sim_time") and bool(
            node.get_parameter("use_sim_time").value
        ):
            return False
    except Exception:
        pass
    return catch_attention_allowed()


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
        # Person clearly clear → residual fear must not stall the brick task.
        # (Previously only drained when fear>15, so fear≈7 kept "caution" forever.)
        safe = float(self.blackboard.safe_distance_m)
        person_bb = float(getattr(self.blackboard, "person_range_m", min_range))
        clear = min(float(min_range), person_bb) >= safe
        if clear and fear > 0.0:
            fear = max(0.0, fear - max(3.0, fear * 0.25))
        self.blackboard.fear_level = fear
        return py_trees.common.Status.SUCCESS


class SelectFearAction(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "SelectFearAction"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=keys.FEAR_LEVEL, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.MIN_LIDAR_RANGE, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.PERSON_RANGE_M, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.FEAR_RISK, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.HAS_BRICK, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=keys.TARGET_DIST_M, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.SELECTED_ACTION, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.SELECTED_ROUTE, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.TARGET_Y, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=keys.POLICY_SOURCE, access=py_trees.common.Access.WRITE)
        for param in (
            "hit_distance_m",
            "safe_distance_m",
            "freeze_fear_threshold",
            "avoid_fear_threshold",
            "catch_fear_low",
            "catch_fear_high",
        ):
            self.blackboard.register_key(key=param, access=py_trees.common.Access.READ)
        self._logged_policy = False
        self._robot_xy = (0.0, 0.0)
        self._person_xy = (0.0, -0.20)
        self._person_seen = False
        self._robot_seen = False
        self._gt_person = False  # True once /sim/subject_pose_ground_truth arrived
        self._gt_robot = False
        self._use_sim_person = False
        self._committed_route: str | None = None
        self._commit_has_brick: bool | None = None
        self._route_switch_margin_s = 2.5

    def setup(self, **kwargs):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry

        self.node = kwargs["node"]
        self._catch_enabled = _catch_attention_enabled(self.node)
        # Defaults so first tick does not KeyError on unset optional keys
        self.blackboard.target_y = 0.0
        self.blackboard.target_dist_m = 99.0
        self.blackboard.selected_route = "middle"
        self.blackboard.selected_action = "task_middle"
        self.blackboard.policy_source = "init"
        self._use_sim_person = (
            os.environ.get("FEAR_USE_SIM", "").strip().lower() in ("1", "true", "on", "yes")
            or self._catch_enabled is False
        )
        try:
            if self.node.has_parameter("use_sim_time") and bool(
                self.node.get_parameter("use_sim_time").value
            ):
                self._use_sim_person = True
        except Exception:
            pass
        policy = get_policy()
        if policy is None:
            self.node.get_logger().error(
                "RL policy missing — train fear_mecanumbot/models/hri_dqn.npz "
                "(or set FEAR_DQN_MODEL)"
            )
        else:
            self.node.get_logger().info(
                "Fear action policy: %s (RL — task/route/avoid/freeze)"
                % policy_kind()
            )
        if self._catch_enabled:
            self.node.get_logger().info(
                "catch_attention: enabled (IRL / FEAR_CATCH_ATTENTION=1)"
            )
        else:
            self.node.get_logger().info(
                "catch_attention: disabled in sim (Csenge — IRL-only; no fake human reaction)"
            )
        if self._use_sim_person:
            self.node.get_logger().info(
                "person range: sim ground truth only (ignore /mecanumbot/subject_pose spikes)"
            )

        def on_odom(msg: Odometry) -> None:
            # In sim, odom vs /sim/robot_pose disagree and flicker HRI range → false freeze.
            if self._use_sim_person and self._gt_robot:
                return
            self._robot_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )
            self._robot_seen = True

        def on_robot_pose(msg: PoseStamped) -> None:
            self._robot_xy = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
            )
            self._robot_seen = True
            self._gt_robot = True

        def on_person_gt(msg: PoseStamped) -> None:
            self._person_xy = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
            )
            self._person_seen = True
            self._gt_person = True

        def on_person_est(msg: PoseStamped) -> None:
            # In sim, DR-SPAAM /mecanumbot/subject_pose false-nears caused
            # avoid→wall while the real person was still far. Ignore once GT lives.
            if self._use_sim_person:
                return
            self._person_xy = (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
            )
            self._person_seen = True

        self.node.create_subscription(Odometry, "/mecanumbot/odom", on_odom, 10)
        self.node.create_subscription(PoseStamped, "/sim/robot_pose", on_robot_pose, 10)
        self.node.create_subscription(
            PoseStamped, "/sim/subject_pose_ground_truth", on_person_gt, 10
        )
        self.node.create_subscription(
            PoseStamped, "/mecanumbot/subject_pose", on_person_est, 10
        )

    def _person_range(self) -> float:
        # Until both poses are live, do not invent a near-person freeze (startup glitch)
        if not (self._person_seen and self._robot_seen):
            return max(float(self.blackboard.min_lidar_range), 2.0)
        rx, ry = self._robot_xy
        px, py = self._person_xy
        return float(((px - rx) ** 2 + (py - ry) ** 2) ** 0.5)

    def _bb_float(self, key: str, default: float) -> float:
        try:
            return float(getattr(self.blackboard, key))
        except (KeyError, AttributeError, TypeError, ValueError):
            return float(default)

    def _bb_bool(self, key: str, default: bool = False) -> bool:
        try:
            return bool(getattr(self.blackboard, key))
        except (KeyError, AttributeError, TypeError):
            return bool(default)

    def _target_xy(self) -> tuple[float, float]:
        ty = self._bb_float("target_y", -0.20)
        tx = 1.35 if self._bb_bool("has_brick", False) else -1.35
        return tx, ty

    def _person_offsets(self) -> tuple[float, float]:
        """Continuous person offset relative to robot."""
        rx, ry = self._robot_xy
        px, py = self._person_xy
        return float(px - rx), float(py - ry)

    def _cost_bundle(self) -> tuple[tuple[float, float, float], dict]:
        """Live completion-time costs: robot decides via RL using these features."""
        robot = self._robot_xy
        person = self._person_xy
        target = self._target_xy()
        costs = all_route_costs(robot, target, person)
        norms = normalized_cost_features(robot, target, person)
        return norms, costs

    def update(self) -> py_trees.common.Status:
        has_brick = self._bb_bool("has_brick", False)
        target_dist = self._bb_float("target_dist_m", 99.0)
        person_d = self._person_range()
        hri_range = person_d
        person_dx, person_dy = self._person_offsets()
        (c_m, c_n, c_s), costs = self._cost_bundle()
        self.blackboard.person_range_m = hri_range
        action = select_action_rl(
            fear_level=float(self.blackboard.fear_level),
            min_range=hri_range,
            has_brick=has_brick,
            target_dist_m=target_dist,
            person_dx=person_dx,
            person_dy=person_dy,
            risk=float(self.blackboard.fear_risk),
            cost_middle=c_m,
            cost_north=c_n,
            cost_south=c_s,
            catch_attention_enabled=self._catch_enabled,
        )
        # Safety latch: person caught the robot → freeze (overrides task/avoid)
        try:
            hit = float(self.blackboard.hit_distance_m)
        except (KeyError, AttributeError, TypeError, ValueError):
            hit = 0.28
        freeze_band = max(0.55, hit + 0.27)
        if hri_range <= freeze_band:
            if action != "freeze":
                self.node.get_logger().info(
                    "freeze latch: person=%.2fm ≤ %.2fm (was %s)"
                    % (hri_range, freeze_band, action)
                )
            action = "freeze"
        if action is None:
            # No DQN weights: still decide by completion-time cost (not a fixed lane)
            best = cheapest_route(self._robot_xy, self._target_xy(), self._person_xy)
            action = "task_%s" % best
            self.blackboard.policy_source = "completion_time_cost"
            if not self._logged_policy:
                self._logged_policy = True
                self.node.get_logger().error(
                    "SelectFear: no DQN — using cheapest completion-time route=%s" % best
                )
        else:
            self.blackboard.policy_source = "rl_%s" % policy_kind()
            if not self._logged_policy:
                self._logged_policy = True
                self.node.get_logger().info(
                    "SelectFear: RL %s (completion-time travel+wait features, %s)"
                    % (
                        policy_kind(),
                        "catch_on" if self._catch_enabled else "catch_off",
                    )
                )
        # Reset corridor commit on pick/drop so each leg re-chooses from costs
        if self._commit_has_brick is None or self._commit_has_brick != has_brick:
            self._commit_has_brick = has_brick
            self._committed_route = None
        route = action_to_route(action)
        if action.startswith("task"):
            # Cost-aware commit: stop thrashing; switch only if clearly cheaper
            proposed = route if route in ("middle", "north", "south") else "middle"
            committed = self._committed_route
            best = min(costs, key=lambda n: costs[n][0])
            # If current lane is blocked (high wait), jump to cheapest clear lane
            if committed in ("middle", "north", "south") and costs[committed][2] >= 8.0:
                if costs[best][0] + 1.0 < costs[committed][0]:
                    proposed = best
            elif committed in ("middle", "north", "south") and proposed != committed:
                if costs[proposed][0] + self._route_switch_margin_s >= costs[committed][0]:
                    proposed = committed
            action = "task_%s" % proposed
            self._committed_route = proposed
            route = proposed
        if route is None:
            try:
                route = str(self.blackboard.selected_route or "middle")
            except (KeyError, AttributeError, TypeError):
                route = "middle"
        # Log when RL changes corridor — show why (travel/wait totals)
        prev = getattr(self, "_last_logged_route", None)
        if route != prev and action.startswith("task"):
            self._last_logged_route = route
            best = min(costs, key=lambda n: costs[n][0])
            self.node.get_logger().info(
                "route decision RL=%s best_cost=%s  "
                "middle=%.1fs(t=%.1f+w=%.1f) north=%.1fs(t=%.1f+w=%.1f) "
                "south=%.1fs(t=%.1f+w=%.1f) person=%.2fm carrying=%s"
                % (
                    route,
                    best,
                    costs["middle"][0],
                    costs["middle"][1],
                    costs["middle"][2],
                    costs["north"][0],
                    costs["north"][1],
                    costs["north"][2],
                    costs["south"][0],
                    costs["south"][1],
                    costs["south"][2],
                    hri_range,
                    has_brick,
                )
            )
        self.blackboard.selected_action = action
        self.blackboard.selected_route = route
        return py_trees.common.Status.SUCCESS
