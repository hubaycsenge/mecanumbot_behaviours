"""Brick-carry task: pick all start-area bricks, detour around the person, deliver."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist
from mecanumbot_msgs.msg import AccessMotorCmd, SimActorArray
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

GOAL_XY = (1.35, -0.20)  # yellow mat centre (from scenario layout)
START_XY = (-1.35, -0.20)  # green mat centre (from scenario layout)

GRABBER_REACH_M = 0.18
GRAB_NEAR_M = 0.30
GOAL_DIST_M = 0.18
GOAL_MIN_X = 1.18
PERSON_CLEAR_M = 1.20
# Speed is NOT a single fixed cruise: base caps, live scale from clearance
DRIVE_SPEED_MAX = 0.42
DRIVE_SPEED_MIN = 0.16
DRIVE_SPEED = 0.34
TURN_GAIN = 1.5
GRIP_OPEN = (7.05, 3.19)
GRIP_CLOSE = (3.19, 7.05)
NECK_LEVEL = 6.0
EXPECTED_BRICKS = 4
POST_FREEZE_HOLD_S = 0.20
POST_FREEZE_RAMP_S = 1.25
POST_FREEZE_MIN_SCALE = 0.12
# Furniture AABBs for low-level collision repel only (not route policy)
ARENA_OBSTACLES = (
    (0.9, 0.4, 0.18, 0.18),
    (-0.7, -0.8, 0.12, 0.25),
)
ROBOT_RADIUS_M = 0.22
FURNITURE_MARGIN_M = ROBOT_RADIUS_M + 0.06
# Controller via geometry (executes RL corridor; does not choose it)
ROUTE_LATERAL_M = 0.90
ROUTE_PEEL_PAST_M = 0.75
GOAL_STUCK_TICKS = 45
RELEASE_RY_LO = -0.55
RELEASE_RY_HI = 0.45  # allow approach from north corridor onto mat


class BrickCarryTask:
    """Approach → grab → reactive navigate to goal → release → return; all bricks."""

    def __init__(self, node) -> None:
        self.node = node
        self.phase = "seek_cube"
        self.has_brick = False
        self._robot_xy = (-1.35, -0.20)
        self._robot_yaw = 0.0
        self._person_xy = (0.0, -0.20)
        self._cubes: list[tuple[str, float, float]] = []
        self._props_seen = False
        self._hold_ticks = 0
        self._delivered = 0
        self._delivered_names: set[str] = set()
        self._carried_name: Optional[str] = None
        self._cube_xy: dict[str, tuple[float, float]] = {}
        self._avoid_side = 0.0  # online: which tangential side clears better
        self.fear_level = 0.0  # tracked for RL/avoid retreat; does not throttle task speed
        self._stuck_ticks = 0
        self._last_progress_dist = 99.0
        self._grip_cmd_open: Optional[bool] = None  # latch — stop servo spam
        self._last_freeze_t: Optional[float] = None  # wall time of last freeze tick
        self._thaw_logged = False
        self._route: Optional[str] = None  # from RL: "middle" | "north" | "south"
        self._rl_route: Optional[str] = None  # latest SelectFear route (no hard lock)
        self._last_goal_progress = 99.0
        self._goal_stuck_ticks = 0
        self._escape_ticks = 0
        self._release_open = False
        self._delivered_at_release_start = 0
        self._wait_props_ticks = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._cmd = node.create_publisher(Twist, "/cmd_vel", 10)
        self._acc = node.create_publisher(AccessMotorCmd, "/cmd_accessory_pos", 10)
        node.create_subscription(SimActorArray, "/sim/props", self._on_props, qos)
        node.create_subscription(Odometry, "/mecanumbot/odom", self._on_odom, qos)
        node.create_subscription(Odometry, "/odom", self._on_odom, qos)
        node.create_subscription(PoseStamped, "/sim/robot_pose", self._on_robot_pose, qos)
        self._gt_person = False
        self._gt_robot = False
        node.create_subscription(
            PoseStamped, "/sim/subject_pose_ground_truth", self._on_person_gt, qos
        )
        node.create_subscription(
            PoseStamped, "/mecanumbot/subject_pose", self._on_person_est, qos
        )

    # --- callbacks ----------------------------------------------------------
    def _on_person_gt(self, msg: PoseStamped) -> None:
        self._gt_person = True
        self._person_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_person_est(self, msg: PoseStamped) -> None:
        if self._gt_person:
            return
        self._person_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        self._gt_robot = True
        self._robot_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        q = msg.pose.orientation
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_odom(self, msg: Odometry) -> None:
        # Prefer /sim/robot_pose when available — odom flicker broke person range
        if self._gt_robot:
            return
        self._robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        q = msg.pose.pose.orientation
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_props(self, msg: SimActorArray) -> None:
        self._props_seen = True
        cubes = []
        at_goal = 0
        # Rebuild delivered from ground truth each tick (don't keep stale names)
        on_goal_names: set[str] = set()
        for actor in msg.actors:
            kind = (actor.kind or "").lower()
            name = actor.name or f"cube_{actor.id}"
            if kind != "cube" and "brick" not in name.lower():
                continue
            x = float(actor.pose.position.x)
            y = float(actor.pose.position.y)
            self._cube_xy[name] = (x, y)
            # Yellow goal mat — do not count the cube while still transporting it
            # (seek_goal). During release the drop must be allowed to count.
            carrying_this = (
                self.has_brick
                and self.phase == "seek_goal"
                and (name == self._carried_name)
            )
            if 1.05 <= x <= 1.65 and -0.45 <= y <= 0.10 and not carrying_this:
                at_goal += 1
                on_goal_names.add(name)
                continue
            if name in on_goal_names:
                continue
            cubes.append((name, x, y))
        self._cubes = cubes
        # Keep already-confirmed deliveries; props oracle adds new ones on the mat.
        # Do not wipe mid-release — that raced with grip-open and caused miss loops.
        if self.phase == "release" and self.has_brick:
            self._delivered_names = set(self._delivered_names) | on_goal_names
        else:
            self._delivered_names = on_goal_names
        self._delivered = len(self._delivered_names)


    # --- motion -------------------------------------------------------------
    def _set_grip(self, open_: bool) -> None:
        """Publish grip only on change. Spam made MuJoCo grabbers jitter (and
        would hunt servos on the real OpenCR) — not related to catch_attention."""
        if self._grip_cmd_open is not None and self._grip_cmd_open == bool(open_):
            return
        self._grip_cmd_open = bool(open_)
        cmd = AccessMotorCmd()
        cmd.n_pos = NECK_LEVEL
        cmd.gl_pos, cmd.gr_pos = GRIP_OPEN if open_ else GRIP_CLOSE
        self._acc.publish(cmd)

    def ensure_grip_closed(self) -> None:
        """Idempotent close while carrying / freeze / avoid."""
        self._set_grip(False)

    def notify_freeze(self) -> None:
        """Called every freeze tick — recovery ramp starts when freeze ends."""
        self._last_freeze_t = self.node.get_clock().now().nanoseconds / 1e9
        self._thaw_logged = False

    def post_freeze_scale(self) -> float:
        """0 → hold, then ramp toward 1 after leaving freeze (HRI recovery)."""
        # Person far → full motion immediately (residual fear must not pin cmd_vel)
        if self._person_dist() >= 1.0:
            self._last_freeze_t = None
            return 1.0
        if self._last_freeze_t is None:
            return 1.0
        now = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._last_freeze_t
        if elapsed < 0.0 or elapsed > (POST_FREEZE_HOLD_S + POST_FREEZE_RAMP_S + 2.0):
            self._last_freeze_t = None
            return 1.0
        if elapsed < POST_FREEZE_HOLD_S:
            return 0.0
        t = (elapsed - POST_FREEZE_HOLD_S) / max(1e-3, POST_FREEZE_RAMP_S)
        if t >= 1.0:
            self._last_freeze_t = None
            return 1.0
        ramp = POST_FREEZE_MIN_SCALE + (1.0 - POST_FREEZE_MIN_SCALE) * t
        if not self._thaw_logged and ramp > 0.05:
            self._thaw_logged = True
            self.node.get_logger().info(
                "post-freeze thaw: easing motion back in (scale=%.2f)" % ramp
            )
        return float(max(0.0, min(1.0, ramp)))

    def _publish(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> None:
        s = self.post_freeze_scale()
        msg = Twist()
        msg.linear.x = float(vx) * s
        msg.linear.y = float(vy) * s
        msg.angular.z = float(wz) * s
        self._cmd.publish(msg)

    def _person_dist(self) -> float:
        rx, ry = self._robot_xy
        px, py = self._person_xy
        return math.hypot(px - rx, py - ry)

    def _away_from_person(self) -> tuple[float, float]:
        rx, ry = self._robot_xy
        px, py = self._person_xy
        dx, dy = rx - px, ry - py
        n = math.hypot(dx, dy) or 1.0
        return dx / n, dy / n

    def _person_blocks_path(self, tx: float, ty: float, corridor: float = 0.55) -> bool:
        """True if the person lies near the robot→target segment (local-min trap)."""
        rx, ry = self._robot_xy
        px, py = self._person_xy
        dx, dy = tx - rx, ty - ry
        seg = math.hypot(dx, dy)
        if seg < 1e-3:
            return False
        t = ((px - rx) * dx + (py - ry) * dy) / (seg * seg)
        if t < 0.02 or t > 0.98:
            return False
        closest_x = rx + t * dx
        closest_y = ry + t * dy
        return math.hypot(px - closest_x, py - closest_y) < corridor

    def _clamp_arena(self, x: float, y: float) -> tuple[float, float]:
        return (
            max(-1.45, min(1.45, x)),
            max(-1.15, min(1.15, y)),
        )

    def _furniture_clearance(self, x: float, y: float) -> float:
        """Signed clearance to nearest furniture AABB (+ outside, − penetrating)."""
        best = 99.0
        for ox, oy, hx, hy in ARENA_OBSTACLES:
            dx = abs(x - ox) - (hx + FURNITURE_MARGIN_M)
            dy = abs(y - oy) - (hy + FURNITURE_MARGIN_M)
            if dx > 0.0 and dy > 0.0:
                clear = math.hypot(dx, dy)
            elif dx > 0.0:
                clear = dx
            elif dy > 0.0:
                clear = dy
            else:
                clear = min(dx, dy)
            if clear < best:
                best = clear
        return best

    def _furniture_repel(self, speed: float) -> tuple[float, float]:
        """Low-level push away from furniture boxes (safety, not route policy)."""
        rx, ry = self._robot_xy
        fx, fy = 0.0, 0.0
        for ox, oy, hx, hy in ARENA_OBSTACLES:
            ehx = hx + FURNITURE_MARGIN_M
            ehy = hy + FURNITURE_MARGIN_M
            dx = rx - ox
            dy = ry - oy
            px = ehx - abs(dx)
            py = ehy - abs(dy)
            if px <= 0.0 or py <= 0.0:
                continue
            strength = speed * 1.35 * max(px / ehx, py / ehy)
            if px < py:
                fx += math.copysign(strength, dx if abs(dx) > 1e-4 else 1.0)
            else:
                fy += math.copysign(strength, dy if abs(dy) > 1e-4 else 1.0)
            n = math.hypot(dx, dy) or 1.0
            fx += (dx / n) * strength * 0.35
            fy += (dy / n) * strength * 0.35
        if self._route in ("north", "south"):
            _px, ppy = self._person_xy
            _rx, rry = self._robot_xy
            away_y = rry - ppy
            if abs(away_y) > 1e-3 and fy * away_y < 0.0:
                fy = 0.0
        return fx, fy

    def _route_y(self, route: str, tx: float, ty: float) -> float:
        """Fixed arena corridors (match completion_cost lanes — not person-relative)."""
        del tx
        mid_y = 0.5 * (self._robot_xy[1] + ty)
        if route == "middle":
            return mid_y
        if route == "north":
            return self._clamp_arena(0.0, 0.90)[1]
        if route == "south":
            return self._clamp_arena(0.0, -0.90)[1]
        return mid_y

    def set_rl_route(self, route: Optional[str]) -> None:
        """Corridor from RL SelectFear (task_middle|north|south)."""
        if route in ("middle", "north", "south"):
            self._rl_route = route

    def target_xy(self) -> tuple[float, float]:
        return self._task_target_xy()

    def _choose_route(self, tx: float, ty: float) -> str:
        """Follow RL-selected corridor; controller only executes vias."""
        del tx, ty
        best = self._rl_route if self._rl_route in ("middle", "north", "south") else "middle"
        if self._route != best:
            prev = self._route
            self._route = best
            self.node.get_logger().info(
                "route RL: %s → %s (person=%.2fm carrying=%s)"
                % (prev, best, self._person_dist(), self.has_brick)
            )
        return best

    def _reactive_via(self, tx: float, ty: float) -> tuple[float, float]:
        """Waypoint detour: corridor entry → pass person in X → peel to target."""
        route = self._choose_route(tx, ty)
        if route == "middle":
            return tx, ty
        rx, ry = self._robot_xy
        px, py = self._person_xy
        cy = self._route_y(route, tx, ty)
        sign = 1.0 if tx >= rx else -1.0
        entry = self._clamp_arena(px - sign * 0.55, cy)
        exit_wp = self._clamp_arena(px + sign * 1.05, cy)

        before_person = (px - rx) * sign > 0.15
        if before_person and abs(ry - cy) > 0.22:
            return entry
        past_person = (rx - px) * sign > ROUTE_PEEL_PAST_M
        if not past_person:
            ahead_x = rx + sign * max(0.55, min(1.10, abs(tx - rx) * 0.45))
            return self._clamp_arena(ahead_x, cy)
        if math.hypot(rx - exit_wp[0], ry - exit_wp[1]) > 0.40 and abs(rx - tx) > 0.55:
            blend_y = 0.55 * cy + 0.45 * ty
            return self._clamp_arena(exit_wp[0], blend_y)
        return tx, ty

    def _task_speed(self, requested: float = DRIVE_SPEED) -> float:
        """Live speed from clearance — not a fixed cruise."""
        pd = self._person_dist()
        if pd >= 1.50:
            person_scale = 1.0
        elif pd <= 0.45:
            person_scale = 0.45
        else:
            person_scale = 0.45 + 0.55 * ((pd - 0.45) / (1.50 - 0.45))

        fc = self._furniture_clearance(self._robot_xy[0], self._robot_xy[1])
        if fc >= 0.35:
            furn_scale = 1.0
        elif fc <= 0.0:
            furn_scale = 0.40
        else:
            furn_scale = 0.40 + 0.60 * (fc / 0.35)

        route_boost = 1.12 if self._route in ("north", "south") else 1.0
        speed = float(requested) * min(person_scale, furn_scale) * route_boost
        return float(max(DRIVE_SPEED_MIN, min(DRIVE_SPEED_MAX, speed)))

    def _drive_routed(
        self, tx: float, ty: float, speed: float = DRIVE_SPEED
    ) -> float:
        """Drive to target via online middle/north/south choice (all travel phases)."""
        via_x, via_y = self._reactive_via(tx, ty)
        follow_side = self._route in ("north", "south")
        return self._drive_toward(
            via_x,
            via_y,
            speed=self._task_speed(speed),
            clear_person=not follow_side,
        )

    def _drive_toward(
        self, tx: float, ty: float, speed: float = DRIVE_SPEED, clear_person: bool = True
    ) -> float:
        """Drive toward target; escape potential-field local minima around the person."""
        rx, ry = self._robot_xy
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self._publish(0.0, 0.0, 0.0)
            return 0.0

        if dist < self._last_progress_dist - 0.03:
            self._stuck_ticks = 0
            self._last_progress_dist = dist
        else:
            self._stuck_ticks += 1
            if self._stuck_ticks > 25:
                self._avoid_side = -self._avoid_side if self._avoid_side != 0 else 1.0
                self._stuck_ticks = 0

        c, s = math.cos(self._robot_yaw), math.sin(self._robot_yaw)
        # Fear does NOT slow the robot — fear only selects freeze/avoid/task via RL.
        # Full requested task speed; clearance still handled by person/furniture repel.
        scale = float(speed) * max(0.4, min(1.0, dist / 0.7))
        to_goal_x, to_goal_y = dx / dist, dy / dist
        wx = to_goal_x * scale
        wy = to_goal_y * scale

        if clear_person:
            pd = self._person_dist()
            px, py = self._person_xy
            left = (-to_goal_y, to_goal_x)
            right = (to_goal_y, -to_goal_x)

            def score(side: tuple[float, float]) -> float:
                nx, ny = rx + side[0], ry + side[1]
                return math.hypot(nx - px, ny - py)

            if self._avoid_side > 0:
                sx, sy = left
            elif self._avoid_side < 0:
                sx, sy = right
            elif score(left) >= score(right):
                sx, sy = left
                self._avoid_side = 1.0
            else:
                sx, sy = right
                self._avoid_side = -1.0

            blocked = self._person_blocks_path(tx, ty)
            if blocked or pd < PERSON_CLEAR_M:
                ax, ay = self._away_from_person()
                if blocked:
                    # Strong lateral detour, keep some goal progress (task-first)
                    lat = 1.05 * speed
                    wx = to_goal_x * scale * 0.35 + sx * lat + ax * 0.30
                    wy = to_goal_y * scale * 0.35 + sy * lat + ay * 0.30
                else:
                    push = (PERSON_CLEAR_M - pd) / PERSON_CLEAR_M
                    wx += (ax * 0.55 + sx * 0.95) * push
                    wy += (ay * 0.55 + sy * 0.95) * push

        # Furniture clearance ALWAYS runs (was wrongly gated behind clear_person,
        # so north/south routes drove straight into the green/orange boxes).
        fxx, fyy = self._furniture_repel(speed)
        wx += fxx
        wy += fyy

        n = math.hypot(wx, wy) or 1.0
        max_s = speed * 1.25
        if n > max_s:
            wx, wy = wx / n * max_s, wy / n * max_s

        desired = math.atan2(wy, wx)
        err = (desired - self._robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(err) > 1.1:
            wx *= 0.45
            wy *= 0.45
        vx = c * wx + s * wy
        vy = -s * wx + c * wy
        # Soft-lock escape: if jammed in a corner empty-handed, pull to arena centre
        # then toward the task target (bricks / start).
        if not self.has_brick and (rx > 1.20 or rx < -1.40 or abs(ry) > 1.05):
            to_c_x, to_c_y = -rx, -ry
            cn = math.hypot(to_c_x, to_c_y) or 1.0
            to_c_x, to_c_y = to_c_x / cn, to_c_y / cn
            wx = to_c_x * speed * 0.95 + to_goal_x * speed * 0.35
            wy = to_c_y * speed * 0.95 + to_goal_y * speed * 0.35
            n2 = math.hypot(wx, wy) or 1.0
            wx, wy = wx / n2 * speed, wy / n2 * speed
            desired = math.atan2(wy, wx)
            err = (desired - self._robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
            vx = c * wx + s * wy
            vy = -s * wx + c * wy
        # Do not drive into arena walls while tasking
        if rx > 1.48 and vx > 0:
            vx = min(vx, 0.0)
        if rx < -1.58 and vx < 0:
            vx = max(vx, 0.0)
        if ry > 1.28 and vy > 0:
            vy = min(vy, 0.0)
        if ry < -1.28 and vy < 0:
            vy = max(vy, 0.0)
        self._publish(vx, vy, TURN_GAIN * err)
        return dist

    def _task_target_xy(self) -> tuple[float, float]:
        """Current delivery target — used so avoid does not abandon the task."""
        if self.has_brick:
            return GOAL_XY
        cube = self._nearest_cube()
        if cube is not None:
            return self._approach_xy(cube[1], cube[2])
        return START_XY

    def avoid_person(self, fear: float, *, keep_task: bool = False) -> None:
        """Move away from the person; while carrying, slip sideways and keep X progress."""
        del keep_task
        ax, ay = self._away_from_person()
        rx, ry = self._robot_xy
        px, py = self._person_xy
        left = (-ay, ax)
        right = (ay, -ax)
        to_c_x, to_c_y = -rx, -ry
        cn = math.hypot(to_c_x, to_c_y) or 1.0
        to_c_x, to_c_y = to_c_x / cn, to_c_y / cn
        # Prefer RL corridor side when carrying
        side_y = 0.0
        if self.has_brick and self._route == "north":
            side_y = 1.0
        elif self.has_brick and self._route == "south":
            side_y = -1.0

        def score(dir_xy: tuple[float, float]) -> float:
            nx = rx + dir_xy[0] * 0.50
            ny = ry + dir_xy[1] * 0.50
            clear = math.hypot(nx - px, ny - py)
            pen = 0.0
            if nx > 1.25 or nx < -1.40 or abs(ny) > 1.05:
                pen -= 2.0
            fc = self._furniture_clearance(nx, ny)
            if fc < 0.0:
                pen -= 2.5
            elif fc < 0.20:
                pen -= 1.2 * (1.0 - fc / 0.20)
            if side_y != 0.0 and dir_xy[1] * side_y > 0:
                clear += 0.8
            # Prefer continuing toward task target in X while carrying
            if self.has_brick:
                tx, _ty = self._task_target_xy()
                if (tx - rx) * dir_xy[0] > 0:
                    clear += 0.5
            return clear + pen

        candidates = [
            (ax, ay),
            left,
            right,
            (to_c_x, to_c_y),
            (0.7 * ax + 0.5 * to_c_x, 0.7 * ay + 0.5 * to_c_y),
            (0.4, side_y) if side_y != 0.0 else (ax, ay),
        ]
        best = max(candidates, key=score)
        bx, by = best
        bn = math.hypot(bx, by) or 1.0
        bx, by = bx / bn, by / bn

        pd = self._person_dist()
        urgency = 1.0 + max(0.0, (1.0 - pd) / 1.0)
        speed = min(0.50, (0.28 + 0.0035 * fear) * urgency)
        wx, wy = bx * speed, by * speed

        if rx > 1.35:
            wx = min(wx, -0.35)
        elif rx < -1.45:
            wx = max(wx, 0.35)
        if ry > 1.15:
            wy = min(wy, -0.35)
        elif ry < -1.15:
            wy = max(wy, 0.35)

        fxx, fyy = self._furniture_repel(speed)
        wx += fxx
        wy += fyy

        c, s = math.cos(self._robot_yaw), math.sin(self._robot_yaw)
        vx = c * wx + s * wy
        vy = -s * wx + c * wy
        desired = math.atan2(wy, wx)
        err = (desired - self._robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._publish(vx, vy, 1.2 * err)

    def target_dist_m(self) -> float:
        """Distance to current task target (brick or goal) for RL state."""
        rx, ry = self._robot_xy
        if self.has_brick:
            return math.hypot(GOAL_XY[0] - rx, GOAL_XY[1] - ry)
        cube = self._nearest_cube()
        if cube is None:
            return math.hypot(START_XY[0] - rx, START_XY[1] - ry)
        return math.hypot(cube[1] - rx, cube[2] - ry)

    def _cube_at_grabbers(self, max_dist: float = 0.35) -> Optional[str]:
        """Name of the cube currently nearest the front grab point."""
        if not self._cube_xy:
            return None
        rx, ry = self._robot_xy
        # Grabbers ~0.20 m ahead of base
        gx = rx + 0.20 * math.cos(self._robot_yaw)
        gy = ry + 0.20 * math.sin(self._robot_yaw)
        best_name = None
        best_d = max_dist
        for name, (cx, cy) in self._cube_xy.items():
            if name in self._delivered_names:
                continue
            d = math.hypot(cx - gx, cy - gy)
            if d < best_d:
                best_d = d
                best_name = name
        return best_name

    def _approach_xy(self, cx: float, cy: float) -> tuple[float, float]:
        """Stand-off point for grabbing — always inside the arena.

        Old logic always used (cx - reach, cy) which aimed through the west wall
        when a brick sat left of the green mat → soft-lock / 'waiting' with task.
        """
        reach = GRABBER_REACH_M
        candidates = (
            (cx - reach, cy),
            (cx + reach, cy),
            (cx, cy - reach),
            (cx, cy + reach),
            (cx - reach * 0.7, cy - reach * 0.7),
            (cx + reach * 0.7, cy - reach * 0.7),
            (cx - reach * 0.7, cy + reach * 0.7),
            (cx + reach * 0.7, cy + reach * 0.7),
        )

        def inside(p: tuple[float, float]) -> bool:
            return -1.40 <= p[0] <= 1.40 and -1.10 <= p[1] <= 1.10

        valid = [p for p in candidates if inside(p)]
        if not valid:
            return (
                max(-1.35, min(1.35, cx)),
                max(-1.0, min(1.0, cy)),
            )
        rx, ry = self._robot_xy
        # Prefer interior points closer to the robot
        return min(valid, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))

    def _nearest_cube(self) -> Optional[tuple[str, float, float]]:
        if not self._cubes:
            return None
        rx, ry = self._robot_xy
        return min(self._cubes, key=lambda c: math.hypot(c[1] - rx, c[2] - ry))

    def _remaining(self) -> int:
        if self._props_seen:
            return len(self._cubes)
        return max(0, EXPECTED_BRICKS - self._delivered)

    # --- state machine ------------------------------------------------------
    def tick(self) -> tuple[bool, str]:
        # Empty-handed stuck in delivery phases → go fetch remaining bricks
        if not self.has_brick and self.phase in ("release", "seek_goal"):
            prev = self.phase
            self.phase = "return_start"
            self._hold_ticks = 0
            self._escape_ticks = 0
            self.node.get_logger().warn(
                "task recovery: empty-handed in %s → return_start" % prev
            )

        if self.phase == "done":
            # Bricks still on the field → reopen (common after a false "4/4" or
            # sim reset while the BT stayed alive). Avoid still worked because
            # Nav2 avoid runs when phase==done; task published cmd_vel=0.
            if self._props_seen and self._cubes:
                self.phase = "seek_cube"
                self.has_brick = False
                self._carried_name = None
                self._hold_ticks = 0
                self._delivered = len(self._delivered_names)
                self.node.get_logger().warn(
                    "task reopened: %d cube(s) remain (was done, delivered=%d)"
                    % (len(self._cubes), self._delivered)
                )
            else:
                self._publish(0.0, 0.0, 0.0)
                return False, "task:done"

        if self.phase == "seek_cube":
            # Never open grippers here if we still think we are carrying
            if not self.has_brick:
                self._set_grip(True)
            cube = self._nearest_cube()
            if cube is None:
                if not self._props_seen:
                    self._wait_props_ticks = getattr(self, "_wait_props_ticks", 0) + 1
                    if self._wait_props_ticks == 1 or self._wait_props_ticks % 50 == 0:
                        self.node.get_logger().warn(
                            "task:wait_props — no /sim/props yet (robot idle until cubes appear)"
                        )
                    # Nudge toward start so we are not frozen in a corner while waiting
                    self._drive_routed(START_XY[0], START_XY[1], speed=0.20)
                    return False, "task:wait_props"
                self._wait_props_ticks = 0
                if self._delivered >= EXPECTED_BRICKS and not self._cubes:
                    self.phase = "done"
                    self.node.get_logger().info(
                        "ALL BRICKS DONE — %d on goal mat" % self._delivered
                    )
                    self._publish(0.0, 0.0, 0.0)
                    return False, "task:done"
                # Never declare done while pickable cubes remain
                if self._cubes:
                    pass  # fall through only if cube is None — handled above
                # Return toward start to find remaining bricks
                self._drive_routed(START_XY[0], START_XY[1], speed=0.36)
                return False, "task:return_start"

            name, cx, cy = cube
            rx, ry = self._robot_xy
            approach = self._approach_xy(cx, cy)
            if math.hypot(cx - rx, cy - ry) < GRAB_NEAR_M:
                self.phase = "grab"
                self._hold_ticks = 0
                self._carried_name = name
                self._route = None
                return False, "task:seek_cube"
            # Soft-lock after avoid/corner: pull to arena centre, reset route
            if self._stuck_ticks > 40 or (abs(rx) > 1.25 and abs(ry) > 0.85):
                if self._stuck_ticks > 40:
                    self._stuck_ticks = 0
                    self._route = None
                    self._avoid_side = 0.0
                    self._last_progress_dist = 99.0
                    self.node.get_logger().warn(
                        "seek_cube unstick → centre then cube (robot=%.2f,%.2f)"
                        % (rx, ry)
                    )
                self._drive_toward(0.0, 0.0, speed=0.38, clear_person=True)
                return False, "task:seek_cube"
            # If progress stalls (wall-side brick), flip approach by driving to cube centre
            if self._stuck_ticks > 25:
                approach = (
                    max(-1.35, min(1.35, cx)),
                    max(-1.05, min(1.05, cy)),
                )
            # Cross-arena cube fetch must also pick north/south when middle is occupied
            self._drive_routed(approach[0], approach[1], speed=0.32)
            return False, "task:seek_cube"

        if self.phase == "grab":
            cube = self._nearest_cube()
            rx, ry = self._robot_xy
            # Stay in grab once started; only abort if cube vanished / far after trying
            if cube is None:
                self.phase = "seek_cube"
                self.has_brick = False
                self._carried_name = None
                return False, "task:seek_cube"
            near = math.hypot(cube[1] - rx, cube[2] - ry)
            if near > 0.60 and self._hold_ticks > 5:
                self.phase = "seek_cube"
                self.has_brick = False
                self._carried_name = None
                return False, "task:seek_cube"
            self._carried_name = cube[0]
            self._set_grip(False)
            self._drive_toward(cube[1], cube[2], speed=0.14, clear_person=False)
            self._hold_ticks += 1
            # Require the cube to actually be in grab reach before claiming carry
            if self._hold_ticks >= 10 and near < GRAB_NEAR_M:
                self.has_brick = True
                self.phase = "seek_goal"
                self._stuck_ticks = 0
                self._last_progress_dist = 99.0
                self._last_goal_progress = 99.0
                self._goal_stuck_ticks = 0
                self._route = None
                # Prefer the cube that is actually at the grabbers (MuJoCo attach)
                attached = self._cube_at_grabbers()
                if attached is not None:
                    self._carried_name = attached
                self._set_grip(False)
                self.node.get_logger().info(
                    "Grabbed %s (%d left after this)"
                    % (self._carried_name, max(0, self._remaining() - 1))
                )
            elif self._hold_ticks > 40:
                # Failed to secure — retry seek
                self.phase = "seek_cube"
                self.has_brick = False
                self._carried_name = None
                self._set_grip(True)
            return self.has_brick, "task:grab"

        if self.phase == "seek_goal":
            # Reactive delivery: RL route + hard unstick to the mat
            self._set_grip(False)
            attached = self._cube_at_grabbers()
            if attached is not None:
                self._carried_name = attached
            rx, ry = self._robot_xy
            gx, gy = GOAL_XY
            goal_dist = math.hypot(gx - rx, gy - ry)

            # Progress vs GOAL (not via) — via-progress was resetting forever
            if goal_dist < self._last_goal_progress - 0.04:
                self._goal_stuck_ticks = 0
                self._last_goal_progress = goal_dist
            else:
                self._goal_stuck_ticks += 1

            near_goal = rx >= 0.95 and goal_dist < 1.05
            force_peel = self._goal_stuck_ticks >= GOAL_STUCK_TICKS

            if force_peel:
                if self._goal_stuck_ticks == GOAL_STUCK_TICKS:
                    self.node.get_logger().warn(
                        "seek_goal unstick → direct to mat (robot=%.2f,%.2f goal=%.2f)"
                        % (rx, ry, goal_dist)
                    )
                # Keep side Y until past person, then go straight to mat
                px = self._person_xy[0]
                if abs(rx - px) < 0.90 and self._route in ("north", "south"):
                    cy = self._route_y(self._route, gx, gy)
                    self._drive_toward(
                        gx, cy, speed=0.36, clear_person=False
                    )
                else:
                    self._drive_toward(gx, gy, speed=0.34, clear_person=False)
            elif near_goal:
                self._route = None
                self._drive_toward(gx, gy, speed=0.24, clear_person=False)
            else:
                self._drive_routed(gx, gy, speed=DRIVE_SPEED)

            if goal_dist < 0.38 and rx >= 1.05 and RELEASE_RY_LO <= ry <= RELEASE_RY_HI:
                self.phase = "release"
                self._hold_ticks = 0
                self._escape_ticks = 0
                self._route = None
                self._goal_stuck_ticks = 0
                self._last_goal_progress = 99.0
                self._release_open = False
                self._delivered_at_release_start = len(self._delivered_names)
            return True, "task:seek_goal"

        if self.phase == "release":
            # Creep onto mat centre while closed — MuJoCo only parks if base is on_mat
            rx, ry = self._robot_xy
            on_mat = 1.08 <= rx <= 1.58 and RELEASE_RY_LO <= ry <= 0.20
            if not on_mat:
                self._release_open = False
                self._set_grip(False)
                self._drive_toward(GOAL_XY[0], GOAL_XY[1], speed=0.16, clear_person=False)
                self._hold_ticks += 1
                if self._hold_ticks > 100:
                    self.phase = "seek_goal"
                    self._stuck_ticks = 0
                    self._escape_ticks = 0
                    self._hold_ticks = 0
                return True, "task:release"
            if not getattr(self, "_release_open", False):
                self._release_open = True
                self._hold_ticks = 0
            self._publish(0.0, 0.0, 0.0)
            self._set_grip(True)  # open → MuJoCo drops at gripper on the goal mat
            self._hold_ticks += 1
            if self._hold_ticks < 25:
                return True, "task:release"
            name = self._carried_name or "brick"
            # Confirm by THIS brick pose OR by any new brick appearing on the mat
            xy = self._cube_xy.get(name)
            on_goal = name in self._delivered_names
            if xy is not None and 1.00 <= xy[0] <= 1.70 and -0.50 <= xy[1] <= 0.15:
                on_goal = True
                self._delivered_names.add(name)
            n_on_goal = len(self._delivered_names)
            if n_on_goal > getattr(self, "_delivered_at_release_start", 0):
                on_goal = True
                if name not in self._delivered_names:
                    for n, (cx, cy) in self._cube_xy.items():
                        if 1.00 <= cx <= 1.70 and -0.50 <= cy <= 0.15:
                            name = n
                            self._delivered_names.add(n)
                            break
            # If still on mat after a long open and cube is near base, accept delivery
            if (
                not on_goal
                and self._hold_ticks >= 45
                and xy is not None
                and math.hypot(xy[0] - rx, xy[1] - ry) < 0.45
            ):
                on_goal = True
                self._delivered_names.add(name)
            if not on_goal and self._hold_ticks < 80:
                return True, "task:release"
            if not on_goal:
                # Missed the mat — close grip and keep seeking goal (don't abandon brick)
                self._set_grip(False)
                self.phase = "seek_goal"
                self._stuck_ticks = 0
                self._escape_ticks = 0
                self._hold_ticks = 0
                self.node.get_logger().warn(
                    "Release of %s missed mat — retrying seek_goal" % name
                )
                return True, "task:seek_goal"
            self._delivered = max(self._delivered, len(self._delivered_names))
            self.has_brick = False
            self._carried_name = None
            self.phase = "return_start"
            self.node.get_logger().info(
                "Confirmed %s on goal mat (%d/%d physical)"
                % (name, self._delivered, EXPECTED_BRICKS)
            )
            return False, "task:release"

        if self.phase == "return_start":
            self._set_grip(True)
            if self._delivered >= EXPECTED_BRICKS and not self._cubes:
                self.phase = "done"
                self.node.get_logger().info(
                    "ALL BRICKS DONE — delivered %d" % self._delivered
                )
                self._publish(0.0, 0.0, 0.0)
                return False, "task:done"
            if self._cubes:
                self.phase = "seek_cube"
                self._route = None
                return False, "task:seek_cube"
            rx, ry = self._robot_xy
            dist = self._drive_routed(START_XY[0], START_XY[1], speed=0.40)
            if rx < -1.0 and abs(ry - START_XY[1]) < 0.55:
                self.phase = "seek_cube"
                self._route = None
            elif dist < 0.40 and rx < -0.8:
                self.phase = "seek_cube"
                self._route = None
            return False, "task:return_start"

        return self.has_brick, "task:%s" % self.phase
