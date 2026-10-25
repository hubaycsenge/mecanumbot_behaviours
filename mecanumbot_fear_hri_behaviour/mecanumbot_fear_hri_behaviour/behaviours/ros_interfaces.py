"""ROS interface adapters for Fear HRI behaviour nodes."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# Relative name: works under namespace:=mecanumbot as /mecanumbot/scan.
SCAN_TOPIC = "scan"
# Narrow forward cone so arena walls/boxes do not pin fear forever.
FORWARD_HALF_ANGLE = math.radians(40.0)


class LidarRangeTracker:
    """Track proximity risk from scan + optional subject-pose oracle."""

    def __init__(self, node) -> None:
        self._node = node
        self._min_range = 10.0
        self._prev_range = 10.0
        self._prev_time = time.monotonic()
        self._dt = 0.1
        self._subject_range = None
        self._subject_xy = None
        self._gt_subject = False
        self._gt_robot = False
        self._robot_xy = (-1.35, -0.20)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = node.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos)
        self._sub_abs = node.create_subscription(
            LaserScan, "/mecanumbot/scan", self._on_scan, qos
        )
        node.create_subscription(PoseStamped, "/sim/robot_pose", self._on_robot, qos)
        node.create_subscription(
            PoseStamped, "/sim/subject_pose_ground_truth", self._on_subject_gt, qos
        )
        node.create_subscription(
            PoseStamped, "/mecanumbot/subject_pose", self._on_subject_est, qos
        )

    def _on_robot(self, msg: PoseStamped) -> None:
        self._gt_robot = True
        self._robot_xy = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        # Recompute subject range with consistent GT robot pose
        if self._subject_xy is not None:
            sx, sy = self._subject_xy
            self._subject_range = math.hypot(sx - self._robot_xy[0], sy - self._robot_xy[1])

    def _set_subject(self, sx: float, sy: float) -> None:
        self._subject_xy = (sx, sy)
        rx, ry = self._robot_xy
        self._subject_range = math.hypot(sx - rx, sy - ry)

    def _on_subject_gt(self, msg: PoseStamped) -> None:
        self._gt_subject = True
        self._set_subject(float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_subject_est(self, msg: PoseStamped) -> None:
        # Ignore detector spikes once sim ground truth is available
        if self._gt_subject:
            return
        self._set_subject(float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        best = float(msg.range_max) if msg.range_max > 0.0 else 8.0
        for i, raw in enumerate(msg.ranges):
            r = float(raw)
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= FORWARD_HALF_ANGLE:
                best = min(best, r)
        now = time.monotonic()
        self._prev_range = self._min_range
        self._dt = max(1e-3, now - self._prev_time)
        self._prev_time = now
        self._min_range = best

    @property
    def wall_range(self) -> float:
        return float(self._min_range)

    @property
    def min_range(self) -> float:
        # Fear tracks the person (GT in sim) — not walls/boxes.
        if self._subject_range is not None:
            return float(self._subject_range)
        return self._min_range

    @property
    def closing_speed(self) -> float:
        return max(0.0, (self._prev_range - self.min_range) / self._dt)
