"""ROS interface adapters for Fear HRI behaviour nodes."""

from __future__ import annotations

import math
import time

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# Relative name: works under namespace:=mecanumbot as /mecanumbot/scan.
# Absolute fallback is remapped in launch if needed.
SCAN_TOPIC = "scan"


class LidarRangeTracker:
    """Track forward min range and closing speed from LaserScan."""

    def __init__(self, node) -> None:
        self._node = node
        self._min_range = 10.0
        self._prev_range = 10.0
        self._prev_time = time.monotonic()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = node.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos)
        # Also subscribe absolute path used by some bringup layouts.
        self._sub_abs = node.create_subscription(
            LaserScan, "/mecanumbot/scan", self._on_scan, qos
        )

    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        half = math.pi / 2.0
        best = float(msg.range_max) if msg.range_max > 0.0 else 8.0
        for i, raw in enumerate(msg.ranges):
            r = float(raw)
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= half:
                best = min(best, r)
        now = time.monotonic()
        self._prev_range = self._min_range
        self._dt = max(1e-3, now - self._prev_time)
        self._prev_time = now
        self._min_range = best

    @property
    def min_range(self) -> float:
        return self._min_range

    @property
    def closing_speed(self) -> float:
        dt = getattr(self, "_dt", 0.1)
        return max(0.0, (self._prev_range - self._min_range) / dt)
