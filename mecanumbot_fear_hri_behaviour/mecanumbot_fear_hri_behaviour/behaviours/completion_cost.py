"""Completion-time cost for route choice (Csenge).

cost = travel_time + wait_time  [seconds]
  travel = path_length / cruise_speed
  wait   = expected linger if the person blocks that corridor

Corridors are FIXED arena lanes (middle / north / south), not "around the person".
If the person stands on a lane, that lane's wait dominates → long clear detour wins.
"""

from __future__ import annotations

import math

CRUISE_MPS = 0.34
WAIT_FULL_S = 18.0
WAIT_PARTIAL_S = 8.0
PERSON_CLEAR_M = 1.25
# Fixed arena corridors (match brick_carry / training scenario)
LANE_Y = {"middle": 0.0, "north": 0.90, "south": -0.90}
LANE_HALF_WIDTH = 0.55


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _via_y(route: str, robot_y: float, target_y: float, person_y: float) -> float:
    del person_y  # lane is fixed; person only affects wait via clearance
    if route == "middle":
        return 0.5 * (robot_y + target_y)
    return LANE_Y.get(route, 0.0)


def path_length_m(
    route: str,
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> tuple[float, float]:
    rx, ry = robot
    tx, ty = target
    px, py = person
    cy = _via_y(route, ry, ty, py)
    if route == "middle":
        samples = [
            (rx + (tx - rx) * t, ry + (ty - ry) * t) for t in (0.20, 0.40, 0.60, 0.80)
        ]
        path = _dist(robot, target)
    else:
        sign = 1.0 if tx >= rx else -1.0
        # Sweep past person x on the fixed lane
        entry = (px - sign * 0.70, cy)
        mid = (px, cy)
        exit_wp = (px + sign * 1.10, cy)
        samples = [entry, mid, exit_wp]
        path = _dist(robot, entry) + _dist(entry, exit_wp) + _dist(exit_wp, target)
    min_person = min(_dist(s, person) for s in samples)
    # Also count how close person sits to the lane in Y (blocks that corridor)
    lane_block = abs(py - cy)
    if abs(px - 0.5 * (rx + tx)) < abs(tx - rx) * 0.6 + 0.4:
        min_person = min(min_person, lane_block)
    return path, min_person


def person_on_lane(
    route: str,
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> bool:
    rx, ry = robot
    tx, ty = target
    px, py = person
    cy = _via_y(route, ry, ty, py)
    lo, hi = (rx, tx) if rx <= tx else (tx, rx)
    in_x = lo - 0.25 <= px <= hi + 0.25
    return in_x and abs(py - cy) < LANE_HALF_WIDTH


def person_on_middle_corridor(
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
    *,
    band: float = 0.55,
) -> bool:
    del band
    return person_on_lane("middle", robot, target, person)


def wait_time_s(
    route: str,
    min_person: float,
    *,
    person_on_middle: bool = False,
    person_blocks_lane: bool = False,
) -> float:
    del person_on_middle
    if min_person >= PERSON_CLEAR_M and not person_blocks_lane:
        return 0.0
    blocked_frac = 1.0 - _clamp(min_person / PERSON_CLEAR_M, 0.0, 1.0)
    if person_blocks_lane:
        # Lane occupied by human → full wait (finish ASAP elsewhere)
        return WAIT_FULL_S * (0.70 + 0.30 * max(blocked_frac, 0.5))
    if min_person < 0.85:
        return WAIT_FULL_S * (0.45 + 0.55 * blocked_frac)
    return WAIT_PARTIAL_S * blocked_frac


def completion_time_s(
    route: str,
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> tuple[float, float, float]:
    """(total_s, travel_s, wait_s)."""
    path, min_person = path_length_m(route, robot, target, person)
    travel = path / max(0.05, CRUISE_MPS)
    blocks = person_on_lane(route, robot, target, person)
    wait = wait_time_s(
        route,
        min_person,
        person_blocks_lane=blocks,
    )
    return travel + wait, travel, wait


def all_route_costs(
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> dict[str, tuple[float, float, float]]:
    return {
        name: completion_time_s(name, robot, target, person)
        for name in ("middle", "north", "south")
    }


def cheapest_route(
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> str:
    costs = all_route_costs(robot, target, person)
    return min(costs, key=lambda n: costs[n][0])


def normalized_cost_features(
    robot: tuple[float, float],
    target: tuple[float, float],
    person: tuple[float, float],
) -> tuple[float, float, float]:
    """Relative completion times in [0,1] for DQN (0 = cheapest)."""
    costs = all_route_costs(robot, target, person)
    totals = [costs[n][0] for n in ("middle", "north", "south")]
    lo, hi = min(totals), max(totals)
    span = max(1e-3, hi - lo)
    return tuple((c - lo) / span for c in totals)  # type: ignore[return-value]
