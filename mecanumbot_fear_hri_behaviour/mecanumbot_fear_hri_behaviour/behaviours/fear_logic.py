"""Fear/risk math vendored for the ROS package (mirrors fear_mecanumbot)."""


def compute_risk(
    *,
    min_range: float,
    safe_distance: float = 1.0,
    hit_distance: float = 0.25,
    closing_speed: float = 0.0,
    speed_scale: float = 0.5,
) -> float:
    if min_range <= hit_distance:
        proximity = 1.0
    elif min_range >= safe_distance:
        proximity = 0.0
    else:
        proximity = (safe_distance - min_range) / (safe_distance - hit_distance)
    speed_term = max(0.0, float(closing_speed)) * speed_scale
    return float(max(0.0, min(1.0, proximity + speed_term)))


def is_near_collision(min_range: float, hit_distance: float = 0.25) -> bool:
    return float(min_range) <= float(hit_distance)


def update_fear_level(
    level: float,
    *,
    risk: float,
    spike_event: bool,
    decay_per_step: float = 0.2,
    spike_on_event: float = 50.0,
    rise_per_risk: float = 5.0,
) -> float:
    fear = float(level)
    if spike_event:
        fear = min(100.0, fear + spike_on_event)
    else:
        fear = max(0.0, fear - decay_per_step)
    fear = min(100.0, fear + rise_per_risk * risk)
    return fear


def select_action(
    *,
    fear_level: float,
    min_range: float,
    risk: float,
    hit_distance: float,
    safe_distance: float,
    freeze_threshold: float,
    avoid_threshold: float,
    catch_low: float,
    catch_high: float,
) -> str:
    if min_range <= hit_distance + 0.05 or (fear_level >= freeze_threshold and min_range < safe_distance):
        return "freeze"
    if fear_level >= avoid_threshold or risk >= 0.5:
        return "avoid"
    if catch_low <= fear_level < catch_high:
        return "catch_attention"
    return "task"


def avoid_retreat_distance_m(fear_level: float, *, base: float = 0.8, scale: float = 0.01) -> float:
    return base + scale * max(0.0, min(100.0, float(fear_level)))
