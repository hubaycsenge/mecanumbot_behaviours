"""Fear HRI blackboard keys and parameter loading."""

from mecanumbot_bt_config.blackboard import ParamsToBlackboard
from mecanumbot_bt_config.tree_runner import RUNTIME_DEFAULTS
from mecanumbot_movement_behaviours.defaults import MOVEMENT_DEFAULTS

FEAR_PARAMS = (
    "safe_distance_m",
    "hit_distance_m",
    "fear_decay_per_step",
    "fear_spike_on_event",
    "fear_rise_per_risk",
    "avoid_distance_m",
    "freeze_fear_threshold",
    "avoid_fear_threshold",
    "catch_fear_low",
    "catch_fear_high",
)

RUN_STATE = {
    "fear_level": 0.0,
    "fear_risk": 0.0,
    "min_lidar_range": 10.0,
    "selected_action": "task",
    "has_brick": False,
}

LOADED_DEFAULTS = (RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS)


class FearParamsToBlackboard(ParamsToBlackboard):
    def __init__(self, name: str, yaml_path: str):
        super().__init__(
            name=name,
            yaml_path=yaml_path,
            defaults=LOADED_DEFAULTS,
            required=FEAR_PARAMS,
            state=RUN_STATE,
        )
