"""Live Fear HRI policy — Double DQN with completion-time cost features.

Cost model (Csenge): completion_time = travel + wait.
DQN sees relative costs for middle/north/south and learns the action.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

ACTION_NAMES = (
    "freeze",
    "avoid",
    "catch_attention",
    "task_middle",
    "task_north",
    "task_south",
)
N_ACTIONS = 6
N_FEATURES = 10
FEAR_MAX = 100.0
# Person "caught" the robot → hard freeze (safety latch; RL handles all other ranges)
FREEZE_CATCH_M = 0.55


def features_live(
    *,
    fear_level: float,
    min_range: float,
    has_brick: bool,
    target_dist_m: float,
    person_dx: float,
    person_dy: float,
    risk: float = 0.0,
    cost_middle: float = 0.5,
    cost_north: float = 0.5,
    cost_south: float = 0.5,
) -> np.ndarray:
    return np.array(
        [
            float(fear_level) / FEAR_MAX,
            min(float(min_range) / 4.0, 1.0),
            1.0 if has_brick else 0.0,
            min(float(target_dist_m) / 4.0, 1.0),
            float(np.clip(person_dy / 2.0, -1.0, 1.0)),
            float(np.clip(person_dx / 3.0, -1.0, 1.0)),
            min(max(float(risk), 0.0), 1.0),
            float(np.clip(cost_middle, 0.0, 1.0)),
            float(np.clip(cost_north, 0.0, 1.0)),
            float(np.clip(cost_south, 0.0, 1.0)),
        ],
        dtype=np.float32,
    )


def _relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


class DqnPolicy:
    def __init__(self, weights: dict[str, np.ndarray]) -> None:
        self.w1 = weights["w1"]
        self.b1 = weights["b1"]
        self.w2 = weights["w2"]
        self.b2 = weights["b2"]
        self.w3 = weights["w3"]
        self.b3 = weights["b3"]
        if self.w1.shape[0] != N_FEATURES or self.w3.shape[1] != N_ACTIONS:
            raise ValueError(
                "DQN shape mismatch: in=%s out=%s expected (%d, %d)"
                % (self.w1.shape[0], self.w3.shape[1], N_FEATURES, N_ACTIONS)
            )

    @classmethod
    def load(cls, path: Path) -> "DqnPolicy":
        data = np.load(path)
        return cls({k: data[k].astype(np.float32) for k in ("w1", "b1", "w2", "b2", "w3", "b3")})

    def q_values(self, feat: np.ndarray) -> np.ndarray:
        x = feat.astype(np.float32).reshape(1, -1)
        h1 = _relu(x @ self.w1 + self.b1)
        h2 = _relu(h1 @ self.w2 + self.b2)
        return (h2 @ self.w3 + self.b3)[0]

    def act(self, feat: np.ndarray, *, catch_attention_enabled: bool = True) -> str:
        fear_n = float(feat[0])
        min_range_n = float(feat[1])  # min_range / 4
        min_range_m = min_range_n * 4.0
        # Hard freeze when the person has caught / is too close — must stop
        if min_range_m <= FREEZE_CATCH_M:
            return "freeze"
        q = self.q_values(feat).copy()
        if not catch_attention_enabled:
            q[2] = -1e9
        # Among task corridors, prefer cheaper completion-time (feat 7/8/9 = M/N/S, 0=cheap)
        for i in range(3):
            q[3 + i] -= 0.55 * float(feat[7 + i])
        # Person clear (≥0.90 m): never freeze — residual fear must not stall the task
        if min_range_m >= 0.90:
            q[0] = -1e9
            if fear_n < 0.35:
                q[1] = -1e9
        # Not caught + essentially unafraid → never freeze
        elif min_range_m > FREEZE_CATCH_M and fear_n < 0.05:
            q[0] = -1e9
        # Close band: bias toward freeze (then avoid), not keep driving past person
        elif min_range_m <= 0.85:
            q[0] += 0.90  # freeze
            q[1] += 0.35  # avoid
            q[3] -= 0.50  # task_middle risky when close
        return ACTION_NAMES[int(np.argmax(q))]


_POLICY: Optional[DqnPolicy] = None
_LOAD_TRIED = False


def _candidate_dqn() -> list[Path]:
    paths: list[Path] = []
    env = os.environ.get("FEAR_DQN_MODEL", "").strip()
    if env:
        paths.append(Path(env))
    here = Path(__file__).resolve()
    for root in (
        here.parents[4] / "fear_mecanumbot",
        Path("/mnt/c/Users/Fahrad/Desktop/AI Lab/fear_mecanumbot"),
        Path.home() / "fear_mecanumbot",
    ):
        paths.append(root / "models" / "hri_dqn.npz")
    return paths


def get_policy() -> Optional[DqnPolicy]:
    global _POLICY, _LOAD_TRIED
    if _POLICY is not None:
        return _POLICY
    if _LOAD_TRIED:
        return None
    _LOAD_TRIED = True
    for path in _candidate_dqn():
        try:
            if path.is_file() and path.suffix == ".npz":
                _POLICY = DqnPolicy.load(path)
                return _POLICY
        except Exception:
            continue
    return None


def policy_kind() -> str:
    return "dqn" if get_policy() is not None else "none"


def catch_attention_allowed() -> bool:
    env = os.environ.get("FEAR_CATCH_ATTENTION", "").strip().lower()
    if env in ("1", "true", "on", "irl", "yes"):
        return True
    if env in ("0", "false", "off", "sim", "no"):
        return False
    sim = os.environ.get("FEAR_USE_SIM", "").strip().lower()
    if sim in ("1", "true", "on", "yes"):
        return False
    return True


def select_action_rl(
    *,
    fear_level: float,
    min_range: float,
    has_brick: bool = False,
    target_dist_m: float = 99.0,
    person_dx: float = 0.0,
    person_dy: float = 0.0,
    risk: float = 0.0,
    cost_middle: float = 0.5,
    cost_north: float = 0.5,
    cost_south: float = 0.5,
    catch_attention_enabled: Optional[bool] = None,
    **_ignored,
) -> Optional[str]:
    policy = get_policy()
    if policy is None:
        return None
    if catch_attention_enabled is None:
        catch_attention_enabled = catch_attention_allowed()
    feat = features_live(
        fear_level=fear_level,
        min_range=min_range,
        has_brick=has_brick,
        target_dist_m=target_dist_m,
        person_dx=person_dx,
        person_dy=person_dy,
        risk=risk,
        cost_middle=cost_middle,
        cost_north=cost_north,
        cost_south=cost_south,
    )
    return policy.act(feat, catch_attention_enabled=bool(catch_attention_enabled))


def action_to_route(action: str) -> Optional[str]:
    if action == "task_north":
        return "north"
    if action == "task_south":
        return "south"
    if action in ("task_middle", "task"):
        return "middle"
    return None
