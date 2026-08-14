"""
The movement behaviours, bound to the way these experiments spell their keys.

`mecanumbot_movement_behaviours` holds the behaviours themselves; what it does
not hold is the name this experiment's constants file gives its route. Binding
`LEADING_KEYS` on a subclass is what supplies that, once, so the trees go on
saying `Approach(target_type=CHECKPOINT)` and no call site carries a `keys=`
argument.

Behaviours that read nothing experiment-specific are re-exported unchanged, so
this module is also the single import the tree files need for movement.
"""

from mecanumbot_movement_behaviours import approach as _approach
from mecanumbot_movement_behaviours import routes as _routes
from mecanumbot_movement_behaviours import turning as _turning
from mecanumbot_movement_behaviours.approach import (  # noqa: F401  (re-export)
    CheckRobotHasBall,
)
from mecanumbot_movement_behaviours.routes import (  # noqa: F401  (re-export)
    SEARCH_BACKWARDS,
    SEARCH_FORWARDS,
)
from mecanumbot_movement_behaviours.turning import (  # noqa: F401  (re-export)
    REPEAT,
    SHORTEST,
    UNWIND,
    FindPeople,
    InPlaceTurn,
    RelativeTurnPattern,
    ScanSpin,
    SmoothTurner,
    Spin360,
)

from mecanumbot_leading_behaviour.behaviours.keys import LEADING_KEYS


class Approach(_approach.Approach):
    """Approach, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class CheckSubjectTargetSuccess(_approach.CheckSubjectTargetSuccess):
    """CheckSubjectTargetSuccess, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class FollowRoute(_routes.FollowRoute):
    """FollowRoute, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class CheckRobotAtLastCheckpoint(_routes.CheckRobotAtLastCheckpoint):
    """CheckRobotAtLastCheckpoint, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class WaitForPerson(_routes.WaitForPerson):
    """WaitForPerson, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class ManageSearchCheckpoint(_routes.ManageSearchCheckpoint):
    """ManageSearchCheckpoint, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class TurnToward(_turning.TurnToward):
    """TurnToward, reading this experiment's route keys."""

    KEYS = LEADING_KEYS


class GlanceBack(_turning.GlanceBack):
    """GlanceBack, reading this experiment's route keys."""

    KEYS = LEADING_KEYS
