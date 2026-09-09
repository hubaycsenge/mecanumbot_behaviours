"""
Find the frontiers: where the known map stops and the robot could learn more.

This behaviour moves nothing. It is here rather than in
`mecanumbot_custom_nav2` because the detector's *state* -- two RRTs that have to
be thrown away and regrown when slam_toolbox re-rasterises the map -- belongs to
the pass, while the detector itself does not. `FrontierSearch`, the scoring and
the occupancy grid stay in `mecanumbot_custom_nav2`, where they are pure
functions over a grid with 173 tests behind them and no notion of a robot at
all.

One tick grows both trees by their per-cycle iteration counts, snaps whatever
stopped against a free/unknown boundary, clusters the candidates, revalidates
each cluster against the current grid and scores what survives:

    score = gain_weight * (gain / largest gain) - cost_weight * distance

plus `hysteresis` for the candidate nearest the goal already being driven to,
without which the explorer re-decides at the tick rate and oscillates between
two equally good frontiers, driving to neither.
"""

from mecanumbot_custom_nav2 import frontiers as frontier_tools
from mecanumbot_custom_nav2.rrt import FrontierSearch

from mecanumbot_autoslam.behaviours.base import FAILURE, SUCCESS, Behaviour


class DetectFrontiers(Behaviour):
    """Grow the RRTs over the current map and score what they find."""

    def __init__(self, name="detect_frontiers"):
        """Create the behaviour; the search is built in `setup()`."""
        super().__init__(name)
        self.search = None
        self._scoring = {}

    def setup(self, node, params):
        """Build the RRT search and cache the scoring constants."""
        super().setup(node, params)
        self.search = FrontierSearch(
            step_size=params["rrt_step_size"],
            snap_radius=params["rrt_snap_radius"],
            global_iterations=int(params["rrt_global_iterations"]),
            local_iterations=int(params["rrt_local_iterations"]),
            local_radius=params["rrt_local_radius"],
            seed=int(params["rrt_seed"]) or None,
        )
        self._scoring = {
            "cluster_radius": params["cluster_radius"],
            "revalidate_radius": params["revalidate_radius"],
            "gain_radius": params["gain_radius"],
            "gain_weight": params["gain_weight"],
            "cost_weight": params["cost_weight"],
            "min_gain": int(params["min_gain"]),
            "hysteresis": params["hysteresis"],
            "hysteresis_radius": params["hysteresis_radius"],
        }

    def restart(self, robot_xy):
        """Throw both trees away and regrow them from the robot's position."""
        self.search.restart(robot_xy)

    def update(self, context, current_goal=None):
        """Grow the trees one cycle and write the candidates into the tick."""
        if context.grid is None or context.robot_xy is None:
            return FAILURE
        points = self.search.step(context.grid, context.robot_xy)
        context.best, context.scored = frontier_tools.best(
            context.grid,
            points,
            context.robot_xy,
            current_goal=current_goal,
            **self._scoring
        )
        return SUCCESS
