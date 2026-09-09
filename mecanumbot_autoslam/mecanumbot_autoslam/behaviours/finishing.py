"""
When the pass is over -- and, just as much, when it is not allowed to be.

The rule itself lives in `mecanumbot_custom_nav2/exit_criteria.py`, which argues
its shape at length and has 173 tests behind it:

    finished = (FRONTIERS or GAIN) and STABLE and CLOUD, or BUDGET

This behaviour is what feeds it and what happens when it says yes. It owns the
`ExplorationProgress` history because that history is the pass's, not the map's:
it measures newly-observed cells per *metre driven*, so a robot standing still
waiting for a plan contributes no evidence either way, and a loop closure resets
its settle timer because slam_toolbox re-rasterises the whole grid and a fresh
crop of frontiers appears a second later. Exiting into that is how a run ends
with half a map.

**Finishing latches `exploration/finished`.** That is the handover: T2 is
started by `mecanumbot_deep3r` seeing the latch, not by a wall-clock guess about
how long a room takes.

**The battery reading is now actually supplied.** `min_battery_voltage` was a
declared exit criterion that nothing fed, so the one budget meant to protect a
whole session -- a flat battery mid-reconstruction loses the cloud, which lives
on the server against a map id that will not survive the robot coming back --
could never fire. It defaults to 0.0, which means no limit, so a run that does
not set it behaves exactly as before.
"""

from sensor_msgs.msg import BatteryState

from mecanumbot_custom_nav2.exit_criteria import ExitCriteria, ExplorationProgress

from mecanumbot_autoslam.behaviours.base import RUNNING, SUCCESS, Behaviour


def _positive(value):
    """Read a budget constant, where 0 means "no limit"."""
    return None if not value else float(value)


class FinishExploration(Behaviour):
    """Evaluate the exit criteria every tick, and latch when they are met."""

    def __init__(self, name="finish_exploration"):
        """Create the behaviour; the criteria are built in `setup()`."""
        super().__init__(name)
        self.progress = None
        self.criteria = None
        self.finished = False
        self.trigger = ""
        self.battery_voltage = None

    def setup(self, node, params):
        """Build the progress history, the criteria and the battery watch."""
        super().setup(node, params)
        self.progress = ExplorationProgress(window=params["progress_window"])
        self.criteria = ExitCriteria(
            frontier_quiet_time=params["frontier_quiet_time"],
            min_cells_per_metre=params["min_cells_per_metre"],
            max_growth_fraction=params["max_map_growth"],
            loop_closure_settle=params["loop_closure_settle"],
            min_agreement=params["min_agreement"],
            cloud_reset_settle=params["cloud_reset_settle"],
            min_runtime=(params["min_runtime"] or None),
            min_grid_coverage=params["min_grid_coverage"],
            max_uncertain_regions=int(params["max_uncertain_regions"]),
            uncertain_score_threshold=params["uncertain_score_threshold"],
            max_cloud_growth=params["max_cloud_growth"],
            cloud_verdict_timeout=params["cloud_verdict_timeout"],
            require_cloud=params["require_cloud"],
            max_duration=_positive(params["max_duration"]),
            max_distance=_positive(params["max_distance"]),
            min_battery_voltage=_positive(params["min_battery_voltage"]),
        )
        if params["battery_topic"]:
            node.create_subscription(
                BatteryState, params["battery_topic"], self._on_battery, 10
            )

    def _on_battery(self, msg):
        self.battery_voltage = float(msg.voltage)

    def update(self, context, cloud=None, elapsed=0.0):
        """Feed the criteria this tick, and finish if they are satisfied."""
        if context.loop_closed:
            self.progress.note_loop_closure(context.now)
        self.progress.add(
            context.now,
            0 if context.grid is None else context.grid.known_cells(),
            context.distance,
        )
        self.criteria.note_frontiers(context.now, len(context.scored))

        verdict = self.criteria.evaluate(
            context.now,
            self.progress,
            cloud=cloud,
            elapsed=elapsed,
            distance=context.distance,
            battery_voltage=self.battery_voltage,
        )
        context.verdict = verdict

        if not verdict.finished:
            return RUNNING

        self.finished = True
        self.trigger = verdict.trigger
        self.log(
            "T1 finished ({}), trigger={}: {}".format(
                "complete" if verdict.complete else "cut short",
                verdict.trigger,
                verdict.summary(),
            )
        )
        return SUCCESS
