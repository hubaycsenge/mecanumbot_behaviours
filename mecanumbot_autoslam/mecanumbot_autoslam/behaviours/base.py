"""
The smallest thing that can be called a behaviour here, and the tick context.

These are **not** `py_trees` behaviours, and that is deliberate. An exploration
pass is one loop with one goal in flight at a time; there is no branch to
select, no parallel to run and no sub-tree to compose, so a tree would be
scaffolding around a sequence of four steps. What is worth borrowing from
`py_trees` is the *shape* -- a named object with `setup()` and `update()`, told
nothing except the tick it is in -- because that is what makes the pieces
testable one at a time and readable in the order they run.

`Context` is the tick. It is built fresh by the node every cycle and passed
down: each behaviour reads what the ones before it wrote, and the node reads the
whole thing to publish the state line. Nothing reaches back into the node.
"""

#: The behaviour did its job this tick and the next one may run.
SUCCESS = "success"

#: The behaviour is still working -- typically a goal that is still in flight.
RUNNING = "running"

#: The behaviour could not act: no map yet, no pose yet, nav2 not up.
FAILURE = "failure"


class Behaviour:
    """One step of an autoslam tick."""

    def __init__(self, name):
        """Name the behaviour; the name is what appears in the log."""
        self.name = name
        self.node = None

    def setup(self, node, params):
        """Create whatever ROS plumbing this behaviour owns. Called once."""
        self.node = node

    def update(self, context):
        """Run one tick. Returns SUCCESS, RUNNING or FAILURE."""
        raise NotImplementedError

    def terminate(self):
        """Stop cleanly. Called once, when the pass finishes or the node dies."""

    # --- logging -------------------------------------------------------------

    def log(self, message):
        """Log one line, tagged with this behaviour's name."""
        if self.node is not None:
            self.node.get_logger().info("[{}] {}".format(self.name, message))


class Context:
    """One tick: what is known now, and what the behaviours decided about it."""

    def __init__(self, now, grid, robot_xy, distance, loop_closed=False):
        """Build the tick from what the trackers currently hold."""
        self.now = now
        self.grid = grid
        self.robot_xy = robot_xy
        self.distance = distance
        self.loop_closed = loop_closed

        #: Filled in by DetectFrontiers.
        self.scored = []
        self.best = None
        #: Filled in by FinishExploration.
        self.verdict = None
        #: Filled in by DriveToGoal.
        self.goal = None
        self.goal_source = ""
