"""
Putting a constants file on the py_trees blackboard, and reading it back off.

`ParamsToBlackboard` is the behaviour every tree starts with: it writes whatever
its YAML declares onto the blackboard, fills in the packaged defaults for what
the file leaves out, and refuses to run when a key the experiment needs is
missing. It knows no key names of its own -- `params.py` explains why, and what
a caller still has to declare.

`Tunables` is the other half. A behaviour reads a constant through
`constant()`, or through `resolve()` where a call site is still allowed to
override it, and gets the packaged default when the loaded file predates the
key. Each behaviour package builds one `Tunables` around its own defaults and
exports the four functions, so a behaviour's call sites say
`constant(self.blackboard, "turn_max_speed")` and nothing has to be told twice
which defaults apply.
"""

import py_trees

from mecanumbot_bt_config.params import (
    blackboard_values,
    load_params,
    merge_defaults,
    missing_keys,
    undeclared_keys,
)


class Tunables:
    """
    The defaults of one behaviour package, and the four ways of reading them.

    Build one per package and export its bound methods::

        _TUNABLES = Tunables(MOVEMENT_DEFAULTS)
        constant = _TUNABLES.constant
        resolve = _TUNABLES.resolve
        file_constant = _TUNABLES.file_constant
        register_param_keys = _TUNABLES.register_param_keys
    """

    def __init__(self, *defaults):
        self.defaults = merge_defaults(defaults)

    @property
    def keys(self):
        """Every key this package declares a default for."""
        return tuple(self.defaults)

    def merged_with(self, *others):
        """Return these defaults plus another package's, the others winning."""
        return Tunables(self.defaults, *others)

    def register_param_keys(self, blackboard, *keys):
        """Give a blackboard client read access to the tunables, or to named ones."""
        for key in keys or self.keys:
            blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def constant(self, blackboard, key, defaults=None):
        """
        Return a tunable from the blackboard, or its packaged default.

        The default covers a constants file older than the key, and a tree that
        borrows this package's behaviours while loading a constants file of its
        own, which declares only the tunables it actually means to set.
        """
        try:
            return getattr(blackboard, key)
        except (AttributeError, KeyError):
            defaults = self.defaults if defaults is None else defaults
            if key not in defaults:
                raise
            return defaults[key]

    def resolve(self, value, blackboard, key, defaults=None):
        """Return an explicit call-site value, or the configured constant for `key`."""
        if value is None:
            return self.constant(blackboard, key, defaults)
        return value

    def file_constant(self, params, key, defaults=None):
        """
        Return a tunable straight out of a loaded YAML block, or its default.

        For the few values needed while the tree is still being built -- the
        tick period, a decorator's retry count -- which is before any blackboard
        exists.
        """
        defaults = self.defaults if defaults is None else defaults
        if key in params:
            return params[key]
        return defaults[key]


class ParamsToBlackboard(py_trees.behaviour.Behaviour):
    """
    Load a constants YAML onto the blackboard, and seed the run's state.

    Loading happens in `setup`, so the constants are there before any other
    behaviour is ticked; the file itself is read at construction, because the
    keys it declares are the keys this behaviour has to claim write access to.

    Arguments beyond the YAML path are the declarations a file cannot make about
    itself:

    * `defaults` -- one mapping, or several, of key to the value that applies
      when the file does not declare it. Undeclared keys are logged, so a run
      says which of its numbers came from the file and which did not;
    * `required` -- keys the file must declare. A missing one fails the setup
      rather than being invented, because a run with no defined approach
      distance is not a run with a plausible one;
    * `state` -- blackboard keys that are not configuration but the run's own
      state, written in `initialise` so restarting the tree starts them over;
    * `on_loaded` -- callables `(node, blackboard, values)` run once the
      constants are written, for the settings that have to reach something other
      than the blackboard. Anything they write must be named in `extra_keys`.
    """

    def __init__(
        self,
        name,
        yaml_path,
        defaults=None,
        required=(),
        state=None,
        on_loaded=(),
        extra_keys=(),
        root_keys=None,
    ):
        super().__init__(name=name)
        self.yaml_path = yaml_path
        self.defaults = merge_defaults(defaults)
        self.required = tuple(required)
        self.state = dict(state or {})
        self.on_loaded = tuple(on_loaded)
        self.root_keys = root_keys

        # Read now: which keys to claim is the file's answer, not ours. A file
        # that cannot be read is reported from `setup`, where py_trees_ros makes
        # a failure visible, rather than from the tree's construction.
        self.params, self.load_error = self._read()
        self.values = blackboard_values(self.params, self.defaults)

        self.blackboard = self.attach_blackboard_client(name=name)
        for key in tuple(self.values) + tuple(self.state) + tuple(extra_keys):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Write every constant to the blackboard, and run the load hooks."""
        self.node = kwargs["node"]
        if self.load_error is not None:
            raise self.load_error

        absent = missing_keys(self.params, self.required)
        if absent:
            raise KeyError(
                f"{self.yaml_path} does not declare {', '.join(absent)}, which "
                "the tree has no default for"
            )

        for key, value in self.values.items():
            setattr(self.blackboard, key, value)

        for hook in self.on_loaded:
            hook(self.node, self.blackboard, self.values)

        undeclared = undeclared_keys(self.params, self.defaults)
        if undeclared:
            self.node.get_logger().info(
                f"{self.name}: {len(undeclared)} constant(s) not in the YAML, using "
                f"the packaged defaults: {', '.join(undeclared)}"
            )
        self.feedback_message = "constants loaded"
        self.node.get_logger().info(
            f"{self.name}: {len(self.values)} constant(s) loaded from {self.yaml_path}"
        )
        return True

    def initialise(self):
        """Seed the run's own state, so a restarted tree starts it over."""
        for key, value in self.state.items():
            setattr(self.blackboard, key, value() if callable(value) else value)

    def update(self):
        """Return SUCCESS -- the work was done in setup and initialise."""
        return py_trees.common.Status.SUCCESS

    def _read(self):
        try:
            return load_params(self.yaml_path, self.root_keys), None
        except Exception as error:  # unreadable, malformed, or not a param file
            return {}, error


class ConfiguredTimer(py_trees.timers.Timer):
    """
    A `py_trees` timer whose duration is a blackboard constant.

    `py_trees.timers.Timer` takes its duration at construction, which is while
    the tree is being built and therefore before any YAML has been read. This
    subclass looks the duration up when the timer starts instead, so a pause
    between two gestures is configured in the same file as the gestures.
    """

    def __init__(self, name, key, tunables=None):
        super().__init__(name=name, duration=0.0)
        self.key = key
        self.tunables = tunables or Tunables()
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def initialise(self):
        """Take the duration from the blackboard, then start the timer."""
        self.duration = float(self.tunables.constant(self.blackboard, self.key))
        super().initialise()
