"""
What the movement behaviours call the things they read off the blackboard.

These behaviours are shared, but the blackboard keys they read are spelled by
whichever experiment loaded the constants file -- the leading conditions call
their route `Dog_checkpoints`, because that is the name in their YAML and the
loader writes what the file says. Hard-coding that spelling here would put one
experiment's vocabulary inside code every experiment uses.

So the spelling is a `KeyMap`, and a package that names things differently
derives one::

    LEADING_KEYS = DEFAULT_KEYS.derive(
        checkpoints="Dog_checkpoints",
        current_checkpoint="Dog_current_checkpoint",
    )

A behaviour takes `keys=` at construction, and a package that wants its spelling
everywhere binds it once on a subclass instead of passing it at every call site.
The *fields* are fixed -- they are what the behaviours mean -- so deriving with
a name that is not one of them is an error rather than a key nothing ever reads.
"""


class KeyMap:
    """The blackboard name each thing a movement behaviour reads is stored under."""

    FIELDS = {
        # --- the route --------------------------------------------------------
        "checkpoints": "checkpoints",
        "current_checkpoint": "current_checkpoint",
        "max_checkpoint": "max_checkpoint",
        # --- the search patrol over that same route ---------------------------
        "patrol_checkpoints": "patrol_checkpoints",
        "patrol_current_checkpoint": "patrol_current_checkpoint",
        "patrol_direction": "patrol_direction",
        "patrol_initialized": "patrol_initialized",
        # --- the places a run is defined by -----------------------------------
        "start_position": "start_position",
        "target_position": "target_position",
        # --- the distances a run is defined by --------------------------------
        "closeness_threshold": "robot_closeness_threshold",
        "approach_distance": "robot_approach_distance",
        "reached_threshold": "target_reached_threshold",
        "following_threshold": "following_max_threshold",
        "visibility_timeout": "visibility_time_threshold",
        # --- pacing of the look backs -----------------------------------------
        "checkpoints_since": "check_in_checkpoints_since",
        "last_check_in": "check_in_last_time",
        # --- handedness of the last turn made while looking for somebody -------
        "spin_sign": "search_spin_sign",
    }

    def __init__(self, **names):
        unknown = sorted(set(names) - set(self.FIELDS))
        if unknown:
            raise ValueError(
                f"unknown key field(s) {', '.join(unknown)}; "
                f"expected some of {', '.join(sorted(self.FIELDS))}"
            )
        self._names = {**self.FIELDS, **names}

    def __getattr__(self, field):
        """Return the blackboard name a field is spelled with."""
        try:
            return self.__dict__["_names"][field]
        except KeyError:
            raise AttributeError(field) from None

    def __repr__(self):
        """Show only the fields this map respells."""
        changed = {k: v for k, v in self._names.items() if v != self.FIELDS[k]}
        return f"KeyMap({', '.join(f'{k}={v!r}' for k, v in changed.items())})"

    def derive(self, **names):
        """Return a copy of this map with some fields spelled differently."""
        return KeyMap(**{**self._names, **names})

    def register(self, blackboard, access, *fields):
        """Give a blackboard client `access` to the named fields."""
        for field in fields:
            blackboard.register_key(key=getattr(self, field), access=access)


DEFAULT_KEYS = KeyMap()
