# mecanumbot_bt_config

Constants YAML → py_trees blackboard, without a schema. Every behaviour package
in this repository used to carry a list of the names its constants file was
allowed to contain — `SCALAR_PARAMS`, `ANGLE_PARAMS`, `LIST_PARAMS`,
`INTEGER_TUNABLES` — which is the same information the YAML already carries,
written down a second time and free to disagree with it. This package takes the
names from the file instead.

No nodes, no launch files, no config: it is a library, and it is the only
package here that knows nothing about the robot.

| Item | Value |
| --- | --- |
| Nodes | None |
| Launch files | None |
| Depends on | `python3-yaml`, `rclpy`, `py_trees`, `ament_index_python`; `geometry_msgs` / `mecanumbot_msgs` optionally, for the shipped decoders |

## Modules

| Module | Holds |
| --- | --- |
| `params.py` | Reading a parameter file: root-key discovery, the `_deg` convention, type coercion, which keys are missing. Imports nothing from ROS. |
| `decoders.py` | Turning the structured strings in a constants file into messages, chosen by the keys inside them. |
| `blackboard.py` | `ParamsToBlackboard` (the behaviour every tree starts with), `Tunables` (how a behaviour reads a constant back), `ConfiguredTimer`. |
| `tree_runner.py` | Finding a tree's YAML (`--yaml_path`, `YAML_PATH`, packaged fallback) and spinning it. |

## The three conventions

**The root key is found, not declared.** A ROS parameter file nests everything
under `<node_name>: ros__parameters:`. The node name is read off the file, so a
constants file works whatever its tree is called. This is what used to fork the
loader in two: `mecanumbot_ostensive_behaviour` had a byte-for-byte copy of
`mecanumbot_leading_behaviour`'s, differing only in the string
`bottom_up_tree_node`. Pass `root_keys=("some_node", "ros__parameters")` to name
the block explicitly; a file holding blocks for two nodes is an error rather
than a guess.

**`_deg` means degrees.** A parameter whose name ends in `_deg` reaches the
blackboard in radians under the name without the suffix, because an angle is
tuned in degrees and used in radians. No list of which parameters are angles
exists anywhere: `glance_sweep_deg: 25.0` in the file is `glance_sweep` on the
blackboard, and a package's default for it is written `math.radians(25.0)`.

**Structure is decoded by shape.** ROS parameter files cannot nest under a
single key, so the constants files write a structure out as a Python literal in
a string. Which kind of thing it is comes from the keys inside it, not from the
name of the parameter holding it:

| Literal contains | Becomes |
| --- | --- |
| `X`, `Y` | `geometry_msgs/Point` |
| `fl`, `fr`, `bl`, `br` | `SetLedStatus.Request` |
| `n_pos`, `gl_pos`, `gr_pos` | `AccessMotorCmd` |

Anything else parses to a plain dictionary; anything that is not a literal is
left alone. A package adds its own with `decoders.register(keys, builder)`. The
message imports are optional, so `params.py` stays importable — and testable —
with no ROS environment.

## What a file cannot say about itself

Two things, and they are the whole of what a tree still declares:

* **`defaults`** — the value a key keeps when the file does not declare it. This
  is why a constants file written before a tunable existed still loads. Defaults
  belong to the behaviours that read them, so they live in
  `mecanumbot_movement_behaviours.defaults` and each experiment package's
  `behaviours/defaults.py`, and are handed to the loader as a list of mappings
  (later mappings win).
* **`required`** — keys the file must declare. A missing one fails the setup
  rather than being invented: a run with no defined approach distance is not a
  run with a plausible one.

A value that has a default is also converted to that default's type, so a count
written `3.0` arrives as an `int` and a threshold written `5` as a `float`.
Keys with neither a default nor a requirement are written exactly as the file
has them.

## `ParamsToBlackboard`

```python
ParamsToBlackboard(
    name="LoadConstantParams",
    yaml_path=yaml_path,
    defaults=(RUNTIME_DEFAULTS, MOVEMENT_DEFAULTS, MY_DEFAULTS),
    required=("robot_closeness_threshold", "Dog_checkpoints"),
    state={"search_spin_sign": 0},          # written in initialise()
    on_loaded=(configure_accessories,),     # (node, blackboard, values)
    extra_keys=("start_position",),         # keys a hook writes
    root_keys=None,                         # None: find it in the file
)
```

* the file is **read at construction**, because which keys to claim write access
  to is the file's answer, not the caller's. A file that cannot be read is
  reported from `setup`, where `py_trees_ros` makes a failure visible;
* constants are **written in `setup`**, so they are there before any other
  behaviour is ticked;
* `state` is **written in `initialise`**, so re-entering the tree starts the
  run's own state over rather than resuming against stale values;
* `on_loaded` hooks run after the constants are written, for settings that have
  to reach something other than the blackboard —
  `AccessoryCommander.configure()`, splitting a checkpoint list into a route.
  Anything a hook writes to the blackboard must be named in `extra_keys`.

Undeclared keys are logged as a single line naming them, so a run says which of
its numbers came from the file and which are packaged defaults.

## `Tunables`

Each behaviour package builds one around its own defaults and exports the bound
methods, so its behaviours' call sites carry no defaults argument:

```python
_TUNABLES = Tunables(MOVEMENT_DEFAULTS)
constant = _TUNABLES.constant                    # blackboard, or the default
resolve = _TUNABLES.resolve                      # call-site value, or the constant
file_constant = _TUNABLES.file_constant          # loaded YAML block, no blackboard yet
register_param_keys = _TUNABLES.register_param_keys
```

`file_constant` exists for the handful of values settled while the tree is being
built — the tick period, a decorator's retry count — which is before any
blackboard exists. `Tunables(A, B)` merges, later mappings winning, which is how
a package extends another's tunables with its own.

## `tree_runner`

```python
run_tree(create_root, tree_name, package_name, node_name, default_yaml)
```

`RUNTIME_DEFAULTS` (`tick_period_ms`, `setup_timeout`) are the only key names
this package knows, and they are here because they belong to the runner: a tree
that has not been built yet has no blackboard to read them from.

## Tests

`test/test_params.py` — 22 tests over root-key discovery, the degree
convention, defaulting and coercion, required keys and shape decoding. They
import `params` and `decoders` only, so they run with nothing sourced:

```bash
cd src/mecanumbot_behaviours/mecanumbot_bt_config
PYTHONPATH=. python3 -m pytest test/test_params.py -v
```

`test_flake8` fails workspace-wide for environment reasons; see the workspace
`CLAUDE.md`.
