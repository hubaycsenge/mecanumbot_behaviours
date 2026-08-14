"""
Read a constants YAML without knowing what is in it.

Every behaviour package used to carry a list of the names its constants file was
allowed to contain -- `SCALAR_PARAMS`, `ANGLE_PARAMS`, `LIST_PARAMS`,
`INTEGER_TUNABLES` -- which is the same information the YAML file already
carries, written down a second time and able to disagree with it. This module
takes the names from the file instead, so adding a constant is one line in one
place and no package needs a schema of its own.

Three conventions do the work the lists used to:

* **the root key is found, not declared.** A ROS parameter file nests everything
  under `<node_name>: ros__parameters:`, and the node name is whatever the tree
  registers as -- so it is read off the file rather than hard-coded, which is
  what used to fork this loader in two;
* **`_deg` means degrees.** A parameter whose name ends in `_deg` reaches the
  blackboard in radians under the name without the suffix, because a deadband or
  a step angle is tuned in degrees and used in radians;
* **structure is decoded by shape.** A string holding a dictionary literal is
  turned into the message its own keys identify -- see `decoders`.

What a file cannot say about itself is what should happen when a key is *not*
in it. That is the one thing a caller still declares: `defaults` for the keys
that keep a packaged value, `required` for the keys a run must not be missing.
Both belong to the behaviours that read them, not here.
"""

import math

import yaml

from mecanumbot_bt_config.decoders import decode

# A parameter name ending in this is declared in degrees and stored in radians.
DEGREE_SUFFIX = "_deg"

# The block a ROS parameter file keeps its parameters in.
PARAMETER_KEY = "ros__parameters"


def load_params(yaml_path, root_keys=None):
    """
    Read the parameter block out of a ROS parameter YAML file.

    `root_keys` names the path to the block for a file that does not follow the
    usual layout; left out, the block is found by looking for
    `ros__parameters`, whatever node name it happens to be nested under.
    """
    with open(yaml_path, "r") as handle:
        document = yaml.safe_load(handle)
    if document is None:
        raise ValueError(f"{yaml_path} is empty")

    if root_keys is not None:
        for key in root_keys:
            document = document[key]
        return document
    return _find_parameter_block(document, yaml_path)


def _find_parameter_block(document, yaml_path):
    """Descend to the `ros__parameters` block, whoever it is nested under."""
    if not isinstance(document, dict):
        raise ValueError(f"{yaml_path} does not hold a parameter block")
    if PARAMETER_KEY in document:
        return document[PARAMETER_KEY] or {}

    nodes = [key for key, value in document.items() if isinstance(value, dict)]
    if not nodes:
        raise KeyError(f"{yaml_path} has no '{PARAMETER_KEY}' block")
    if len(nodes) > 1:
        raise KeyError(
            f"{yaml_path} holds parameters for more than one node "
            f"({', '.join(sorted(nodes))}); name the one to load with root_keys"
        )
    return _find_parameter_block(document[nodes[0]], yaml_path)


def merge_defaults(defaults):
    """Flatten one dict, or several, into the single mapping the loader uses."""
    if defaults is None:
        return {}
    if isinstance(defaults, dict):
        return dict(defaults)
    merged = {}
    for entry in defaults:
        merged.update(entry)
    return merged


def blackboard_key(name):
    """Blackboard name a parameter is stored under -- `_deg` loses its suffix."""
    if name.endswith(DEGREE_SUFFIX):
        return name[: -len(DEGREE_SUFFIX)]
    return name


def blackboard_values(params, defaults=None):
    """
    Return every blackboard key a parameter block sets, with its value.

    Keys absent from the block keep their default, degrees become radians, and
    structured strings become the messages they describe. A value that has a
    default is converted to that default's type, so a count written `3.0` still
    arrives as an `int` and a threshold written `5` as a `float`.
    """
    defaults = merge_defaults(defaults)
    values = dict(defaults)
    for name, raw in params.items():
        key = blackboard_key(name)
        value = _convert(name, raw)
        values[key] = _coerce(value, defaults.get(key))
    return values


def missing_keys(params, keys):
    """Names in `keys` the parameter block does not declare, in the given order."""
    declared = {blackboard_key(name) for name in params}
    return tuple(key for key in keys if blackboard_key(key) not in declared)


def undeclared_keys(params, defaults):
    """Default-carrying keys the parameter block leaves out, sorted for a log line."""
    return tuple(sorted(missing_keys(params, tuple(merge_defaults(defaults)))))


def _convert(name, value):
    """Apply the degree and the structure conventions to one parameter."""
    if name.endswith(DEGREE_SUFFIX) and isinstance(value, (int, float)):
        return math.radians(float(value))
    return decode(value)


def _coerce(value, default):
    """Give a value the type of the default it replaces, where there is one."""
    if default is None or isinstance(value, type(default)):
        return value
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int):
        return int(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, str):
        return str(value)
    return value
