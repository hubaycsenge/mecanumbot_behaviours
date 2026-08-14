"""
Turn the structured strings in a constants YAML into the messages they describe.

ROS parameter files cannot hold a nested structure under a single key, so the
behaviour constants files write one out as a Python literal in a string::

    Dog_checkpoints:
      - "{'X':0.45, 'Y':-0.71, 'Z':0.0 }"
    LED_thank_seq:
      - "{'fl':{'mode':6, 'color':6}, ...}"
    Dog_thank_seq:
      - "{'n_pos':6.0,'gl_pos':6.83,'gr_pos':3.36}"

Which of those a given entry is, is decided **by the keys inside it** rather
than by the name of the parameter holding it. That is what lets the loader be
name-agnostic: a file may call its route anything it likes, and a package may
add a decoder of its own with `register`, without either of them having to be
listed anywhere in `params.py`.

The message types are imported optionally. Without a ROS environment the module
still imports and simply decodes nothing, which is what keeps `params.py`
testable on its own.
"""

import ast

# (required keys, builder). The first entry whose keys are all present in a
# decoded literal wins, so a more specific decoder has to be registered first.
_DECODERS = []


def register(keys, builder):
    """Register `builder` for literals containing every name in `keys`."""
    _DECODERS.append((frozenset(keys), builder))


def decode(value):
    """
    Return `value` with any structured string in it replaced by its message.

    Lists are decoded entry by entry, which is how the LED, gesture and
    checkpoint sequences arrive. A literal no decoder claims is handed back as
    the plain dictionary it parses to; anything that is not a literal at all is
    returned untouched.
    """
    if isinstance(value, list):
        return [decode(entry) for entry in value]
    literal = _literal(value)
    if literal is None:
        return value
    for keys, builder in _DECODERS:
        if keys <= literal.keys():
            return builder(literal)
    return literal


def _literal(value):
    """Parse a string holding a dictionary literal, or return None."""
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text.startswith("{"):
        return None
    try:
        parsed = ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return None
    return parsed if isinstance(parsed, dict) else None


# --- the decoders this workspace ships -------------------------------------

try:  # pragma: no cover - exercised only with a ROS environment
    from geometry_msgs.msg import Point
except ImportError:  # pragma: no cover
    Point = None

try:  # pragma: no cover
    from mecanumbot_msgs.msg import AccessMotorCmd
    from mecanumbot_msgs.srv import SetLedStatus
except ImportError:  # pragma: no cover
    AccessMotorCmd = None
    SetLedStatus = None


def _build_point(literal):
    """`{'X': .., 'Y': .., 'Z': ..}` -> geometry_msgs/Point."""
    point = Point()
    point.x = float(literal["X"])
    point.y = float(literal["Y"])
    point.z = float(literal.get("Z", 0.0))
    return point


def _build_led(literal):
    """`{'fl': {'color': .., 'mode': ..}, ...}` -> SetLedStatus request."""
    request = SetLedStatus.Request()
    for corner in ("fl", "fr", "bl", "br"):
        setattr(request, f"{corner}_color", literal[corner]["color"])
        setattr(request, f"{corner}_mode", literal[corner]["mode"])
    return request


def _build_gesture(literal):
    """`{'n_pos': .., 'gl_pos': .., 'gr_pos': ..}` -> AccessMotorCmd."""
    cmd = AccessMotorCmd()
    cmd.n_pos = float(literal["n_pos"])
    cmd.gl_pos = float(literal["gl_pos"])
    cmd.gr_pos = float(literal["gr_pos"])
    return cmd


if Point is not None:
    register(("X", "Y"), _build_point)
if SetLedStatus is not None:
    register(("fl", "fr", "bl", "br"), _build_led)
if AccessMotorCmd is not None:
    register(("n_pos", "gl_pos", "gr_pos"), _build_gesture)
