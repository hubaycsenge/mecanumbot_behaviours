"""Pose, angle and checkpoint helpers shared by all leading behaviours.

Everything here is pure geometry: no ROS communication, no py_trees. Keeping it
in one place means the behaviours only contain their decision logic.
"""

import math

from geometry_msgs.msg import Pose, Quaternion

# Rotation handedness, seen from above (right-handed z axis).
COUNTERCLOCKWISE = 1
CLOCKWISE = -1

# Below this the robot is considered "already facing there" and no turn is made,
# even when a turn direction was requested. The fallback for the `facing_epsilon`
# constant, which the turning behaviours pass in from the YAML.
FACING_EPSILON = math.radians(1.5)


def yaw_from_quaternion(q):
    """Yaw [rad] of a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """geometry_msgs Quaternion for a yaw-only rotation."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def bearing_to(from_position, to_position):
    """Absolute map-frame yaw [rad] pointing from one position to another."""
    return math.atan2(to_position.y - from_position.y, to_position.x - from_position.x)


def calculate_facing_orientation(robot_pose, target_position):
    """Orientation that makes the robot face `target_position` (absolute, map frame)."""
    return quaternion_from_yaw(bearing_to(robot_pose.position, target_position))


def signed_rotation(
    current_yaw, desired_yaw, preferred_sign=None, epsilon=FACING_EPSILON
):
    """Rotation [rad] that turns `current_yaw` into `desired_yaw`.

    Without a preference the shortest way round is taken. With `preferred_sign`
    (`COUNTERCLOCKWISE` / `CLOCKWISE`) the turn always goes that way round,
    taking the long way if it has to. That is what lets the robot unwind a
    search turn -- if it looked back for its human clockwise, it returns to its
    route counterclockwise instead of carrying on around.

    A rotation smaller than `epsilon` is reported as no rotation at all.
    """
    delta = normalize_angle(desired_yaw - current_yaw)
    if abs(delta) < epsilon:
        return 0.0
    if preferred_sign and (delta > 0.0) != (preferred_sign > 0):
        delta -= math.copysign(2.0 * math.pi, delta)
    return delta


def distance_xy(position_a, position_b):
    """Planar distance between two positions."""
    return math.hypot(position_b.x - position_a.x, position_b.y - position_a.y)


def closest_checkpoint_index(checkpoints, x, y):
    """Index of the checkpoint nearest to (x, y); 0 for an empty list."""
    closest_index = 0
    min_distance = float("inf")
    for index, checkpoint in enumerate(checkpoints):
        distance = math.hypot(checkpoint.x - x, checkpoint.y - y)
        if distance < min_distance:
            min_distance = distance
            closest_index = index
    return closest_index


def route_progress(checkpoints, position):
    """How far along the checkpoint polyline `position` lies, as a float index.

    `2.0` sits exactly on checkpoint 2 and `2.4` four tenths of the way from
    checkpoint 2 towards checkpoint 3, so comparing a progress value against a
    checkpoint index tells whether that checkpoint has already been walked past.
    Returns `0.0` for a route too short to have a segment.
    """
    best_progress, best_distance = 0.0, float("inf")
    for index in range(len(checkpoints) - 1):
        start, end = checkpoints[index], checkpoints[index + 1]
        fraction = _projection_fraction(start, end, position)
        offset_x = start.x + (end.x - start.x) * fraction - position.x
        offset_y = start.y + (end.y - start.y) * fraction - position.y
        distance = math.hypot(offset_x, offset_y)
        if distance < best_distance:
            best_distance = distance
            best_progress = index + fraction
    return best_progress


def path_progress_sign(checkpoints, robot_position, other_position):
    """Is `other_position` further along the checkpoint path than the robot?

    Returns `1` when it lies ahead (towards the target) and `-1` when it lies
    behind or level with the robot. Used to decide which way along the route a
    lost human is searched for, so the tie falls to `-1`: back the way the pair
    came, where somebody who dropped behind is most likely to be.
    """
    robot_progress = route_progress(checkpoints, robot_position)
    other_progress = route_progress(checkpoints, other_position)
    return 1 if other_progress > robot_progress else -1


def _projection_fraction(start, end, position):
    """Where `position` projects onto the segment `start` -> `end`, clamped to [0, 1]."""
    segment_x, segment_y = end.x - start.x, end.y - start.y
    length_squared = segment_x * segment_x + segment_y * segment_y
    if length_squared == 0.0:
        return 0.0
    fraction = (
        (position.x - start.x) * segment_x + (position.y - start.y) * segment_y
    ) / length_squared
    return max(0.0, min(1.0, fraction))


def pose_to_goal(
    object_position, robot_pose, stop_threshold=0.3, mode="exact", go_threshold=1.0
):
    """Nav2 goal pose towards `object_position`, always facing it.

    `mode="exact"` aims for the object minus `stop_threshold`, `mode="fixed_distance"`
    only steps `go_threshold` along the way (unless the remaining distance is
    shorter). Returns the current robot pose when it is already close enough.
    """
    distance = distance_xy(robot_pose.position, object_position)
    if distance < stop_threshold:
        return robot_pose

    remaining = distance - stop_threshold
    if mode == "fixed_distance" and go_threshold < remaining:
        ratio = go_threshold / distance
    else:
        ratio = remaining / distance

    goal = Pose()
    goal.position.x = (
        robot_pose.position.x + (object_position.x - robot_pose.position.x) * ratio
    )
    goal.position.y = (
        robot_pose.position.y + (object_position.y - robot_pose.position.y) * ratio
    )
    goal.orientation = quaternion_from_yaw(bearing_to(goal.position, object_position))
    return goal
