"""
Pose, angle and checkpoint helpers shared by all leading behaviours.

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
    """
    Rotation [rad] that turns `current_yaw` into `desired_yaw`.

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
    """
    How far along the checkpoint polyline `position` lies, as a float index.

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
    """
    Say whether `other_position` is further along the path than the robot.

    Returns `1` when it lies ahead (towards the target) and `-1` when it lies
    behind or level with the robot. Used to decide which way along the route a
    lost human is searched for, so the tie falls to `-1`: back the way the pair
    came, where somebody who dropped behind is most likely to be.
    """
    robot_progress = route_progress(checkpoints, robot_position)
    other_progress = route_progress(checkpoints, other_position)
    return 1 if other_progress > robot_progress else -1


def human_is_ahead_on_route(checkpoints, robot_position, person_position, margin=0.0):
    """
    Say whether `person_position` is further along the route than the robot.

    `margin` is how much further, as a fraction of the stretch between two
    checkpoints, so that two people standing level do not flip the answer back
    and forth. It is the same quantity `path_progress_sign` answers without a
    margin, and the tie falls the same way: level, or behind, is not ahead.

    This exists as its own function because the question is easy to ask wrongly.
    Comparing the person's progress -- a continuous position along the polyline
    -- against the *index* of the checkpoint nearest the robot compares a
    fraction with an integer, and reads "ahead" for anybody standing more than
    `margin` past the last checkpoint the robot happens to be nearest, whether
    or not the robot itself is further along than they are. That is what
    `DogResumeLeading` did until 2026-09-23, and it let the robot lead on while
    the human it had just found was still behind it.
    """
    if not checkpoints:
        return False
    robot_progress = route_progress(checkpoints, robot_position)
    person_progress = route_progress(checkpoints, person_position)
    return person_progress > robot_progress + margin


def resume_checkpoint_index(checkpoints, robot_position, person_position, margin=0.0):
    """
    Return the checkpoint to lead on to once the robot has its human back.

    The first checkpoint ahead of the pair along the route: ahead of the robot,
    or ahead of the human when they are further along than the robot by more
    than `margin` (the `human_is_ahead_on_route` question). A pair standing
    between two checkpoints carries on to the later one -- the earlier one has
    been walked past already, and leading back to it is leading backwards.

    Until 2026-09-30 `DogResumeLeading` resumed at the checkpoint *nearest* the
    robot unless the human was ahead, so a pair standing a fifth of the way past
    a checkpoint was led back to it: in the 08:46 run of that day the robot
    found its human at progress 0.26 and 2.25, and turned round to walk them to
    checkpoints 0 and 2. `person_position` may be None, when the robot is all
    there is to go by.
    """
    if not checkpoints:
        return 0
    index = _next_checkpoint(checkpoints, route_progress(checkpoints, robot_position))
    if person_position is not None and human_is_ahead_on_route(
        checkpoints, robot_position, person_position, margin
    ):
        # At most one checkpoint further on. The route folds back on itself, so
        # somebody standing in the middle of the room can project onto a stretch
        # far down it -- 2.70 against the robot's 0.19 in that same run -- and
        # following the projection all the way would skip most of the walk.
        person = _next_checkpoint(checkpoints, route_progress(checkpoints, person_position))
        index = min(max(index, person), index + 1, len(checkpoints) - 1)
    return index


def _next_checkpoint(checkpoints, progress):
    """Return the first checkpoint beyond a route progress value."""
    # Before the start of the route (the projection clamps to 0.0) the start
    # itself is still ahead; anywhere along it, the next checkpoint is.
    if progress <= 0.0:
        return 0
    return min(int(math.floor(progress)) + 1, len(checkpoints) - 1)


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


def route_poses(checkpoints, indices, look_beyond=None):
    """
    Goal poses for a run through `indices` of the checkpoint list.

    Each pose sits on its checkpoint and faces the next point of the route, so a
    robot handed the whole leg at once arrives at every waypoint already
    pointing the way it is about to go -- there is no turn-on-the-spot between
    one waypoint and the next. `look_beyond` is what the final checkpoint of the
    route faces (the target, where the human is being led); without it that
    checkpoint keeps the heading the robot came in on.
    """
    poses = []
    for index in indices:
        point = checkpoints[index]
        if index + 1 < len(checkpoints):
            yaw = bearing_to(point, checkpoints[index + 1])
        elif look_beyond is not None:
            yaw = bearing_to(point, look_beyond)
        elif index > 0:
            yaw = bearing_to(checkpoints[index - 1], point)
        else:
            yaw = 0.0
        pose = Pose()
        pose.position.x = float(point.x)
        pose.position.y = float(point.y)
        pose.orientation = quaternion_from_yaw(yaw)
        poses.append(pose)
    return poses


def approach_arrived(distance, stop_threshold, margin=0.0):
    """
    Say whether a robot `distance` metres from a person has walked up to them.

    It has when it stands within `stop_threshold` -- the closest it may come --
    plus `margin`, which absorbs the goal tolerance nav2 parks within. This is
    the question a stepped approach asks after every step; before 2026-09-24 the
    dog tree never asked it, took one step of `approach_distance` and called the
    person reached from 3 m away.
    """
    return distance <= stop_threshold + margin


def at_route_end(distance, reached_distance, stop_distance, margin=0.0):
    """
    Say whether a robot `distance` metres from the last checkpoint stands at it.

    A drive to the end of the route finishes in one of two places: on the
    checkpoint, within `reached_distance` of it, or parked `stop_distance` short
    of it by a single goal. Either is the end of the route; `margin` absorbs the
    goal tolerance nav2 parks within, so a drive nav2 calls finished is never
    asked for again. Anything further out is a robot still on its way.
    """
    return distance <= max(reached_distance, stop_distance) + margin


def pose_to_goal(
    object_position, robot_pose, stop_threshold=0.3, mode="exact", go_threshold=1.0
):
    """
    Nav2 goal pose towards `object_position`, always facing it.

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
