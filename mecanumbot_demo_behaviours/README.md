# mecanumbot_demo_behaviours

`py_trees_ros` behaviour-tree package for Mecanumbot demos.

Unlike `mecanumbot_leading_behaviour` — which runs the scripted leading experiment
conditions — this package holds open-ended demo trees: the robot looks for people and
visits them, or patrols a map and intercepts detections. It reuses the leading
package's behaviour library and adds its own movement, blackboard and map-waypoint
helpers.

## Executable nodes

| Executable                   | Source file                           | Main behavior                                                                                                         |
| ---------------------------- | ------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `wander_between_people_node` | `tree_nodes/wander_between_people.py` | Spin until people are detected, then repeatedly drive to a randomly chosen person and greet them with an LED pattern. |

`tree_nodes/hide_and_seek.py` implements a patrol-and-intercept tree but has **no
entry point in `setup.py`**, so it is not installed as an executable. Run it as a
module or add an entry point before using it.

## Node interfaces

The ROS interfaces are created inside the BT behaviour classes and used by the
executable tree nodes.

### Publishers

| Topic        | Data type                       | Function                                    |
| ------------ | ------------------------------- | ------------------------------------------- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Navigation goal toward the selected person. |
| `/cmd_vel`   | `geometry_msgs/msg/Twist`       | In-place spin while searching for people.   |

`hide_and_seek.py` instead sends goals through a `py_trees_ros` `ActionClient` on the
`/navigate_to_pose` action, reading the goal from the `intercept_goal` /
`patrol_goal` blackboard keys.

### Subscribers

| Topic                              | Data type                                     | Processing                                                                       |
| ---------------------------------- | --------------------------------------------- | -------------------------------------------------------------------------------- |
| `/mecanumbot/people_fusion`        | `geometry_msgs/msg/PoseArray`                 | Fused people detections; freshness is checked against a sight timeout.           |
| `/amcl_pose`                       | `geometry_msgs/msg/PoseWithCovarianceStamped` | Robot pose used to build the approach goal (`RELIABLE` + `TRANSIENT_LOCAL` QoS). |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray`             | Tracks the sent goal by UUID and retries on abort/cancel.                        |
| `/detections`                      | `geometry_msgs/msg/PoseArray`                 | `hide_and_seek.py` only — pushed onto the blackboard by a pre-tick handler.      |

### Services handled

| Service | Type | Behavior                           |
| ------- | ---- | ---------------------------------- |
| None    | -    | Nodes do not provide ROS services. |

### Service clients used

| Service                      | Type                               | Function                                                                |
| ---------------------------- | ---------------------------------- | ----------------------------------------------------------------------- |
| `/mecanumbot/set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Greeting LED pattern, via the leading package's `LEDBehaviourSequence`. |

## Behaviour library

### `behaviours/movement_managers.py`

| Behaviour           | Role                                                                                                                                                                             |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `FindPeople`        | Spins in place (0.2 rad/s) until a `people_fusion` message newer than the sight timeout arrives, then stops the robot.                                                           |
| `GoToRandomPerson`  | Picks a random fresh detection, builds a goal with the leading package's `pose_to_goal` (stop threshold 0.5 m), publishes it and follows the Nav2 goal status until it succeeds. |
| `ProcessDetections` | `hide_and_seek` — turns the first pose of the blackboard `pose_array` into `intercept_goal`; FAILURE when there is nothing to intercept, so the tree falls back to patrolling.   |
| `GetNextWaypoint`   | `hide_and_seek` — cycles through the blackboard `waypoints` list, writing each in turn to `patrol_goal`.                                                                         |

The module re-exports `Approach`, `TurnToward`, `RelativeTurnPattern`,
`CheckRobotAtLastCheckpoint`, `CheckRobotHasBall`, `CheckSubjectTargetSuccess`, the
`pose_to_goal` / `calculate_facing_orientation` helpers and the `STATUS_*` constants
from `mecanumbot_leading_behaviour`.

### `behaviours/blackboard_managers.py`

| Behaviour                    | Role                                                                                                                                                                                         |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ConstantParamsToBlackboard` | Local copy of the leading package's YAML constants loader.                                                                                                                                   |
| `DistanceToBlackboard`       | Local copy of the robot/subject/target distance calculator.                                                                                                                                  |
| `MapWaypointsToBlackboard`   | Loads `<map>_waypoints.yaml` from `mecanumbot_description/maps/<map>/`, generating it with `utils/map_generate.py` if it does not exist yet, and writes the waypoint list to the blackboard. |

Note that `wander_between_people.py` imports `ConstantParamsToBlackboard` from
`mecanumbot_leading_behaviour`, not from this local copy.

### `utils/map_generate.py`

Standalone waypoint generator: reads a map `.pgm` plus its `.yaml`, ray-casts a
visibility score for every free cell, extracts the highest-visibility points with a
minimum separation, converts them to Nav2 world coordinates and writes
`<map>_waypoints.yaml`. Waypoint count, minimum separation and maximum ray range are
auto-derived from the map resolution and free area when not given explicitly. Requires
OpenCV (`cv2`).

## Launch files

`launch/launch_wifi_condition_sequence.launch.py` is a copy of the leading package's
launcher (SSID-based YAML selection, `params` / `yaml_path` / `namespace` /
`condition` arguments, `YAML_PATH` and `BEHAVIOUR_YAML_PATH` env vars). It still
starts `doglike_demo_bt_node`, `control_demo_bt_node` or `LED_demo_bt_node` — none of
which exist in this package's `setup.py`, so the launch file fails as written. Use
`ros2 run` until it is repointed at `wander_between_people_node`.

## Folder structure

| Path                                     | Role                                                                                                      |
| ---------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `mecanumbot_demo_behaviours/tree_nodes/` | Top-level BT compositions and executable entry points.                                                    |
| `mecanumbot_demo_behaviours/behaviours/` | Demo-specific BT leaf behaviours plus local copies of the blackboard managers.                            |
| `mecanumbot_demo_behaviours/utils/`      | Map waypoint generation and subtree construction helpers.                                                 |
| `launch/`                                | Copy of the condition-based launcher (see the caveat above).                                              |
| `config/`                                | Behaviour constants in YAML (`behaviour_setting_constants.yaml`, `Eto_behaviour_setting_constants.yaml`). |
| `resource/`                              | ROS package resource marker.                                                                              |
| `test/`                                  | Package lint tests.                                                                                       |

## BT logic

### `wander_between_people.py`

Root is a memory sequence:

1. `ConstantParamsToBlackboard` — load the YAML constants (falls back to
   `Eto_behaviour_setting_constants.yaml` from the **leading** package's share
   directory when `YAML_PATH` / `BEHAVIOUR_YAML_PATH` are unset).
2. `FindPeople` — spin until somebody is detected.
3. `GoingToRandomPerson` sequence — `GoToRandomPerson`, then a `catch_attention` LED
   sequence as a greeting.

Ticked at 10 ms.

### `hide_and_seek.py`

Root is a memory-less priority selector:

1. `MapWaypointsToBlackboard` — load or generate the waypoints for the `AI_dept` map.
2. Intercept branch — `ProcessDetections` then a Nav2 `ActionClient` on
   `intercept_goal`. Runs whenever `/detections` is non-empty; `py_trees_ros` cancels
   the in-flight patrol goal automatically.
3. Patrol branch — `GetNextWaypoint` then a Nav2 `ActionClient` on `patrol_goal`.

Ticked at 100 ms, with a `ToBlackboard` pre-tick handler pushing `/detections` onto
the blackboard.

## Configuration model

`config/*.yaml` uses the same schema as `mecanumbot_leading_behaviour` — see that
package's README for the full key list. Only the LED sequence keys and the general
thresholds matter for the demo trees.

## Run examples

```bash
ros2 run mecanumbot_demo_behaviours wander_between_people_node

# with an explicit constants file
YAML_PATH=/absolute/path/to/behaviour_setting_constants.yaml \
  ros2 run mecanumbot_demo_behaviours wander_between_people_node
```

Both trees expect Nav2 with AMCL localized on a map. `wander_between_people_node`
additionally needs the people-detection pipeline publishing
`/mecanumbot/people_fusion` and the `mecanumbot_led` service node for the greeting.
