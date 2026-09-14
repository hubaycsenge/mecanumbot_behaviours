# mecanumbot_fetch_behaviour

Playing fetch. The robot looks for a tennis ball, goes and gets it, and takes it to the
first person it can see.

One round is: **search → secure → hand over → rest**, repeated for ever.

```text
ROOT (memory)
├── LoadFetchParams
└── FetchLoop  (repeat for ever)
    └── EpisodeOrRelease  (selector — a failed round is not a stopped game)
        ├── FetchEpisode
        │   ├── FindBall
        │   │   └── SearchUntilSighted            parallel, SuccessOnSelected([WatchForBall])
        │   │       ├── SweepHead                 always RUNNING — tilts the neck up and down
        │   │       ├── WatchForBall              the only child that can end the parallel
        │   │       └── CircleSearch              widening circles; fails when the laps run out
        │   ├── SecureBall
        │   │   └── RetryTheGrab  ×fetch_grasp_attempts
        │   │       ├── ApproachBall              nav2, re-aimed as the estimate refines
        │   │       ├── CheckBallReachable        is it on the floor, or on a table?
        │   │       └── GraspBall                 close, settle, read /mecanumbot/has_object
        │   ├── HandOver
        │   │   ├── FindSomeone                   SomebodyIsHere, else LookForSomebody (scan)
        │   │   ├── GoToThePerson                 Approach(SUBJECT, fixed_distance)
        │   │   ├── FaceThePerson                 TurnToward(SUBJECT) — lifts the head
        │   │   ├── OfferPause / ReleaseBall / LetThemTakeIt
        │   │   └── BackOffIfWeCan → BackAway
        │   ├── EndEpisode
        │   └── RestBeforeTheNext
        └── AbandonEpisode
```

## What it needs running

The tree runs **no detector of its own**. It reads `/mecanumbot/ball_detections`
(`vision_msgs/Detection3DArray`, `map` frame), which `mecanumbot_locate_detections`
publishes from the fetch camera detector's bounding boxes.

```bash
# on the robot: drivers, nav2, the GUI -- no perception
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py

# the tree, which starts the FETCH detector and the fusion for itself
ros2 launch mecanumbot_fetch_behaviour launch_fetch.launch.py

# watch it
ros2 topic echo /mecanumbot/fetch/state
ros2 topic echo /mecanumbot/ball_fusion

# and see what the detector sees (debug_image:=false turns it off)
ros2 run rqt_image_view rqt_image_view /mecanumbot/cam_object_detections/debug_image/compressed
```

The launch starts the detector with `debug_image:=true`, so it publishes the frame the
network was given with every box on it: people blue, balls yellow, and a refused box red
with the check it failed (`score`, `size`, `shape`, `unconfirmed`). With `use_camera`
false, the default, this is the only view of the camera there is. It costs a copy of
the frame out of GPU memory and a JPEG encode per frame; switch it off for runs that
need the GPU and CPU headroom.

The launch file includes `mecanumbot_sensorprocess_smart`'s `perception.launch.py` with
`detector:=fetch`; `use_perception:=false` when it is already running.

The detector choice is not optional and not an upgrade. A pose network has exactly one
class, so there is no threshold at which it starts finding tennis balls; a plain
detector has no skeletons, so the ostensive gestures are unavailable while it is the one
in use; and on an Orin Nano running both at once is most of the GPU. That is also why
this game and the ostensive experiment cannot share a perception pipeline. See
`mecanumbot_sensorprocess_smart/README.md`.

It also needs **nav2 with AMCL localized against the room's map**. Every drive here is a
nav2 goal — the tree never drives the wheels for navigation, exactly like every other
tree in this repository.

## Why the search is a parallel

The same argument `mecanumbot_seek` makes. In a sequence, "have we seen the ball?" would
be a check between search steps, and the check would only fire at the seams — the robot
would drive right past a ball it could see and notice on arrival. As a parallel,
`WatchForBall` is doing nothing else for the whole search, so wherever the robot happens
to be, a sighting ends the parallel, `terminate()` cancels the nav2 goal in flight, and
the approach starts from where the robot is standing.

`SuccessOnSelected([watch])` rather than `SuccessOnOne`, for the same reason too: with
`SuccessOnOne` the parallel would also succeed if the *search* branch succeeded, and the
tree would fall into the approach with nothing sighted.

## The search has two dimensions, and they need two behaviours

**`CircleSearch` covers the floor.** Widening circles around wherever the robot was
standing when it started looking. A circle rather than a lawnmower sweep because the
constraint is the camera's ~60°, not the floor: what matters is ending up pointed in
every direction from a spread of places, and going round is the cheapest way to do that
one nav2 goal at a time. Everything at radius *r* is looked at before anything at
*r + step*, so the near floor comes first.

Which way the robot faces at each stop is `fetch_circle_facing`, and it is a **design
choice about what the robot attends to**, not a tuning constant. `tangent` (the default)
faces the direction of travel, sweeping the camera over floor it has not looked at yet,
which is right when the robot has no idea where the ball is. `inward` circles a place
and stares at it — that is `mecanumbot_seek`'s pattern, and it is right only when there
is a remembered location to circle. `outward` searches the walls.

Each lap starts again from the innermost circle rather than continuing outwards for
ever. That is deliberate: this is a game with a person in it, and the ball that was not
at the robot's feet a minute ago is quite likely to be there now, because somebody threw
it. A search that only ever widens is a search for a stationary object, which is
`mecanumbot_seek`'s problem and not this one.

**`SweepHead` covers the height band.** The camera is on a tilting neck about 0.2 m up
with a vertical field of view of roughly 36°, so at any one tilt it sees a band of floor
and nothing else: tilted down it sees from its own feet out to a couple of metres; level
it sees the far wall but not the floor in front of it. A ball outside the current band is
invisible however good the detector is, so a search with a fixed head is a search for
balls at one distance. The sweep is a **triangle** wave and not a sine, because a sine
lingers at both ends and hurries through the middle — and the middle of this range is the
band a ball two metres away sits in.

`ball.range_source` in the perception layer stays on `size`, whose range does not depend
on the tilt. The ball's **height** does — it is that range along a ray the tilt points —
so the fusion node places every frame with the neck position it was taken at
(`ball.neck.*` there), not with one fixed pitch. Placed as if the camera looked level, a
ball on the floor seen with the head down comes out as high as the camera, and
`CheckBallReachable` calls it out of reach.

## Height is what decides whether the ball can be had

`CheckBallReachable` compares the located ball's `z` against
`fetch_grasp_height_min`/`_max`. That band is **hardware, not taste**: the grabbers are a
horizontal pincer whose shafts sit at about z = 0.034 with a 0.116 m clear gap, and there
is no lift. A ball on a table, on a chair or in somebody's hand is one the robot cannot
have however close it drives — and from a bearing alone, "the ball is on the table" and
"the ball is not here" are the same fact.

This is the same argument `mecanumbot_seek` makes about the Deep3R point cloud's height,
one sensor cheaper: `ball_locating.py` gets the height from the ball's apparent size,
because a tennis ball's known diameter turns a box into a range and a range into a ray
length.

The **response** to an unreachable ball is where the two trees part. Seek finds a person
and shows them the object, because it was sent to find that specific thing and an object
located but not obtained is news. A ball out of reach in a fetch game is not news — the
person can see it, they are in the room playing — so this tree gives up on that ball and
goes looking for another one.

## Why the grab is retried around the *approach*

The usual way a grab fails is that the ball rolls off the shafts as they close. That
leaves the robot half a metre from a ball that has *moved*, so repeating the grab alone
would close the grabbers on empty floor. `RetryTheGrab` wraps approach, check and grab
together, with a fresh detection behind each attempt.

## Why this tree does not model SEEKING

`mecanumbot_seek` runs a Panksepp SEEKING circuit, and everything about the shape of this
tree — an appetitive search, a widening pattern, giving up — looks like it should too. It
deliberately does not, and the reason is taxonomic rather than technical.

Fetch with a person is **PLAY**. It is social, it is reciprocal, it needs a partner, and
in Panksepp's scheme it is a separate primary-process system with its own
neurochemistry and its own developmental story. Driving it with a SEEKING circuit would
have the thesis claim that fetching a ball *for somebody* is the same motivation as
foraging for an object, which is exactly the collapse the wiki records as one not to
make.

So the search here is a plain pattern with a lap count — honestly a piece of
engineering — and the emotional model is left open. If a PLAY circuit is modelled later
it belongs in this package as `seeking.py`'s counterpart, and the two places it would
attach are `CircleSearch`'s lap limit and `WatchForBall`'s threshold: exactly where
`SeekingDrive` attaches in the other tree.

## Configuration

`config/fetch_setting_constants.yaml`, with `config/Eto_fetch_setting_constants.yaml`
for the smaller LED_exp room; the launch file picks between them by Wi-Fi SSID, the same
convention every other tree here uses. The root key is the node name, `fetch_bt_node`.

Constants are loaded by `mecanumbot_bt_config`, which fills in a packaged default for
anything the file leaves out (`defaults.py`) and refuses to start when a `required` key
is missing (`behaviours/blackboard_managers.py`). What is required is the small set that
describes this robot's body and the people in the room:

| Required key | Why it has no default |
| --- | --- |
| `fetch_approach_stop`, `fetch_grasp_distance` | where the robot parks and how close counts as grippable |
| `fetch_gripper_open_left/right`, `fetch_gripper_closed_left/right` | this robot's grabber positions |
| `robot_approach_distance`, `robot_closeness_threshold` | how close a robot may drive to a person |

Neck tilts (`fetch_head_low`, `fetch_head_high`, `fetch_head_approach`) are the accessory
board's own units — about 2.0 to 8.6, larger looks further up — and are **not** angles;
there is no calibration to radians, which is why they carry no `_deg` suffix.

## Topics

| Topic | Direction | Type | Use |
| --- | --- | --- | --- |
| `/mecanumbot/ball_detections` | in | `vision_msgs/Detection3DArray` | where the ball is, in `map`, with a score and a height |
| `/mecanumbot/people_fusion` | in | `geometry_msgs/PoseArray` | who is there to give it to |
| `/amcl_pose` | in | `geometry_msgs/PoseWithCovarianceStamped` | where the robot is |
| `/mecanumbot/has_object` | in | `std_msgs/Bool` | whether anything is actually held |
| `/mecanumbot/fetch/state` | out | `std_msgs/String` | phase label per transition, for the trial record |
| `/cmd_accessory_pos` | out | `mecanumbot_msgs/AccessMotorCmd` | neck sweep and grabbers |
| `navigate_to_pose` | out | action | every drive |
| `/cmd_vel` | out | `geometry_msgs/Twist` | in-place turns only, via the movement library |

`/mecanumbot/fetch/state` publishes `searching`, `approaching`, `delivering`,
`handover`, `abandoned`. A `std_msgs/String` and not a new message type on purpose:
`mecanumbot_msgs` is deliberately small and is for data no standard type covers, and a
phase name is a label.

## Files

| File | Function |
| --- | --- |
| `tree_nodes/fetch_tree.py` | The tree. Every structural decision is argued in its module docstring. |
| `tree_nodes/tree_common.py` | The package name and the node name; everything else is `mecanumbot_bt_config`'s. |
| `search_patterns.py` | The circles and the head sweep. Pure geometry, no ROS. |
| `defaults.py` | Every tunable and the value it has when the YAML does not say. |
| `keys.py` | Binds `fetch_ball_position` onto the movement library's `target_position`. |
| `behaviours/searching.py` | `WatchForBall`, `SweepHead`, `CircleSearch`. |
| `behaviours/approach.py` | `ApproachBall`, `CheckBallReachable`, `GraspBall`. |
| `behaviours/delivery.py` | `SomeoneToGiveTo`, `ReleaseBall`, `BackAway`. |
| `behaviours/signalling.py` | `AnnouncePhase`. |
| `behaviours/blackboard_managers.py` | Constants loading, required keys, per-episode state. |
| `behaviours/ros_interfaces.py` | `BallDetectionTracker`, `GripperCommander`, `FetchStatePublisher`. |
| `test/test_search_patterns.py` | 21 tests over the circles and the sweep; no ROS needed. |

## Tests

```bash
cd src/mecanumbot_behaviours/mecanumbot_fetch_behaviour
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_search_patterns.py -q -p no:launch_testing
```

`colcon test` is broken workspace-wide on this machine (the installed `launch_testing`
pytest plugin is incompatible with the installed pytest), and so is `test_flake8` (the
installed `flake8` cannot load its own `pycodestyle` plugin). `test_pep257` passes.
