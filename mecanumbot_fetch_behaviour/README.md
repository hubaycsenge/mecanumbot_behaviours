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
        │   │       ├── HoldSearchGaze            always RUNNING — head still at fetch_head_search
        │   │       │   (or SweepHead)            fetch_head_search_mode: sweep — the old up-and-down sweep
        │   │       ├── WatchForBall              the only child that can end the parallel
        │   │       └── SpinAndHop  (repeat)       fetch_search_strategy: spin (default)
        │   │           └── TurnThenMove
        │   │               ├── FullCircle        one revolution where it stands (Spin360, head left alone)
        │   │               └── HopToNextSpot     nav2 to the next spot; fails when the laps run out
        │   │       (or CircleSearch)             fetch_search_strategy: circles — widening circles, no stops
        │   ├── SecureBall
        │   │   └── RetryTheGrab  ×fetch_grasp_attempts
        │   │       ├── CloseInOnBall             parallel, SuccessOnSelected([DriveAndFace])
        │   │       │   ├── TrackBallWithHead     always RUNNING — neck keeps the ball centred vertically
        │   │       │   └── DriveAndFace
        │   │       │       ├── ApproachBall      nav2, re-aimed as the estimate refines
        │   │       │       └── FaceBall          in-place turn until the ball is centred left-to-right
        │   │       ├── CheckBallReachable        is it on the floor, or on a table?
        │   │       └── GraspBall                 close (head where the tracker left it), read has_object
        │   ├── HandOver
        │   │   ├── FindSomeone                   SomebodyIsHere, else LookForSomebody (scan)
        │   │   ├── GoToThePerson                 Approach(SUBJECT, fixed_distance)
        │   │   ├── FaceThePerson                 TurnToward(SUBJECT) — lifts the head, grabbers stay shut
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
with the check it failed (`score`, `size`, `shape`, `unconfirmed`). With
`camera_source:=direct`, the default, the detector opens the webcam itself (no camera
node, no ROS image topic in the frame path), and this is the only view of the camera
there is. (`camera_source:=topic` makes the detector read `/camera/image_raw/compressed`
but does not start a publisher for it. Start `mecanumbot_camera_stream`'s
`camera_compressed.launch.py` by hand.) It costs a copy of
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

**The body search is a strategy, `fetch_search_strategy`.** The default, `spin`, turns a
full circle where the robot stands (`FetchScan`, the library's `Spin360` at
`full_scan_spin_speed`), drives to the next spot (`HopToNextSpot`), and turns a full circle
again. The spots are rings around where the search began, nearest first, laid out with the
same `expanding_circles` as the circling search but sparser (`fetch_spot_*`): a spot is
looked round from, so neighbours need only be about two detection ranges apart. A lap is
every spot once — 9 spots in both rooms as shipped — and after `fetch_search_laps` laps the
round is given up. `fetch_search_timeout` was raised to 900 s for it in both shipped files
(the packaged default in `defaults.py` is still 300 s), since one lap is ten turns of
~25 s plus the drives.

`spin` replaced `circles` as the default on 2026-09-14 because the circles missed balls: a
tangent-facing robot only ever looks along its direction of travel, so a ball lying beside
it is never in view. A full turn looks at every bearing; at `full_scan_spin_speed: 0.3` a
bearing stays in the 51° view for about 3 s. (With `fetch_head_search_mode: sweep` the
turn must also stay below ~0.35 rad/s, or a ball passes through the view while the head
looks at the wrong band.)

**`CircleSearch` (`circles`) covers the floor.** Widening circles around wherever the robot was
standing when it started looking. A circle rather than a lawnmower sweep because the
constraint is the camera's ~51°, not the floor: what matters is ending up pointed in
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

**The head is held still, at the tilt that sees the most floor (`HoldSearchGaze`).** The
camera is on the neck about 0.2 m up with a vertical field of view of about 30°. That is
low enough that **one tilt sees nearly all of the floor**: with the top edge of the frame
just above the horizon, the bottom edge meets the floor about 0.35 m in front of the lens,
and everything from there to the far wall is in view at once. So there is no band left
over for a sweep to cover. Holding the horizon inside the frame is also what finds a ball
**as far away as possible**, because a far ball sits just below the horizon.
`fetch_head_search` is that tilt, in board units.

`fetch_head_search: 5.5` (550 ticks) is −9.8° by the calibration below: the top edge
+5° above the horizon, the floor in view from ~0.36 m out. `5.4` gains a few centimetres
of near floor for a 2° horizon margin. `fetch_head_low: 4.0` (−53°) looks at the floor
5–17 cm beyond the lens, where a ball about to be gripped is.

Until 2026-09-15 the head swept up and down between `fetch_head_low` and `fetch_head_high`
(`SweepHead`, still available as `fetch_head_search_mode: sweep`). The sweep made the
ball *harder* to find, in three ways:

* Half of each lap pointed the camera at the ceiling or at the grabbers, where no ball on
  the floor can be.
* Every frame came from a moving camera, so the ball was smeared.
* The fusion node places each ball with the neck's *goal*: the firmware echoes the last
  command as `pos_n` and never reads the servo. Mid-sweep the goal is ahead of the head,
  so a ball on the floor was placed along a ray steeper than the real one. On
  2026-09-15 that put a ball 63 cm *under* the floor, and `CheckBallReachable` threw away
  every round it found.

When a sighting ended the search, `SweepHead` also sent the head to `fetch_head_low`,
which pointed the camera at the robot's own feet. A ball seen across the room was out of
frame before the approach started. Neither head behaviour moves the head at the end of
the search now; the ball was just seen at the tilt the head is at.

`ball.range_source` in the perception layer stays on `size`, whose range does not depend
on the tilt. The ball's **height** does, because it is that range along a ray the tilt
points. So the fusion node places every frame with the neck position it was taken at
(`ball.neck.*` there), and that position is only right while the head is still, and
only if `ball.neck.pitch_at_level_deg` is (see below).

## From the sighting to the grab, the ball stays in the middle of the picture

`TrackBallWithHead` runs beside the approach and tilts the neck to keep the ball centred
vertically. `FaceBall` then turns the body in place until the ball is centred
left-to-right. Both close the loop on the ball's **pixel box**
(`/mecanumbot/cam_ball_boxes`), not on its map position. A pixel offset becomes an angle
with nothing but the lens's field of view; a map position also needs the neck's
calibration, which is the number that has been wrong. So the centring works whatever the
neck's zero is.

* **The head** takes a step of `fetch_head_track_gain` × the ball's elevation. It ignores
  frames stamped within `fetch_head_track_settle` of its last move, because those were
  taken with the head still moving, and correcting on them makes a visual servo
  oscillate. As the robot closes in, the ball sinks in the frame and the head follows it
  down. If the ball goes out of view (`fetch_head_track_lost`) while the robot is within
  `fetch_head_close_range` of it, it has gone under the lens, and the head drops to
  `fetch_head_low`, the pose that looks just beyond the lens. Otherwise the head holds
  where it is.
* **The body** turns on `/cmd_vel`, in place, proportional to the ball's bearing, until
  it is within `fetch_face_tolerance` on two frames. nav2's rotation shim already faces a
  ball the approach drives to, but not one closer than `fetch_approach_stop`. For those,
  `pose_to_goal` returns the robot's own pose, nav2 reports the goal reached at once,
  and the robot would grab at a ball off to one side. `FaceBall` never fails: when the
  ball is out of view for `fetch_face_lost` or the turn times out, the grab goes ahead on
  the approach's heading.

The tracker stops before `CheckBallReachable` and `GraspBall`, because every neck command
is also a gripper command and the grab owns the grippers. `GraspBall` keeps the head
where the tracker left it.

**The grabbers stay shut while the head moves during delivery.** Every accessory message
carries the neck and both grippers, and the movement library's head lifts (`FindPeople`,
`TurnToward(SUBJECT)`) sent `gripper_*_neutral`, which in these files are the open
positions. So the delivery opened the grabbers the moment it looked up for a person.
`GraspBall` now hands the closed positions to every later head command
(`GripperCommander.keep_grippers`), and `ReleaseBall` and `ClearFetchEpisode` hand the
open ones back.

## The neck and lens calibration (measured 2026-09-15)

`CheckBallReachable` turns on the ball's height, and that height is only as right as
the neck model and the lens. Both were measured on 2026-09-15 with a ball on the floor
about 1 m in front of the robot:

* **Neck:** the neck was held still at 500–600 ticks in 0.1-unit steps, up and then
  down, and every box went through `ball_locating.floor_pitch`. The slope is the AX-12A's
  0.005061 rad/tick (a free fit gave 0.005115). At 600 ticks the camera looks **+4.7°**
  up (+4.4° coming down, +5.0° going up, so a couple of ticks of backlash), which puts
  optical level at ~584 ticks. It was 0 before.
* **Lens:** **51°** horizontal, not the assumed 60°. The robot turned ±0.2 rad in place,
  and the ball's pixel shift against odometry yaw gave a ~1340 px focal length. At 60°
  the neck slope came out 1.2× the datasheet's and the ball ranged 20 % short; at 51°
  both are right.

The offset is set in all three places that share the model:
`ball.neck.pitch_at_level_deg` in `mecanumbot_sensorprocess_smart`, `head_joint` in
`mecanumbot_sensorproc_node` (the TF), and `camera.pitch_at_level_deg` in
`mecanumbot_deep3r`. The lens is `camera_hfov_deg` in `perception.launch.py`, and
`fetch_camera_hfov_deg` here. The centring does not depend on the offset, but the
reachability check does.

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

Neck positions (`fetch_head_search`, `fetch_head_low`, `fetch_head_high`) are the
accessory board's own units, about 2.0 to 8.6, where larger looks further up. They are
**not** angles, because they are the numbers the neck is commanded in (the calibration to
radians is in the section above), so they
carry no `_deg` suffix. The centring angles (`fetch_camera_hfov_deg`,
`fetch_head_track_deadband_deg`, `fetch_face_tolerance_deg`) are angles in the image,
and do. `fetch_head_approach` is gone: the grab keeps the head where the tracker left it.

## Topics

| Topic | Direction | Type | Use |
| --- | --- | --- | --- |
| `/mecanumbot/ball_detections` | in | `vision_msgs/Detection3DArray` | where the ball is, in `map`, with a score and a height |
| `/mecanumbot/cam_ball_boxes` | in | `vision_msgs/Detection2DArray` | the same balls as pixel boxes; what the head and `FaceBall` centre on |
| `/mecanumbot/people_fusion` | in | `geometry_msgs/PoseArray` | who is there to give it to |
| `/amcl_pose` | in | `geometry_msgs/PoseWithCovarianceStamped` | where the robot is |
| `/mecanumbot/has_object` | in | `std_msgs/Bool` | whether anything is actually held |
| `/mecanumbot/fetch/state` | out | `std_msgs/String` | phase label per transition, for the trial record |
| `/cmd_accessory_pos` | out | `mecanumbot_msgs/AccessMotorCmd` | search tilt, ball tracking, grabbers |
| `navigate_to_pose` | out | action | every drive |
| `/cmd_vel` | out | `geometry_msgs/Twist` | in-place turns only: the scans, and `FaceBall` |

`/mecanumbot/fetch/state` publishes `searching`, `approaching`, `delivering`,
`handover`, `abandoned`. A `std_msgs/String` and not a new message type on purpose:
`mecanumbot_msgs` is deliberately small and is for data no standard type covers, and a
phase name is a label.

## Files

| File | Function |
| --- | --- |
| `tree_nodes/fetch_tree.py` | The tree. Every structural decision is argued in its module docstring. |
| `tree_nodes/tree_common.py` | The package name and the node name; everything else is `mecanumbot_bt_config`'s. |
| `search_patterns.py` | The circles (and the spots, laid out the same way), the search strategies and the head sweep. Pure geometry, no ROS. |
| `gaze.py` | Where the camera points: the floor band one tilt sees, image offsets, the neck step and the turn rate that centre a ball. Pure geometry, no ROS. |
| `defaults.py` | Every tunable and the value it has when the YAML does not say. |
| `keys.py` | Binds `fetch_ball_position` onto the movement library's `target_position`: `FetchApproach`, `FetchTurnToward`, `FetchFindPeople`, `FetchScan`. |
| `behaviours/searching.py` | `WatchForBall`, `HoldSearchGaze`, `SweepHead`, `CircleSearch`, `HopToNextSpot`. |
| `behaviours/centring.py` | `TrackBallWithHead`, `FaceBall`. |
| `behaviours/approach.py` | `ApproachBall`, `CheckBallReachable`, `GraspBall`. |
| `behaviours/delivery.py` | `SomeoneToGiveTo`, `ReleaseBall`, `BackAway`. |
| `behaviours/signalling.py` | `AnnouncePhase`. |
| `behaviours/blackboard_managers.py` | Constants loading, required keys, per-episode state. |
| `behaviours/ros_interfaces.py` | `BallDetectionTracker`, `BallBoxTracker`, `GripperCommander`, `FetchStatePublisher`. |
| `test/test_search_patterns.py` | 21 tests over the circles and the sweep; no ROS needed. |
| `test/test_gaze.py` | 19 tests over the search tilt's floor band and the centring steps; no ROS needed. |

## Tests

```bash
cd src/mecanumbot_behaviours/mecanumbot_fetch_behaviour
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_search_patterns.py test/test_gaze.py -q -p no:launch_testing
```

`colcon test` is broken workspace-wide on this machine (the installed `launch_testing`
pytest plugin is incompatible with the installed pytest), and so is `test_flake8` (the
installed `flake8` cannot load its own `pycodestyle` plugin). `test_pep257` passes.
