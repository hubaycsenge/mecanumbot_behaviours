# mecanumbot_fear_hri_behaviour

ROS 2 package that activates Fear HRI on Mecanumbot simulation or hardware.

## What it does

1. Subscribes to LaserScan (`scan` / `/mecanumbot/scan`).
2. Updates continuous `fear_level` (spike, rise, slow decay).
3. Selects `freeze` / `avoid` / `catch_attention` / `task`.
4. Publishes `/cmd_vel` (stop, reverse, attention yaw) and logs fear each tick.

## Run with the Fear sim scene

```bash
# terminal 1
ros2 launch mecanumbot_bringup sim.launch.py \
  mode:=behaviour condition:=none scenario:=fear_hri_brick_carry

# terminal 2
ros2 launch mecanumbot_fear_hri_behaviour launch_fear_hri.launch.py
```

Expect logs: `fear=… risk=… range=… action=…`

## Build

```bash
colcon build --packages-select mecanumbot_fear_hri_behaviour
source install/setup.bash
```

Branch: `fahrad/fear_hri_behaviour` on `mecanumbot_behaviours`.
The same package is in https://github.com/fahradnamazov-ops/fear_mecanumbot under `ros_draft/`.
