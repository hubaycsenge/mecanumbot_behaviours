# mecanumbot_fear_hri_behaviour

Fear HRI experiment package for `mecanumbot_behaviours`.

Subscribes to `/mecanumbot/scan`, updates `fear_level`, selects freeze / avoid / catch_attention / task.

## Build

```bash
colcon build --packages-select mecanumbot_fear_hri_behaviour
source install/setup.bash
ros2 launch mecanumbot_fear_hri_behaviour launch_fear_hri.launch.py
```

Branch: `fahrad/fear_hri_behaviour`
