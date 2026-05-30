# Debug checklist

## Build
```bash
cd ~/manchester
colcon build --packages-select puzzlebot_sim2 --symlink-install
source install/setup.bash
```

## Package found?
```bash
ros2 pkg list | grep puzzlebot_sim2
ros2 pkg prefix puzzlebot_sim2
```

## Executables found?
```bash
ros2 pkg executables puzzlebot_sim2
```
Expected:
```text
puzzlebot_sim2 simulator
puzzlebot_sim2 localisation
puzzlebot_sim2 joint_states
puzzlebot_sim2 final_bug_nav
```

## Base stack check
```bash
ros2 launch puzzlebot_sim2 sim_base.launch.py
```
Then check:
```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 run rqt_graph rqt_graph
```

## Navigation check
Before running navigation, verify LaserScan:
```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

If `/scan` does not exist, `final_bug_nav` cannot avoid obstacles.

## Common symptoms

### Robot visible in RViz but does not move
Likely no `/scan`, or `final_bug_nav` is waiting for `/odom` and `/scan`.

### Two nodes publish `/cmd_vel`
Do not run the old point controller together with `final_bug_nav`. The final navigation node must be the only active `/cmd_vel` publisher.

### RViz opens but RobotModel is missing
Check that `/robot_description` exists and that the fixed frame is `map`.

### TF exists but model jumps or is offset
Check the TF chain: `map -> odom -> base_footprint -> base_link -> wheels/caster`.
