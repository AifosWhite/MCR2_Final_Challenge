# Team handoff: MCR2 Final Challenge package

## Goal
This package keeps the final challenge work isolated from the original Manchester packages. The team should work inside:

```bash
~/manchester/src/finalchallenge/puzzlebot_sim2
```

Do not edit the original `puzzlebot_sim` unless we explicitly decide to port a fix back.

## What each part does

- `simulator.py`: kinematic differential-drive simulator. It reads `/cmd_vel` and publishes `/wr`, `/wl`, `/pose_sim`, `/sim_x`, `/sim_y`, `/sim_theta`.
- `localisation.py`: odometry/localisation node. It reads `/wr` and `/wl`, integrates the robot pose, propagates covariance, and publishes `/odom`.
- `joint_states.py`: publishes `/joint_states`, `/tf`, and `/tf_static` so the URDF robot model moves in RViz.
- `final_bug_nav.py`: final navigation controller. It reads `/odom` and `/scan`, then publishes `/cmd_vel`.
- `urdf/puzzlebot.urdf`: robot model.
- `rviz/final_challenge.rviz`: RViz layout for the presentation/video.
- `config/final_bug_nav.yaml`: navigation parameters and waypoints.
- `config/localisation.yaml`: odometry and covariance parameters.

## Launch files

### Base visual test
Use this first to verify that the robot model, TF, odometry, and covariance are visible.

```bash
ros2 launch puzzlebot_sim2 sim_base.launch.py
```

### Navigation only
Use this when the Manchester/Gazebo world is already running and publishing `/scan`, `/odom`, and accepting `/cmd_vel`.

```bash
ros2 launch puzzlebot_sim2 navigation_only.launch.py
```

### Full local stack
Starts the local simulator, localisation, RViz, rqt_graph, and final navigation node.
This only performs obstacle avoidance if `/scan` is also available in the graph.

```bash
ros2 launch puzzlebot_sim2 final_challenge.launch.py
```

## Important current limitation
The clean local simulator does not create `/scan`. That means Bug navigation will start, but it will not move until a LaserScan source exists. This is expected behavior, not necessarily a code crash.

Check with:

```bash
ros2 topic list | grep scan
```

If nothing appears, run the Manchester/Gazebo world that has LiDAR, or add a LaserScan sensor/plugin to the simulation.

## Workflow for the team

1. Pull or copy the package.
2. Build with `colcon build --packages-select puzzlebot_sim2 --symlink-install`.
3. Source `install/setup.bash`.
4. Run `sim_base.launch.py` and verify RViz.
5. Run `scripts/check_ros_graph.sh` from the package folder.
6. Only then test `navigation_only.launch.py` with the real Gazebo environment that publishes `/scan`.

## Parameters to tune first

Edit `config/final_bug_nav.yaml`:

- `waypoints_x`, `waypoints_y`, `waypoints_theta`: final closed trajectory.
- `front_clearance`: how close an obstacle can be before switching to wall following.
- `wall_follow_safety`: desired wall distance.
- `max_linear_speed`, `max_angular_speed`: safety limits.
- `bug_direction`: use `fwcw` or `fwccw` depending on which side gives cleaner obstacle circumnavigation.
- `bug_mode`: `2` is preferred for the final challenge because it uses the M-line idea from Bug2.

## What to show in the video

Use RViz and rqt_graph to explain the workflow:

`/cmd_vel -> simulator -> /wr,/wl -> localisation -> /odom -> final_bug_nav -> /cmd_vel`

With Gazebo/LiDAR, add:

`/scan -> final_bug_nav`

Explain that covariance grows during pure odometry because wheel integration accumulates uncertainty. When the robot moves longer without external correction, the odometry ellipse grows. This connects directly to the final challenge requirement about analysing growing covariance.
