# MCR2 Final Challenge - Puzzlebot Sim 2

Clean ROS2 package for the Manchester Robotics final challenge using the Puzzlebot.

The package is organized around a simple rule: first prove navigation, then add RViz, then add ArUco/EKF. Avoid launching every node at once while debugging.

## Main workflows

This package now keeps only two launch files:

- `bringup_simulation.launch.py` — launch Gazebo with the maze world and spawn the robot.
- `final_simulation_clean.launch.py` — launch the clean simulation workflow with the ROS nodes.

### 1. Gazebo launch

Use this to start Gazebo and spawn the robot in the world:

```bash
ros2 launch puzzlebot_sim2 bringup_simulation.launch.py
```

### 2. Clean simulation workflow

This is the recommended launch for development. It opens the maze and ArUco models in Gazebo, but the ROS algorithmic loop is lightweight and stable:

`simulator -> localisation -> sim_lidar / sim_aruco -> reactive_navigation -> cmd_vel`

Run without RViz:

```bash
ros2 launch puzzlebot_sim2 final_simulation_clean.launch.py
```

Run with RViz:

```bash
ros2 launch puzzlebot_sim2 final_simulation_clean.launch.py use_rviz:=true
```

Run with camera-based ArUco detection instead of simulated ArUco detections:

```bash
ros2 launch puzzlebot_sim2 final_simulation_clean.launch.py use_camera_arucos:=true use_sim_aruco:=false use_rviz:=true
```

## Important nodes

- `reactive_navigation_node.py`: simple Bug-style reactive navigation. Imported from the working Karifm logic because it is more robust for quick demos.
- `final_bug_nav.py`: more complete Bug navigation implementation. Keep it for later tuning.
- `localisation.py`: dead-reckoning odometry with optional ArUco correction and covariance publishing.
- `sim_lidar_node.py`: lightweight LaserScan simulator using the maze walls.
- `sim_aruco_node.py`: lightweight simulated ArUco detector that publishes `/aruco/detections`.
- `visualization_node.py`: publishes covariance, camera field of view and path as `/localisation_markers`.
- `aruco_detector.py`: camera-based ArUco detector. Keep optional because it is heavier.

## Expected topics in the clean workflow

```text
/cmd_vel
/odom
/scan
/aruco/detections
/localisation_markers
/joint_states
/tf
/tf_static
```

## Build

```bash
cd ~/manchester
colcon build --packages-select puzzlebot_sim2 --symlink-install
source install/setup.bash
```

## Debug order

1. Run `final_simulation_clean.launch.py` without RViz.
2. Confirm `/cmd_vel`, `/odom`, `/scan` exist.
3. Add RViz with `use_rviz:=true`.
4. Confirm `/localisation_markers` appears in RViz.
5. Only then test camera-based ArUco with `use_camera_arucos:=true`.

## Real robot note

For the real Puzzlebot, the expected topics are:

```text
/odom
/scan
/cmd_vel
/image_raw
/camera_info
```

Use the real-robot launch only after micro-ROS, RPLidar and camera topics are verified.
