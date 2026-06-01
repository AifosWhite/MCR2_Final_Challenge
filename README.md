# MCR2 Final Challenge - Puzzlebot Sim 2

Clean ROS2 package for the Manchester Robotics final challenge using the Puzzlebot.

The package is organized around a simple rule: first prove navigation, then add RViz, then add ArUco/EKF. Avoid launching every node at once while debugging.

## Main launch files

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

### 3. ArUco detector with USB camera

Launch the ArUco detector with USB camera for real-time marker detection:

```bash
ros2 launch puzzlebot_sim2 aruco_detector.launch.py
```

Optional parameters:

```bash
ros2 launch puzzlebot_sim2 aruco_detector.launch.py video_device:=/dev/video0 image_width:=640 image_height:=480 marker_size:=0.165
```

**What it does:**
- Captures video from USB camera at `/image_raw`
- Detects ArUco markers in real-time and publishes:
  - `/aruco/detections`: Closest marker info (id, distance, bearing) as Float32MultiArray
  - `/aruco/result`: Annotated image with detected markers drawn (green boxes)

**Viewing detected markers:**

In another terminal, run one of these to visualize the annotated image:

```bash
# Option 1: Use image_view (if installed)
ros2 run image_view image_view --ros-args --remap image:=/aruco/result

# Option 2: Use rqt
ros2 run rqt_image_view rqt_image_view
# Then select /aruco/result in the dropdown
```

**In terminal output**, look for messages like:
```
[#1] Detectados 1 IDs: [17]
Publishado: marker_id=17, distance=0.325m, bearing=0.123rad
```

## Important nodes

- `reactive_navigation_node.py`: simple Bug-style reactive navigation.
- `localisation.py`: dead-reckoning odometry with optional ArUco correction.
- `sim_lidar_node.py`: lightweight LaserScan simulator using the maze walls.
- `sim_aruco_node.py`: lightweight simulated ArUco detector.
- `aruco_detector.py`: camera-based ArUco detector using OpenCV.

## Expected topics

**Clean simulation workflow:**
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

**ArUco detector launch (USB or CSI camera):**
```text
/image_raw (or input from video source)
/aruco/detections (Float32MultiArray: [marker_id, distance, bearing])
/aruco/result (Image: annotated with detected markers drawn)
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
5. Test camera-based ArUco with `use_camera_arucos:=true`.
6. Use `aruco_detector.launch.py` for standalone ArUco detection with real camera.
