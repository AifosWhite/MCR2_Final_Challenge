# MCR2 Final Challenge

ROS2 package for the Manchester Robotics final challenge using the Puzzlebot.

The project goal is to create a clean base for autonomous exploration in the Manchester Robotics simulator or real Puzzlebot setup. The package contains the Puzzlebot model, RViz visualisation, odometry/localisation nodes, ArUco localisation scaffolding, and Bug-based reactive navigation.

## Challenge scope

The final challenge requires two main components:

1. **Localisation** using odometry and camera-based ArUco marker observations.
2. **Unknown exploration / navigation** using a closed trajectory with at least four target points while avoiding obstacles.

This repository is organised so that the navigation, localisation, robot model, launch files and configuration files are easy for the team to edit independently.

## Repository structure

```text
config/              Parameter files
launch/              ROS2 launch files
meshes/              Puzzlebot STL meshes
puzzlebot_sim2/      Python ROS2 nodes
resource/            ament package marker
rviz/                RViz configuration
urdf/                Puzzlebot URDF model with LiDAR link
calibration/         Placeholder for camera calibration files
 tools/vision_reference/  Standalone ArUco/camera calibration reference scripts
package.xml
setup.py
setup.cfg
```

The repository is intentionally flat. It can be cloned directly inside the `src` folder of a ROS2 workspace.

## Main nodes

### `final_bug_nav.py`

Bug-based reactive navigation node. It subscribes to `/scan` and `/odom`, follows a waypoint sequence, avoids obstacles, and publishes velocity commands to `/cmd_vel`.

The waypoint trajectory is configured in:

```text
config/final_bug_nav.yaml
```

### `localisation.py`

Odometry-based localisation node using wheel angular velocities `/wr` and `/wl`. It publishes `/odom` with pose covariance. This is useful for local simulation and for showing covariance growth.

### `aruco_ekf_localisation.py`

Experimental EKF localisation node. It predicts the robot pose from wheel odometry and corrects the pose when ArUco marker transforms are available through TF.

The marker map is configured in:

```text
config/aruco_ekf.yaml
```

### `aruco_tf_listener.py`

Small validation node that listens to a marker TF, for example `base_link -> marker_0`, and prints the marker distance and bearing.

### `simulator.py`

Simple local differential-drive simulator for testing the robot without Gazebo. It does not create obstacles or a LaserScan topic.

### `joint_states.py`

Publishes wheel joint states and the base TF needed for RViz visualisation.

## Build

Clone the repository inside your workspace:

```bash
cd ~/manchester/src
git clone https://github.com/AifosWhite/MCR2_Final_Challenge.git
```

Build from the workspace root:

```bash
cd ~/manchester
colcon build --packages-select puzzlebot_sim2 --symlink-install
source install/setup.bash
```

## Launch files

### Local visual test

Use this when you only want to test the robot model, odometry, wheel joints and RViz without Gazebo.

```bash
ros2 launch puzzlebot_sim2 sim_base.launch.py
```

This launch does not publish `/scan`, so obstacle avoidance cannot be tested with it.

### Navigation only

Use this when the Manchester Gazebo environment is already running and provides `/scan`, `/odom`, and accepts `/cmd_vel`.

```bash
ros2 launch puzzlebot_sim2 navigation_only.launch.py
```

### Final challenge nodes

Use this when the simulator or real robot is already providing the sensor topics and you want to start the challenge nodes and RViz.

```bash
ros2 launch puzzlebot_sim2 final_challenge.launch.py
```

### Real robot ArUco test

Use this on the Puzzlebot Jetson when testing the camera and ArUco detection pipeline.

```bash
ros2 launch puzzlebot_sim2 aruco_jetson.launch.py
```

### ArUco EKF test

Use this after ArUco marker TFs are available.

```bash
ros2 launch puzzlebot_sim2 aruco_ekf.launch.py
```

## Required topics

For navigation:

```text
/scan       sensor_msgs/LaserScan
/odom       nav_msgs/Odometry
/cmd_vel    geometry_msgs/Twist
```

For odometry/localisation:

```text
/wr         std_msgs/Float32
/wl         std_msgs/Float32
/odom       nav_msgs/Odometry
```

For ArUco localisation:

```text
/video_source/raw              camera image
/camera_info                   camera calibration
/marker_publisher/markers      detected ArUco marker poses
/tf                            marker transforms
```

## LiDAR

The URDF contains a `lidar_link` and a Gazebo ray sensor block that remaps the sensor output to `/scan`. If the official Manchester Gazebo world already provides `/scan`, use that topic directly. If `/scan` is missing, obstacle avoidance cannot run correctly.

Check available topics with:

```bash
ros2 topic list
```

## Branch workflow

Use `main` for the shared stable version. Work on separate branches and merge only after testing.

Suggested branches:

```text
sofiadev
karim
feature/lidar-gazebo
feature/aruco-ekf
feature/final-navigation
feature/rviz-video
```

Typical workflow:

```bash
git checkout sofiadev
git pull origin main
# edit files
git status
git add .
git commit -m "Describe the change"
git push origin sofiadev
```
