# puzzlebot_sim2 — clean package for MCR2 Final Challenge

This package is intended to be copied into:

```bash
~/manchester/src/finalchallenge/puzzlebot_sim2
```

It keeps the final challenge work separated from the original Manchester packages.

## Main structure

```text
puzzlebot_sim2/
├── config/                         # Parameters for localisation and navigation
├── docs/                           # Team handoff and debug checklist
├── launch/                         # Clean launch files
├── meshes/                         # Puzzlebot STL meshes
├── puzzlebot_sim2/                 # Python nodes
├── rviz/                           # RViz configuration
├── scripts/                        # Team helper scripts
├── urdf/                           # Puzzlebot robot description
├── package.xml
└── setup.py
```

## Install

```bash
cd ~/manchester/src
mkdir -p finalchallenge
cp -r /path/to/puzzlebot_sim2 ~/manchester/src/finalchallenge/
cd ~/manchester
colcon build --packages-select puzzlebot_sim2 --symlink-install
source install/setup.bash
```

## Launches

Base visual/localisation test:

```bash
ros2 launch puzzlebot_sim2 sim_base.launch.py
```

Full local stack:

```bash
ros2 launch puzzlebot_sim2 final_challenge.launch.py
```

Navigation only, for use with an external Manchester/Gazebo world:

```bash
ros2 launch puzzlebot_sim2 navigation_only.launch.py
```

## Current limitation

`final_bug_nav.py` requires `/scan`. The local clean simulator does not generate LiDAR data by itself. If `/scan` is missing, navigation will wait and publish zero velocity.

Check:

```bash
ros2 topic list | grep scan
```

## Team docs

Read these before testing:

```text
docs/TEAM_HANDOFF.md
docs/DEBUG_CHECKLIST.md
```
