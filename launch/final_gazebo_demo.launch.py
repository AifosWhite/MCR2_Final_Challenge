#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("puzzlebot_sim2")

    world_file = os.path.join(pkg_dir, "worlds", "maze.world")
    urdf_file = os.path.join(pkg_dir, "urdf", "puzzlebot.urdf")
    rviz_file = os.path.join(pkg_dir, "rviz", "final_challenge.rviz")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # Make Gazebo able to find local models: ArUcos, meshes, worlds.
    current_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    local_paths = [
        pkg_dir,
        os.path.join(pkg_dir, "models"),
        os.path.dirname(pkg_dir),
    ]
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        current_gz_path + ":" + ":".join(local_paths)
        if current_gz_path
        else ":".join(local_paths)
    )

    run_rviz = LaunchConfiguration("run_rviz")
    run_aruco = LaunchConfiguration("run_aruco")
    run_covariance = LaunchConfiguration("run_covariance")

    declare_run_rviz = DeclareLaunchArgument(
        "run_rviz",
        default_value="false",
        description="Open RViz together with Gazebo.",
    )

    declare_run_aruco = DeclareLaunchArgument(
        "run_aruco",
        default_value="true",
        description="Run camera-based ArUco detector.",
    )

    declare_run_covariance = DeclareLaunchArgument(
        "run_covariance",
        default_value="true",
        description="Run lightweight covariance visualisation node.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_puzzlebot",
        output="screen",
        arguments=[
            "-file", urdf_file,
            "-name", "puzzlebot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
            "-Y", "0.0",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # Bridges only Gazebo data that our ROS nodes need.
    # No simulator.py, no sim_lidar_node.py, no sim_aruco_node.py.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    navigation = Node(
        package="puzzlebot_sim2",
        executable="final_bug_nav",
        name="final_bug_nav",
        output="screen",
        parameters=[
            os.path.join(pkg_dir, "config", "final_bug_nav_gazebo.yaml"),
            {"use_sim_time": True},
        ],
    )

    aruco_detector = Node(
        package="puzzlebot_sim2",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        condition=IfCondition(run_aruco),
        parameters=[
            os.path.join(pkg_dir, "config", "aruco_detector_gazebo.yaml"),
            {"use_sim_time": True},
        ],
    )

    covariance_node = Node(
        package="puzzlebot_sim2",
        executable="gazebo_covariance_node",
        name="gazebo_covariance_node",
        output="screen",
        condition=IfCondition(run_covariance),
        parameters=[
            {"use_sim_time": True},
            {"odom_topic": "/odom"},
            {"output_odom_topic": "/ekf_odom"},
            {"marker_ids": [705, 706, 70, 703, 708, 75, 702]},
            {"base_frame": "base_link"},
            {"marker_frame_prefix": "marker_"},
            {"growth_rate_xy": 0.0008},
            {"growth_rate_theta": 0.0005},
            {"marker_correction_factor": 0.35},
            {"max_covariance_xy": 0.25},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(run_rviz),
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            declare_run_rviz,
            declare_run_aruco,
            declare_run_covariance,
            gazebo,
            bridge,
            robot_state_publisher,
            TimerAction(period=2.0, actions=[spawn_robot]),
            TimerAction(period=4.0, actions=[navigation]),
            TimerAction(period=4.5, actions=[aruco_detector, covariance_node]),
            TimerAction(period=5.0, actions=[rviz]),
        ]
    )

