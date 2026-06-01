#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("puzzlebot_sim2")

    run_rqt = LaunchConfiguration("run_rqt")
    run_localisation = LaunchConfiguration("run_localisation")
    invert_bearing = LaunchConfiguration("invert_bearing")

    adapter = Node(
        package="puzzlebot_sim2",
        executable="jetson_aruco_adapter",
        name="jetson_aruco_adapter",
        output="screen",
        parameters=[
            {"input_topic": "/marker_publisher/markers"},
            {"output_topic": "/aruco/detections"},
            {"max_detection_distance": 3.0},
            {"invert_bearing": invert_bearing},
        ],
    )

    localisation = Node(
        package="puzzlebot_sim2",
        executable="localisation",
        name="localisation",
        output="screen",
        condition=IfCondition(run_localisation),
        parameters=[
            os.path.join(pkg_dir, "config", "localisation.yaml"),
        ],
    )

    rqt = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view"],
        output="screen",
        condition=IfCondition(run_rqt),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_rqt", default_value="true"),
            DeclareLaunchArgument("run_localisation", default_value="true"),
            DeclareLaunchArgument("invert_bearing", default_value="false"),
            adapter,
            localisation,
            rqt,
        ]
    )