#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    nav_params = os.path.join(pkg_dir, 'config', 'final_bug_nav_real.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'aruco_ekf.yaml')

    camera = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='video_source',
        output='screen',
        parameters=[
            {'resource': 'csi://0'},
            {'width': 1280},
            {'height': 720},
            {'codec': 'unknown'},
            {'loop': 0},
            {'latency': 2000},
        ],
    )

    camera_info = Node(
        package='camera_info_publisher',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[
            {'camera_calibration_file': 'file:///home/puzzlebot/.ros/jetson_cam.yaml'},
            {'frame_id': 'camera'},
        ],
    )

    aruco_detector = Node(
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'image_topic': '/video_source/raw'},
            {'marker_length': 0.165},
            {'camera_frame': 'camera'},
            {'parent_frame': 'base_link'},
        ],
    )

    aruco_ekf = Node(
        package='puzzlebot_sim2',
        executable='aruco_ekf_localisation',
        name='aruco_ekf_localisation',
        output='screen',
        parameters=[ekf_params],
    )

    final_bug_nav = Node(
        package='puzzlebot_sim2',
        executable='final_bug_nav',
        name='final_bug_nav',
        output='screen',
        parameters=[nav_params],
    )

    return LaunchDescription([
        camera,
        camera_info,
        aruco,
        aruco_ekf,
        final_bug_nav,
    ])
