#!/usr/bin/env python3
"""Minimal Gazebo-only navigation launch.

Use this only after the clean simulation is stable. This launch does not run the
Python simulator or sim_lidar; it expects Gazebo to provide /odom and /scan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    world_file = os.path.join(pkg_dir, 'worlds', 'maze.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    nav_params = os.path.join(pkg_dir, 'config', 'final_bug_nav.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'final_challenge_clean.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    resource_path = ':'.join([
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        pkg_dir,
        os.path.join(pkg_dir, 'models'),
        os.path.dirname(pkg_dir),
    ])

    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', 'default',
                    '-topic', 'robot_description',
                    '-name', 'puzzlebot',
                    '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    navigation = Node(
        package='puzzlebot_sim2',
        executable='final_bug_nav',
        name='final_bug_nav',
        output='screen',
        parameters=[nav_params, {'odom_topic': 'odom'}, {'scan_topic': 'scan'}, {'cmd_vel_topic': 'cmd_vel'}],
    )

    rviz = TimerAction(
        condition=IfCondition(use_rviz),
        period=3.0,
        actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_config])
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        navigation,
        rviz,
    ])
