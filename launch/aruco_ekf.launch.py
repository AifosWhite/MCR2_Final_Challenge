import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    ekf_params = os.path.join(pkg_dir, 'config', 'aruco_ekf.yaml')

    aruco_ekf = Node(
        package='puzzlebot_sim2',
        executable='aruco_ekf_localisation',
        name='aruco_ekf_localisation',
        output='screen',
        parameters=[ekf_params],
    )

    aruco_tf_listener = Node(
        package='puzzlebot_sim2',
        executable='aruco_tf_listener',
        name='aruco_tf_listener',
        output='screen',
        parameters=[
            {'parent_frame': 'base_link'},
            {'child_frame': 'marker_0'},
            {'timer_period': 0.5},
        ],
    )

    return LaunchDescription([aruco_ekf, aruco_tf_listener])
