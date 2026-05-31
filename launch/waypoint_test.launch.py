import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Default simple waypoint list; can be overridden via parameters.
    # Pass as strings to avoid launch normalization issues.
    waypoints = ['1.5,-0.5', '2.0,0.0', '2.0,2.0']



    waypoint_node = Node(
        package='puzzlebot_sim2',
        executable='waypoint_driver',
        name='waypoint_driver',
        output='screen',
    )

    # Minimal launch: only start the waypoint driver to isolate issues.
    return LaunchDescription([
        waypoint_node,
    ])
