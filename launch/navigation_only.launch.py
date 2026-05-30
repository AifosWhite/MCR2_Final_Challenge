import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    nav_params = os.path.join(pkg_dir, 'config', 'final_bug_nav.yaml')

    return LaunchDescription([
        Node(
            package='puzzlebot_sim2',
            executable='final_bug_nav',
            name='final_bug_nav',
            output='screen',
            parameters=[nav_params],
        )
    ])
