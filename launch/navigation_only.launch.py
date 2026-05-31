import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    nav_params = os.path.join(pkg_dir, 'config', 'final_bug_nav.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time from /clock.',
        ),
        Node(
            package='puzzlebot_sim2',
            executable='final_bug_nav',
            name='final_bug_nav',
            output='screen',
            parameters=[nav_params, {'use_sim_time': use_sim_time}],
        )
    ])
