import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'final_challenge.rviz')
    nav_params = os.path.join(pkg_dir, 'config', 'final_bug_nav.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'aruco_ekf.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False},
        ],
    )

    final_bug_nav = Node(
        package='puzzlebot_sim2',
        executable='final_bug_nav',
        name='final_bug_nav',
        output='screen',
        parameters=[nav_params],
    )

    aruco_ekf = Node(
        package='puzzlebot_sim2',
        executable='aruco_ekf_localisation',
        name='aruco_ekf_localisation',
        output='screen',
        parameters=[ekf_params],
    )

    rviz = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': False}],
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        final_bug_nav,
        aruco_ekf,
        rviz,
    ])
