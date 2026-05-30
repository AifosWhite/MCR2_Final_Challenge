import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'final_challenge.rviz')
    loc_params = os.path.join(pkg_dir, 'config', 'localisation.yaml')

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

    simulator = Node(
        package='puzzlebot_sim2',
        executable='simulator',
        name='puzzlebot_simulator',
        output='screen',
        parameters=[
            {'x0': 0.0},
            {'y0': 0.0},
            {'theta0': 0.0},
            {'wheel_radius': 0.05},
            {'wheel_base': 0.19},
        ],
    )

    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params],
    )

    joint_states = Node(
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
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
        simulator,
        localisation,
        joint_states,
        rviz,
    ])
