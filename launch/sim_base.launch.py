import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'final_challenge.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time},
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
            {'use_sim_time': use_sim_time},
        ],
    )

    aruco_ekf = Node(
        package='puzzlebot_sim2',
        executable='aruco_ekf_localisation',
        name='aruco_ekf_localisation',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'aruco_ekf.yaml'), {'use_sim_time': use_sim_time}],
    )

    joint_states = Node(
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'odom_topic': 'ekf_odom'}],
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
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    rqt_image = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    waypoint_driver = Node(
        package='puzzlebot_sim2',
        executable='waypoint_driver',
        name='waypoint_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    aruco_detector = Node(
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'image_topic': '/camera/image_raw'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from /clock.',
        ),
        robot_state_publisher,
        simulator,
        aruco_ekf,
        joint_states,
        waypoint_driver,
        aruco_detector,
        rviz,
        rqt_image,
    ])
