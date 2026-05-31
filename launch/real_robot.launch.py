import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'mcr2_robots', 'puzzlebot_jetson_lidar_ed.xacro')
    loc_params = os.path.join(pkg_dir, 'config', 'localisation.yaml')

    robot_desc = Command(['xacro ', xacro_file])

    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    wr_topic = LaunchConfiguration('wr_topic')
    wl_topic = LaunchConfiguration('wl_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_marker_publisher = LaunchConfiguration('use_marker_publisher')
    publish_robot_description = LaunchConfiguration('publish_robot_description')
    publish_joint_tf = LaunchConfiguration('publish_joint_tf')

    robot_state_publisher = Node(
        condition=IfCondition(publish_robot_description),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False},
        ],
    )

    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params],
        remappings=[
            ('wr', wr_topic),
            ('wl', wl_topic),
            ('odom', odom_topic),
            ('/aruco/detections', '/aruco/detections'),
        ],
    )

    aruco_detector = Node(
        condition=UnlessCondition(use_marker_publisher),
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
        remappings=[
            ('/image_raw', image_topic),
            ('/aruco/detections', '/aruco/detections'),
        ],
    )

    aruco_marker_bridge = Node(
        condition=IfCondition(use_marker_publisher),
        package='puzzlebot_sim2',
        executable='aruco_marker_bridge',
        name='aruco_marker_bridge',
        output='screen',
        parameters=[
            {
                'markers_topic': '/marker_publisher/markers',
                'detections_topic': '/aruco/detections',
            },
        ],
    )

    joint_states = Node(
        condition=IfCondition(publish_joint_tf),
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
        remappings=[
            ('odom', odom_topic),
            ('wr', wr_topic),
            ('wl', wl_topic),
        ],
    )

    reactive_navigation = Node(
        package='puzzlebot_sim2',
        executable='reactive_navigation_node',
        name='reactive_navigation_node',
        output='screen',
        parameters=[
            {
                'bug_algorithm': 2,
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'cmd_vel_topic': cmd_vel_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('wr_topic', default_value='/wr'),
        DeclareLaunchArgument('wl_topic', default_value='/wl'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_marker_publisher', default_value='true'),
        DeclareLaunchArgument('publish_robot_description', default_value='true'),
        DeclareLaunchArgument('publish_joint_tf', default_value='true'),
        robot_state_publisher,
        localisation,
        aruco_detector,
        aruco_marker_bridge,
        joint_states,
        reactive_navigation,
    ])
