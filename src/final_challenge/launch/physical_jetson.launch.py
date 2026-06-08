import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para la JETSON — nodos que consumen /scan localmente.
    Correr en la Jetson para evitar lag de LiDAR por WiFi.
    En la PC correr physical_pc.launch.py (solo RViz).
      ros2 launch final_challenge physical_jetson.launch.py
      ros2 launch final_challenge physical_jetson.launch.py nav:=false
    """
    pkg        = get_package_share_directory('final_challenge')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_physical.yaml')
    xacro_file = os.path.join(pkg, 'urdf', 'mcr2_robots',
                              'puzzlebot_jetson_lidar_ed.xacro')

    use_nav    = LaunchConfiguration('nav')
    robot_desc = Command(['xacro ', xacro_file])

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

    laser_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'laser'],
        output='screen',
    )

    joint_state_publisher = Node(
        package='final_challenge',
        executable='puzzlebot_joint_state_publisher',
        name='puzzlebot_joint_state_publisher',
        output='screen',
        parameters=[{
            'right_wheel_joint': 'wheel_right_joint',
            'left_wheel_joint':  'wheel_left_joint',
        }],
        remappings=[
            ('wr', 'VelocityEncR'),
            ('wl', 'VelocityEncL'),
        ],
    )

    localisation = Node(
        package='final_challenge',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params, {'use_ground_truth_pose': False}],
        remappings=[
            ('wr', 'VelocityEncR'),
            ('wl', 'VelocityEncL'),
        ],
    )

    aruco_detector = Node(
        package='final_challenge',
        executable='aruco_detector_physical',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
    )

    bug_controller = Node(
        condition=IfCondition(use_nav),
        package='final_challenge',
        executable='bug_controller',
        name='bug_controller',
        output='screen',
        parameters=[nav_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='true'),
        robot_state_publisher,
        joint_state_publisher,
        laser_frame_alias,
        localisation,
        aruco_detector,
        bug_controller,
    ])