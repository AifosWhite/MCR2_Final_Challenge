import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para la JETSON — nodos de navegación y sensores.
    NO lanza robot_state_publisher ni laser_frame_alias — esos van en la PC.
    Correr en la Jetson:
      ros2 launch final_challenge physical_jetson.launch.py
      ros2 launch final_challenge physical_jetson.launch.py nav:=false
    """
    pkg        = get_package_share_directory('final_challenge')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_physical.yaml')

    use_nav = LaunchConfiguration('nav')

    # Alias TF: base → base_footprint (necesario para aruco_ros marker_publisher)
    base_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base'],
        output='screen',
    )
    
    camera_alias = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_alias',
    arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera'],
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
        base_alias,
        camera_alias,
        joint_state_publisher,
        localisation,
        aruco_detector,
        bug_controller,
    ])