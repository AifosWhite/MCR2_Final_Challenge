import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para el Puzzlebot FISICO — todo en una sola máquina.
    Para el setup Jetson+PC separados usar:
      physical_jetson.launch.py  (en la Jetson)
      physical_pc.launch.py      (en la PC)
    """
    pkg         = get_package_share_directory('final_challenge')
    loc_params  = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params  = os.path.join(pkg, 'config', 'navigation_physical.yaml')
    xacro_file  = os.path.join(pkg, 'urdf', 'mcr2_robots',
                               'puzzlebot_jetson_lidar_ed.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')

    use_rviz   = LaunchConfiguration('use_rviz')
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

    viz_debug = Node(
        condition=IfCondition(use_rviz),
        package='final_challenge',
        executable='viz_debug',
        name='viz_debug',
        output='screen',
        parameters=[loc_params],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('nav',      default_value='true'),
        robot_state_publisher,
        joint_state_publisher,
        laser_frame_alias,
        localisation,
        aruco_detector,
        bug_controller,
        viz_debug,
        rviz,
    ])