import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para la PC — robot_state_publisher + RViz + viz_debug.
    Los nodos de navegación corren en la Jetson (physical_jetson.launch.py).
    Correr en la PC:
      ros2 launch final_challenge physical_pc.launch.py
    """
    pkg         = get_package_share_directory('final_challenge')
    loc_params  = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    xacro_file  = os.path.join(pkg, 'urdf', 'mcr2_robots',
                               'puzzlebot_jetson_lidar_ed.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')

    use_rviz   = LaunchConfiguration('use_rviz')
    robot_desc = Command(['xacro ', xacro_file])

    # robot_state_publisher en la PC — genera paths locales para los .stl
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

    # Alias TF: laser_frame → laser (el rplidar publica en frame 'laser')
    laser_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'laser'],
        output='screen',
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
        robot_state_publisher,
        laser_frame_alias,
        viz_debug,
        rviz,
    ])