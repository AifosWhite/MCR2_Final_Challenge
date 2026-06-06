import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para la PC — solo visualización.

    Los nodos de navegación y sensores corren en la Jetson
    (physical_jetson.launch.py).

      ros2 launch final_challenge physical_pc.launch.py
    """
    pkg         = get_package_share_directory('final_challenge')
    loc_params  = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

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
        viz_debug,
        rviz,
    ])
