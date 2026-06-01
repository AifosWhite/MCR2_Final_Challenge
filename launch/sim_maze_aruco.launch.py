"""Bloque C2: maze con ArUco en Gazebo (solo world + robot + camara + aruco_opencv).

Reutiliza la infraestructura oficial de puzzlebot_gazebo:
  - gazebo_world_launch.py    -> arranca gz sim y fija GZ_SIM_RESOURCE_PATH a
                                 puzzlebot_gazebo/models (resuelve model://aruco_marker_N).
                                 Le pasamos el world por RUTA ABSOLUTA desde el repo
                                 (os.path.join colapsa al path absoluto), asi NO hay que
                                 modificar puzzlebot_gazebo ni copiar el world ahi.
  - gazebo_puzzlebot_launch.py -> spawnea puzzlebot_jetson_lidar_ed en (0,0) con sus
                                  bridges (scan, camera_info, cmd_vel, ...) e image_bridge
                                  (publica la imagen en /camera).

NO incluye EKF ni navegacion: este launch es solo para verificar deteccion ArUco.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    puzzlebot_gazebo = get_package_share_directory('puzzlebot_gazebo')
    pkg_share = get_package_share_directory('puzzlebot_sim2')

    # World del repo (ruta absoluta): gazebo_world_launch hace os.path.join(.,'worlds',world),
    # que con una ruta absoluta devuelve la ruta absoluta tal cual.
    world = os.path.join(pkg_share, 'worlds', 'maze_aruco.world')

    use_sim_time = 'true'

    # ---- Mundo (gz sim + GZ_SIM_RESOURCE_PATH + bridge de /clock y /tf) ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_gazebo, 'launch', 'gazebo_world_launch.py')
        ),
        launch_arguments={
            'world': world,
            'pause': 'false',
            'verbosity': '3',
        }.items(),
    )

    # ---- Robot puzzlebot_jetson_lidar_ed en (0,0) con camara + lidar + bridges ----
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_gazebo, 'launch', 'gazebo_puzzlebot_launch.py')
        ),
        launch_arguments={
            'robot': 'puzzlebot_jetson_lidar_ed',
            'robot_name': '',
            'x': '0.0',
            'y': '0.0',
            'yaw': '0.0',
            'prefix': '',
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ---- Deteccion ArUco (aruco_opencv) ----
    # image_bridge publica la imagen en /camera y camera_info en /camera_info.
    # aruco_opencv con cam_base_topic=camera busca camera/image_raw y camera/camera_info,
    # por eso los remapeamos a los topics reales.
    aruco_tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'cam_base_topic': 'camera',
            'marker_size': 0.14,
            'image_is_rectified': True,
            'marker_dict': '4X4_50',
            'use_sim_time': True,
        }],
        remappings=[
            ('camera/image_raw', '/camera'),
            ('camera/camera_info', '/camera_info'),
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        robot_launch,
        aruco_tracker,
    ])
