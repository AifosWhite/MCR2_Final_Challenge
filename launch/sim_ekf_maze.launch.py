"""Stack EKF + ArUco sobre el maze de Gazebo (SIN navegacion).

Reutiliza la infra de puzzlebot_gazebo (world + robot con camara/lidar) y le anade:
  - aruco_detector  : detecta ArUco en /camera y publica /aruco/detections [id,dist,bearing].
  - localisation    : odometria por encoders (/VelocityEncR, /VelocityEncL) + correccion EKF
                      con /aruco/detections. Publica /odom con covarianza.
  - covariance_marker: elipse de covarianza para RViz.

El robot se maneja con teleop_twist_keyboard (/cmd_vel). No hay final_bug_nav aqui.
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

    world = os.path.join(pkg_share, 'worlds', 'maze_aruco.world')
    loc_params = os.path.join(pkg_share, 'config', 'localisation_sim.yaml')

    # ---- Mundo (gz sim + GZ_SIM_RESOURCE_PATH + /clock,/tf) ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_gazebo, 'launch', 'gazebo_world_launch.py')
        ),
        launch_arguments={'world': world, 'pause': 'false', 'verbosity': '3'}.items(),
    )

    # ---- Robot puzzlebot_jetson_lidar_ed en (0,0): camara, lidar, encoders, cmd_vel ----
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puzzlebot_gazebo, 'launch', 'gazebo_puzzlebot_launch.py')
        ),
        launch_arguments={
            'robot': 'puzzlebot_jetson_lidar_ed',
            'robot_name': '',
            'x': '0.0', 'y': '0.0', 'yaw': '0.0',
            'prefix': '',
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link',
            'use_sim_time': 'true',
        }.items(),
    )

    # ---- Deteccion ArUco (alimenta el EKF) ----
    # La camara del robot Gazebo: 640x480, hfov=1.089 -> fx=fy~528.5, cx=320, cy=240.
    # marker_size_m = lado del cuadro negro detectado (0.10 m).
    aruco_detector = Node(
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_size_m': 0.10,
            'fx': 528.5, 'fy': 528.5, 'cx': 320.0, 'cy': 240.0,
            'use_sim_time': True,
        }],
        remappings=[('/image_raw', '/camera')],
    )

    # ---- Localizacion EKF ----
    # Encoders Gazebo: /VelocityEncR -> 'wr', /VelocityEncL -> 'wl'.
    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params, {'use_sim_time': True}],
        remappings=[('wr', '/VelocityEncR'), ('wl', '/VelocityEncL')],
    )

    covariance_marker = Node(
        package='puzzlebot_sim2',
        executable='covariance_marker',
        name='covariance_marker',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch,
        robot_launch,
        aruco_detector,
        localisation,
        covariance_marker,
    ])
