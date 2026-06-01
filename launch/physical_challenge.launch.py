import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento para el Puzzlebot FISICO (Ruta A).

    NO levanta Gazebo ni simuladores. Asume que el robot fisico ya publica:
      - VelocityEncR / VelocityEncL  (encoders, Float32)
      - scan                          (LiDAR real)
      - /video_source/raw             (camara)
    y que escucha cmd_vel.

    Pipeline:
      encoders --> localisation (EKF) --> odom
      camara   --> aruco_detector ------> /aruco/detections --> correccion EKF
      odom + scan --> bug_controller ----> cmd_vel
    """
    pkg = get_package_share_directory('puzzlebot_sim2')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_physical.yaml')

    # EKF / localizacion. Remapeamos wr/wl a los topicos reales del Puzzlebot.
    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params, {'use_ground_truth_pose': False}],
        remappings=[
            ('wr', 'VelocityEncR'),
            ('wl', 'VelocityEncL'),
        ],
    )

    # Detector ArUco fisico (publica /aruco/detections para el EKF).
    aruco_detector = Node(
        package='puzzlebot_sim2',
        executable='aruco_detector_physical',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
    )

    # Control Bug2 con seguimiento de pared (recorre la lista de waypoints).
    bug_controller = Node(
        package='puzzlebot_sim2',
        executable='bug_controller',
        name='bug_controller',
        output='screen',
        parameters=[nav_params],
    )

    return LaunchDescription([
        localisation,
        aruco_detector,
        bug_controller,
    ])
