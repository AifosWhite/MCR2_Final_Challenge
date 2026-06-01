import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
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
      xacro    --> robot_state_publisher -> TF de los links --> RViz
    """
    pkg = get_package_share_directory('puzzlebot_sim2')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_physical.yaml')
    xacro_file = os.path.join(pkg, 'urdf', 'mcr2_robots',
                              'puzzlebot_jetson_lidar_ed.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    robot_desc = Command(['xacro ', xacro_file])

    # TF de los links del robot (en fisico: use_sim_time False).
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

    # Visualizacion (opcional).
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
        localisation,
        aruco_detector,
        bug_controller,
        rviz,
    ])
