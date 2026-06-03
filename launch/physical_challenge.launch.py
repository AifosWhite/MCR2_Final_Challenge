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
    pkg = get_package_share_directory('final_challenge')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_physical.yaml')
    xacro_file = os.path.join(pkg, 'urdf', 'mcr2_robots',
                              'puzzlebot_jetson_lidar_ed.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_nav = LaunchConfiguration('nav')
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

    # El rplidar fisico publica /scan en el frame 'laser', pero el URDF nombra el
    # frame del LiDAR 'laser_frame'. Sin este puente, RViz descarta el scan (no hay
    # TF a 'laser'). Alias identidad laser_frame -> laser para que /scan se vea.
    # (Alternativa: lanzar rplidar con frame_id:=laser_frame y quitar esto.)
    laser_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'laser'],
        output='screen',
    )

    # EKF / localizacion. Remapeamos wr/wl a los topicos reales del Puzzlebot.
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

    # Detector ArUco fisico (publica /aruco/detections para el EKF).
    aruco_detector = Node(
        package='final_challenge',
        executable='aruco_detector_physical',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
    )

    # Control Bug2 con seguimiento de pared (recorre la lista de waypoints).
    # Se puede apagar con nav:=false para probar SOLO localizacion.
    bug_controller = Node(
        condition=IfCondition(use_nav),
        package='final_challenge',
        executable='bug_controller',
        name='bug_controller',
        output='screen',
        parameters=[nav_params],
    )

    # Visualizacion de depuracion: markers de mapa/pose/lectura de ArUco + logs.
    viz_debug = Node(
        condition=IfCondition(use_rviz),
        package='final_challenge',
        executable='viz_debug',
        name='viz_debug',
        output='screen',
        parameters=[loc_params],
    )

    # RViz (opcional).
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
        DeclareLaunchArgument('nav', default_value='true'),
        robot_state_publisher,
        laser_frame_alias,
        localisation,
        aruco_detector,
        bug_controller,
        viz_debug,
        rviz,
    ])
