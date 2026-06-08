import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lanzamiento de la simulacion Python pura para el MCR2 Final Challenge.

    NO requiere Gazebo ni robot fisico. El stack completo corre en la PC:

      simulator      --> publica wr/wl y la pose real (sim_pose_odom)
      sim_lidar_node --> raycast contra paredes del maze --> /scan
      sim_aruco_node --> detecta marcadores en el FOV   --> /aruco/detections
      localisation   --> EKF con dead-reckoning + correccion ArUco --> /odom
      bug_controller --> Bug2 + wall-following --> /cmd_vel
      viz_debug      --> markers de mapa, covarianza, trayectoria en RViz

    Uso:
    ros2 launch final_challenge sim_challenge.launch.py
    ros2 launch final_challenge sim_challenge.launch.py nav:=false  # solo EKF
    ros2 launch final_challenge sim_challenge.launch.py use_rviz:=false
    ros2 launch final_challenge sim_challenge.launch.py use_gazebo:=false
    ros2 launch final_challenge sim_challenge.launch.py use_gazebo:=false use_python_sim:=true
    ros2 launch final_challenge sim_challenge.launch.py start_x:=1.20 start_y:=-0.30 initial_mode:=following_walls
    """
    pkg = get_package_share_directory('final_challenge')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    loc_params = os.path.join(pkg, 'config', 'localisation_physical.yaml')
    nav_params = os.path.join(pkg, 'config', 'navigation_sim.yaml')
    xacro_file = os.path.join(pkg, 'urdf', 'mcr2_robots',
                              'puzzlebot_jetson_lidar_ed.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'puzzlebot_desc.rviz')
    world_file = os.path.join(pkg, 'worlds', 'maze.world')
    gz_launch = os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_python_sim = LaunchConfiguration('use_python_sim')
    use_nav = LaunchConfiguration('nav')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    initial_mode = LaunchConfiguration('initial_mode')
    wall_side = LaunchConfiguration('wall_side')
    robot_desc = Command(['xacro ', xacro_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        condition=IfCondition(use_gazebo),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    gazebo_spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                condition=IfCondition(use_gazebo),
                package='ros_gz_sim',
                executable='create',
                name='spawn_puzzlebot',
                output='screen',
                arguments=[
                    '-world', 'default',
                    '-param', 'robot_description',
                    '-name', 'puzzlebot',
                    '-allow_renaming', 'true',
                    '-x', start_x,
                    '-y', start_y,
                    '-z', '0.08',
                    '-Y', start_yaw,
                ],
                parameters=[{'robot_description': robot_desc}],
            ),
        ],
    )

    gazebo_bridge = Node(
        condition=IfCondition(use_gazebo),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    # Simulador diferencial: integra cmd_vel -> cinematica -> publica wr/wl
    # y la pose real del robot (sim_pose_odom). Inicia en la posicion fisica
    # real del robot dentro del laberinto.
    simulator = Node(
        condition=IfCondition(use_python_sim),
        package='final_challenge',
        executable='simulator',
        name='puzzlebot_simulator',
        output='screen',
        parameters=[{
            'x0': start_x,
            'y0': start_y,
            'theta0': start_yaw,
            'wheel_radius': 0.05,
            'wheel_base': 0.19,
            'update_rate': 50.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )

    # LiDAR simulado: lanza rayos desde la pose REAL del robot contra las
    # paredes del laberinto. Remap odom -> sim_pose_odom para usar la pose
    # verdadera (no la estimada por el EKF) al calcular los rayos.
    sim_lidar = Node(
        condition=IfCondition(use_python_sim),
        package='final_challenge',
        executable='sim_lidar_node',
        name='sim_lidar_node',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'use_physical_frame': True,
            'range_max': 3.0,
            'samples': 360,
            'update_rate': 10.0,
        }],
        remappings=[('odom', 'sim_pose_odom')],
    )

    # Detector ArUco simulado: dada la pose real del robot, calcula que
    # marcadores son visibles (dentro del FOV y sin oclusion). Publica
    # [id, distancia, bearing] igual que el detector fisico.
    sim_aruco = Node(
        condition=IfCondition(use_python_sim),
        package='final_challenge',
        executable='sim_aruco_node',
        name='sim_aruco_node',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'use_physical_frame': True,
            'pose_topic': 'sim_pose_odom',
            'detections_topic': '/aruco/detections',
            'fov_deg': 60.0,
            'max_range': 2.5,
            'min_range': 0.15,
            'check_occlusion': True,
            'marker_ids': [70, 75, 701, 702, 703, 705, 706, 708],
            'marker_pos_x': [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185],
            'marker_pos_y': [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21],
        }],
    )

    # EKF / localizacion: mismos parametros y marcadores que en el robot
    # fisico. Para probar localizacion pura sin navegacion: nav:=false.
    # Remap /ground_truth -> sim_pose_odom por si se quiere activar
    # use_ground_truth_pose: true para depurar.
    localisation = Node(
        package='final_challenge',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params, {
            'use_ground_truth_pose': use_gazebo,
            'x0': start_x,
            'y0': start_y,
            'theta0': start_yaw,
            'initial_x': start_x,
            'initial_y': start_y,
            'initial_theta': start_yaw,
        }],
    )

    # Control Bug2 con wall-following. Solo activo con nav:=true (por defecto).
    bug_controller = Node(
        condition=IfCondition(use_nav),
        package='final_challenge',
        executable='bug_controller',
        name='bug_controller',
        output='screen',
        parameters=[nav_params, {
            'initial_controller_mode': initial_mode,
            'initial_wall_side': wall_side,
        }],
    )

    # TF de los links del robot desde URDF (ruedas, lidar, etc.).
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

    # Visualizacion: marcadores del mapa, elipse de covarianza, trayectoria,
    # linea de medicion ArUco. Solo activo con use_rviz:=true (por defecto).
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
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_python_sim', default_value='false'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('start_x', default_value='1.20'),
        DeclareLaunchArgument('start_y', default_value='-0.30'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_mode', default_value='following_walls'),
        DeclareLaunchArgument('wall_side', default_value='right'),
        gazebo,
        gazebo_spawn_robot,
        gazebo_bridge,
        simulator,
        sim_lidar,
        sim_aruco,
        localisation,
        bug_controller,
        robot_state_publisher,
        viz_debug,
        rviz,
    ])
