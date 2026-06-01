import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'mcr2_robots', 'puzzlebot_jetson_lidar_ed.xacro')
    loc_params = os.path.join(pkg_dir, 'config', 'localisation.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'puzzlebot_desc.rviz')
    world_file = os.path.join(pkg_dir, 'worlds', 'maze.world')
    use_camera_arucos = LaunchConfiguration('use_camera_arucos')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_dir, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )

    gazebo = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

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

    spawn_robot = TimerAction(
        condition=IfCondition(use_gazebo),
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', 'default',
                    '-topic', 'robot_description',
                    '-name', 'puzzlebot',
                    '-x', '-0.25',
                    '-y', '0.50',
                    '-z', '0.05',
                    '-Y', '-0.72',
                ],
                output='screen',
            )
        ],
    )

    cmd_vel_bridge = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    ground_truth_bridge = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    scan_bridge = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '--ros-args', '-r', '/scan:=/scan_gz',
        ],
        output='screen',
    )

    scan_timestamp = Node(
        condition=IfCondition(use_gazebo),
        package='puzzlebot_sim2',
        executable='scan_timestamp_node',
        name='scan_timestamp_node',
        output='screen',
    )

    simulator = Node(
        package='puzzlebot_sim2',
        executable='simulator',
        name='puzzlebot_simulator',
        output='screen',
        parameters=[
            {'x0': -0.25},
            {'y0': 0.50},
            {'theta0': -0.72},
            {'wheel_radius': 0.05},
            {'wheel_base': 0.19},
        ],
    )

    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params, {'use_ground_truth_pose': use_gazebo}],
    )

    joint_states = Node(
        condition=UnlessCondition(use_gazebo),
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
    )

    sim_lidar = Node(
        condition=UnlessCondition(use_gazebo),
        package='puzzlebot_sim2',
        executable='sim_lidar_node',
        name='sim_lidar_node',
        output='screen',
        parameters=[{'world_file': world_file}],
    )

    use_real_aruco_detector = PythonExpression([
        "'", use_camera_arucos, "' == 'true' or '", use_gazebo, "' == 'true'"
    ])

    camera_bridge = ExecuteProcess(
        condition=IfCondition(use_real_aruco_detector),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    aruco_detector = Node(
        condition=IfCondition(use_real_aruco_detector),
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
    )

    sim_aruco = Node(
        condition=IfCondition(PythonExpression(["'", use_camera_arucos, "' == 'false' and '", use_gazebo, "' == 'false'"])),
        package='puzzlebot_sim2',
        executable='sim_aruco_node',
        name='sim_aruco_node',
        output='screen',
    )

    # Bug navigation: lazo cerrado de waypoints con fallback de seguimiento de
    # pared. Waypoints centrados en el origen (frame EKF/odom) y validados
    # wall-safe contra maze.world (peor holgura de pasillo ~0.16 m para un robot
    # de ~0.13 m de radio). Forman un lazo por las cuatro esquinas, empezando y
    # terminando en el spawn (1.20, -1.00). NO los reemplaces por una marcha
    # recta por y~=0.55: ese camino cruza paredes interiores.
    reactive_navigation = Node(
        package='puzzlebot_sim2',
        executable='reactive_navigation_node',
        name='reactive_navigation_node',
        output='screen',
        parameters=[{
            'bug_mode': 2,
            'bug_direction': 'fwcw',
            'loop': True,
            # Ruta generada con A* + linea de vista sobre la geometria real del
            # maze (inflado 0.10 m). SOLO usa la zona alcanzable desde el spawn
            # (region superior): el laberinto es demasiado angosto para que el
            # robot llegue al resto. Verificado: 0 segmentos cruzan pared,
            # tramo promedio 0.30 m. Empieza en (-0.26,0.49) ~= spawn.
            'waypoints_x': [-0.26, -0.02, 0.16, 0.16, 0.19, 0.43, 0.82, 1.06,
                            0.88, 0.82, 0.40, 0.64, 0.49, 0.46, 0.43, 0.19,
                            0.16, 0.16, -0.26],
            'waypoints_y': [0.49, 0.28, 0.85, 1.09, 1.12, 1.12, 0.73, 0.73,
                            0.73, 0.73, 1.15, 0.88, 0.82, 1.09, 1.12, 1.12,
                            1.09, 0.37, 0.49],
            'goal_tolerance': 0.13,
            'max_linear_speed': 0.06,
            'max_angular_speed': 1.0,
            # Cono ANGOSTO + safety baja: como los waypoints ya van por el centro
            # del pasillo en linea de vista, el robot debe CONDUCIR RECTO de WP a
            # WP, no hacer wall-following (que en pasillos de doble pared zigzaguea
            # y roza). Solo entra a follow_wall ante un obstaculo real de frente.
            'ahead_clearance_angle_deg': 15.0,
            'goal_heading_clear_angle_deg': 15.0,
            'wall_follow_safety': 0.18,
            'line_distance_threshold_bug2': 0.15,
            'center_gain': 2.6,
            'center_trigger': 0.50,
            'front_slow_distance': 0.40,
        }],
    )

    visualization = Node(
        package='puzzlebot_sim2',
        executable='visualization_node',
        name='localisation_visualization',
        output='screen',
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
        DeclareLaunchArgument('use_camera_arucos', default_value='false'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        cmd_vel_bridge,
        ground_truth_bridge,
        scan_bridge,
        scan_timestamp,
        simulator,
        localisation,
        joint_states,
        sim_lidar,
        camera_bridge,
        aruco_detector,
        sim_aruco,
        reactive_navigation,
        visualization,
        rviz,
    ])
