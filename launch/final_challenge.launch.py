import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
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
                    '-x', '0.80',
                    '-y', '-1.05',
                    '-z', '0.05',
                    '-Y', '1.57',
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

    simulator = Node(
        package='puzzlebot_sim2',
        executable='simulator',
        name='puzzlebot_simulator',
        output='screen',
        parameters=[
            {'x0': 0.80},
            {'y0': -1.05},
            {'theta0': 1.57},
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
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
    )

    sim_lidar = Node(
        package='puzzlebot_sim2',
        executable='sim_lidar_node',
        name='sim_lidar_node',
        output='screen',
        parameters=[{'world_file': world_file}],
    )

    camera_bridge = ExecuteProcess(
        condition=IfCondition(use_camera_arucos),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
    )

    aruco_detector = Node(
        condition=IfCondition(use_camera_arucos),
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[loc_params],
    )

    sim_aruco = Node(
        condition=UnlessCondition(use_camera_arucos),
        package='puzzlebot_sim2',
        executable='sim_aruco_node',
        name='sim_aruco_node',
        output='screen',
    )

    reactive_navigation = Node(
        package='puzzlebot_sim2',
        executable='reactive_navigation_node',
        name='reactive_navigation_node',
        output='screen',
        parameters=[{
            'bug_algorithm': 2,
            'waypoints_x': [0.80, 1.20, 1.20, 0.80, 0.30, -0.30, -0.90, -1.25],
            'waypoints_y': [-0.70, -0.70, -1.15, -1.15, -0.70, -0.70, -0.70, -0.95],
            'goal_tolerance': 0.14,
            'max_linear_speed': 0.08,
            'max_angular_speed': 0.65,
            'wall_distance': 0.24,
            'front_clearance': 0.30,
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
        gazebo,
        robot_state_publisher,
        spawn_robot,
        cmd_vel_bridge,
        ground_truth_bridge,
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
