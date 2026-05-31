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
    world_file = os.path.join(pkg_dir, 'worlds', 'maze.world')
    use_camera_arucos = LaunchConfiguration('use_camera_arucos')
    use_gazebo = LaunchConfiguration('use_gazebo')

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
                    '-topic', 'robot_description',
                    '-name', 'puzzlebot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.05',
                ],
                output='screen',
            )
        ],
    )

    simulator = Node(
        package='puzzlebot_sim2',
        executable='simulator',
        name='puzzlebot_simulator',
        output='screen',
        parameters=[
            {'x0': 0.0},
            {'y0': 0.0},
            {'theta0': 0.0},
            {'wheel_radius': 0.05},
            {'wheel_base': 0.19},
        ],
    )

    localisation = Node(
        package='puzzlebot_sim2',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[loc_params],
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
        parameters=[{'bug_algorithm': 2}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera_arucos', default_value='false'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        simulator,
        localisation,
        sim_lidar,
        camera_bridge,
        aruco_detector,
        sim_aruco,
        reactive_navigation,
    ])
