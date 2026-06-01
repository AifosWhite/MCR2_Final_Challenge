#!/usr/bin/env python3
"""Clean final challenge launch.

This is intentionally conservative. It does NOT include sim_base.launch.py and
therefore does not start the old internal waypoint driver / full EKF stack
by accident.

Default workflow:
- Gazebo shows the maze and ArUco marker models.
- ROS nodes run a light closed-loop simulation: simulator -> localisation ->
  sim_lidar/sim_aruco -> reactive_navigation -> cmd_vel.
- RViz is optional and can be launched only when needed.

This gives a stable base to debug navigation, covariance and ArUco logic before
moving back to the real robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    world_file = os.path.join(pkg_dir, 'worlds', 'maze.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'final_challenge_clean.rviz')
    loc_params = os.path.join(pkg_dir, 'config', 'localisation.yaml')
    nav_params = os.path.join(pkg_dir, 'config', 'reactive_navigation.yaml')
    viz_params = os.path.join(pkg_dir, 'config', 'visualization.yaml')

    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera_arucos = LaunchConfiguration('use_camera_arucos')
    use_sim_aruco = LaunchConfiguration('use_sim_aruco')
    use_navigation = LaunchConfiguration('use_navigation')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    resource_path = ':'.join([
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        pkg_dir,
        os.path.join(pkg_dir, 'models'),
        os.path.dirname(pkg_dir),
    ])

    gazebo = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Optional visual spawn. The algorithm does not depend on Gazebo physics here;
    # it depends on the ROS simulator/localisation chain below.
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
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.05',
                    '-Y', '0.0',
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

    joint_states = Node(
        package='puzzlebot_sim2',
        executable='joint_states',
        name='joint_states',
        output='screen',
        parameters=[
            {'odom_topic': 'odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
        ],
    )

    sim_lidar = Node(
        package='puzzlebot_sim2',
        executable='sim_lidar_node',
        name='sim_lidar_node',
        output='screen',
        parameters=[{'world_file': world_file}],
    )

    sim_aruco = Node(
        condition=IfCondition(use_sim_aruco),
        package='puzzlebot_sim2',
        executable='sim_aruco_node',
        name='sim_aruco_node',
        output='screen',
    )

    # Camera ArUco is optional because it is heavier. Use only when the camera
    # image topic is confirmed and the simulation is stable.
    camera_bridge = ExecuteProcess(
        condition=IfCondition(use_camera_arucos),
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    aruco_detector = Node(
        condition=IfCondition(use_camera_arucos),
        package='puzzlebot_sim2',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'image_topic': '/camera/image_raw'},
            {'dictionary': 'DICT_4X4_250'},
            {'marker_length': 0.165},
        ],
    )

    reactive_navigation = Node(
        condition=IfCondition(use_navigation),
        package='puzzlebot_sim2',
        executable='reactive_navigation_node',
        name='reactive_navigation_node',
        output='screen',
        parameters=[nav_params],
    )

    visualization = Node(
        package='puzzlebot_sim2',
        executable='visualization_node',
        name='localisation_visualization',
        output='screen',
        parameters=[viz_params],
    )

    rviz = TimerAction(
        condition=IfCondition(use_rviz),
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_camera_arucos', default_value='false'),
        DeclareLaunchArgument('use_sim_aruco', default_value='true'),
        DeclareLaunchArgument('use_navigation', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        simulator,
        localisation,
        joint_states,
        sim_lidar,
        sim_aruco,
        camera_bridge,
        aruco_detector,
        reactive_navigation,
        visualization,
        rviz,
    ])
