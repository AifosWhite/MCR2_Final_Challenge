import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    world_file = os.path.join(pkg_dir, 'worlds', 'maze.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')

    # Add the local package share directory and the generated ArUco models path.
    local_gz_model_path = os.path.dirname(pkg_dir)
    marker_model_path = os.path.join(pkg_dir, 'models')
    current_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    paths = []
    if current_path:
        paths.append(current_path)
    paths.extend([pkg_dir, marker_model_path, local_gz_model_path])
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(paths)

    # Add known external model paths if available.
    puzzlebot_gazebo_models = os.path.expanduser(
        '~/robotec_sim_ws/install/puzzlebot_gazebo/share/puzzlebot_gazebo/models'
    )
    if os.path.isdir(puzzlebot_gazebo_models):
        current_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            current_path + ':' + puzzlebot_gazebo_models
            if current_path
            else puzzlebot_gazebo_models
        )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_puzzlebot',
        output='screen',
        arguments=[
            '-file', urdf_file,
            '-name', 'puzzlebot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0',
        ],
    )

    spawn_after_sim = TimerAction(
        period=2.0,
        actions=[spawn_robot],
    )

    return LaunchDescription([
        gz_sim_launch,
        spawn_after_sim,
    ])

