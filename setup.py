from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sofia Blanco Prigmore',
    maintainer_email='sofiablancopr@gmail.com',
    description='ROS2 package for the MCR2 Final Challenge: Puzzlebot model, localisation, ArUco EKF scaffold and Bug navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = puzzlebot_sim2.simulator:main',
            'localisation = puzzlebot_sim2.localisation:main',
            'joint_states = puzzlebot_sim2.joint_states:main',
            'final_bug_nav = puzzlebot_sim2.final_bug_nav:main',
            'aruco_ekf_localisation = puzzlebot_sim2.aruco_ekf_localisation:main',
            'aruco_tf_listener = puzzlebot_sim2.aruco_tf_listener:main',
            'fake_scan = puzzlebot_sim2.fake_scan:main',
            'waypoint_driver = puzzlebot_sim2.waypoint_driver:main',
        ],
    },
)
