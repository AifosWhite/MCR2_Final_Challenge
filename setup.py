from setuptools import setup
import os
from glob import glob

package_name = 'final_challenge'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, f) for f in filenames]
        if files:
            paths.append((os.path.join('share', package_name, path), files))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=['final_challenge'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        *package_files('meshes'),
        *package_files('urdf'),
        *package_files('worlds'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sofia Blanco Prigmore',
    maintainer_email='sofiablancopr@gmail.com',
    description='Clean ROS2 package for the MCR2 Final Challenge using Puzzlebot simulation, localisation, RViz and final Bug navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson_aruco_adapter = final_challenge.jetson_aruco_adapter:main',

            'localisation = final_challenge.localisation:main',
            'bug_controller = final_challenge.bug_controller:main',
            'aruco_detector_physical = final_challenge.aruco_detector_physical:main',
            'viz_debug = final_challenge.viz_debug:main',
            'simulator = final_challenge.simulator:main',
            'sim_lidar_node = final_challenge.sim_lidar_node:main',
            'sim_aruco_node = final_challenge.sim_aruco_node:main',
            
        ],
    },
)
