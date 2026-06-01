from setuptools import setup
import os
from glob import glob

package_name = 'puzzlebot_sim2'


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
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        *package_files('meshes'),
        *package_files('urdf'),
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
            'localisation = puzzlebot_sim2.localisation:main',
            'bug_controller = puzzlebot_sim2.bug_controller:main',
            'aruco_detector_physical = puzzlebot_sim2.aruco_detector_physical:main',
            'viz_debug = puzzlebot_sim2.viz_debug:main',
        ],
    },
)
