# MCR2 Final Challenge

Paquete ROS 2 sencillo para simular el Puzzlebot con:

- odometria por ruedas
- localizacion con covarianza y correccion EKF por ArUco
- LiDAR simulado del laberinto
- navegacion reactiva tipo Bug 2 con trayectoria cerrada de 4 puntos

## Archivos que si se usan

Estos son los importantes para el proyecto:

```text
package.xml
setup.py
setup.cfg
resource/puzzlebot_sim2
config/localisation.yaml
launch/final_challenge.launch.py
puzzlebot_sim2/simulator.py
puzzlebot_sim2/localisation.py
puzzlebot_sim2/sim_lidar_node.py
puzzlebot_sim2/sim_aruco_node.py
puzzlebot_sim2/reactive_navigation_node.py
puzzlebot_sim2/aruco_detector.py
puzzlebot_sim2/aruco_marker_bridge.py
worlds/maze.world
worlds/aruco_textures/
urdf/
meshes/
rviz/puzzlebot_desc.rviz
```

## Archivos que no necesitas tocar

Estos no son parte de la entrega principal o se generan solos:

```text
build/
install/
log/
__pycache__/
puzzlebot_sim/
launch/real_robot.launch.py
```

`puzzlebot_sim/` queda como referencia vieja de clase. Para correr este proyecto se usa `puzzlebot_sim2/`.

## Compilar

```bash
cd ~/MCR2_Final_Challenge
colcon build --packages-select puzzlebot_sim2 --symlink-install
source install/setup.bash
```

## Correr la simulacion

Simulacion limpia con nodos ROS:

```bash
ros2 launch puzzlebot_sim2 final_challenge.launch.py
```

Si tambien quieres abrir el mundo de Gazebo:

```bash
ros2 launch puzzlebot_sim2 final_challenge.launch.py use_gazebo:=true
```

## Revisar que esta corriendo

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /cmd_vel
```

La meta se recorre como trayectoria cerrada con 4 waypoints. Si quieres cambiarla, edita `waypoints_x` y `waypoints_y` en `reactive_navigation_node.py`.
